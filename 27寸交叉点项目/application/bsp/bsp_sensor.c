// bsp_sensor.c
#include "bsp_sensor.h"
#include "bsp_adc.h"
#include "bsp_tim.h"
#include "tim.h"
#include "main.h"
#include "config.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

// 背景噪声（暗电流/跨组）：bg[0]=ADC1原始/包络, bg[1]=ADC2 TIA
static int16_t bg[2][SEG_SENSOR_COUNT];

// 下层通道选择（SCTRL）：SCTRL_A=PA4 SCTRL_B=PA3 SCTRL_C=PA2
#define S_MASK   (SCTRL_A_Pin | SCTRL_B_Pin | SCTRL_C_Pin)
// 上层 bank 选择（ACTRL）：ACTRL_A=PA9 ACTRL_B=PA8 ACTRL_C=PB15

static void mux_select(uint8_t upper, uint8_t lower)
{
    uint16_t val;

    // 下层（通道）
    val = 0;
    if (lower & 0x01) val |= SCTRL_A_Pin;
    if (lower & 0x02) val |= SCTRL_B_Pin;
    if (lower & 0x04) val |= SCTRL_C_Pin;
    HAL_GPIO_WritePin(SCTRL_A_GPIO_Port, S_MASK, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SCTRL_A_GPIO_Port, val, GPIO_PIN_SET);

    // 上层（bank）
    val = 0;
    if (upper & 0x01) val |= ACTRL_A_Pin;
    if (upper & 0x02) val |= ACTRL_B_Pin;
    HAL_GPIO_WritePin(ACTRL_A_GPIO_Port, ACTRL_A_Pin | ACTRL_B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTRL_C_GPIO_Port, ACTRL_C_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACTRL_A_GPIO_Port, val, GPIO_PIN_SET);
    if (upper & 0x04) HAL_GPIO_WritePin(ACTRL_C_GPIO_Port, ACTRL_C_Pin, GPIO_PIN_SET);
}

// 开机背景校准：全通道扫约 1 秒求平均，存为背景
// 注意：建立时间必须与 bsp_sensor_acquire 一致（同一 MUX_STABLE_CUR）！
//   ENV(包络)路若建立时间不足，读到的是上一通道切换激励带通后的振铃(≈330µs 衰减),
//   bg 被抬高 → 采集时(等足时间的真实值)减成 0, 表现为"全零"
void bsp_sensor_calibrate(void)
{
    uint32_t sum[2][SEG_SENSOR_COUNT];
    memset(sum, 0, sizeof(sum));
    uint32_t n = 0;
    uint32_t t0 = HAL_GetTick();
    uint8_t last_upper_c = 0xFF;

    while (HAL_GetTick() - t0 < 1000) {
        for (uint8_t upper = 0; upper < MUX_UPPER_MAX; upper++) {
            for (uint8_t lower = 0; lower < MUX_LOWER_MAX; lower++) {
                if (upper == MUX_UPPER_MAX - 1 && lower >= SEG_TAIL_COUNT) continue;

                mux_select(upper, lower);
                if (upper != last_upper_c) {
                    bsp_delay_us(MUX_BANK_EXTRA_US);
                    last_upper_c = upper;
                }
                bsp_delay_us(MUX_STABLE_CUR);
                uint16_t idx = (uint16_t)upper * MUX_LOWER_MAX + lower;

                // 丢弃前几次采样（消除电荷注入）
                for (int i = 0; i < ADC_DISCARD_COUNT; i++) {
                    bsp_adc1_read();
                    bsp_adc2_read();
                }

                uint32_t s0 = 0, s1 = 0;
                for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
                    s0 += bsp_adc1_read();
                    s1 += bsp_adc2_read();
                }
                bsp_delay_us(1);

                /* 两路背景都测（TIA 路和包络路各自要用） */
                sum[0][idx] += s0 / ADC_SAMPLE_COUNT;
                sum[1][idx] += s1 / ADC_SAMPLE_COUNT;
            }
        }
        n++;
    }

    for (uint16_t i = 0; i < SEG_SENSOR_COUNT; i++) {
        bg[0][i] = (int16_t)(sum[0][i] / n);
        bg[1][i] = (int16_t)(sum[1][i] / n);
    }
}

void bsp_sensor_init(void)
{
    bsp_adc_init();
    bsp_tim_init();
#if TEST_FIXED_SENSOR
    // 临时调试：固定拨通一路，mux 保持不动
    mux_select(TEST_UPPER, TEST_LOWER);
#else
    bsp_sensor_calibrate();
#endif
    bsp_sensor_enable_default_long();   /* 所有板统一 17974Hz / 4 周期 / 相位 19µs */
}

#if TEST_FIXED_SENSOR
#include "usart.h"

// 诊断用：全通道 DC 扫描（不解调）
// 每通道打印: idx  ADC1均值 ADC1最小 ADC1最大  ADC2均值 ADC2最小 ADC2最大
// 判断方法:
//   光斑照到某通道 -> 对应 ADC 均值抬升
//   18kHz 调制光   -> 对应 ADC 的 max-min 明显大于暗底噪声
void bsp_sensor_debug_dump(void)
{
    char line[72];
    uint8_t last_upper = 0xFF;
    static const char hdr[] = "--- DC scan: idx a1avg a1min a1max a2avg a2min a2max ---\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)hdr, sizeof(hdr) - 1, 100);

    for (uint8_t upper = 0; upper < MUX_UPPER_MAX; upper++) {
        for (uint8_t lower = 0; lower < MUX_LOWER_MAX; lower++) {
            if (upper == MUX_UPPER_MAX - 1 && lower >= SEG_TAIL_COUNT) continue;

            mux_select(upper, lower);
            if (upper != last_upper) {
                bsp_delay_us(MUX_BANK_EXTRA_US);
                last_upper = upper;
            }
            bsp_delay_us(MUX_STABLE_CUR);
            uint16_t idx = (uint16_t)upper * MUX_LOWER_MAX + lower;

            for (int i = 0; i < ADC_DISCARD_COUNT; i++) {
                bsp_adc1_read();
                bsp_adc2_read();
            }

            uint32_t s1 = 0, s2 = 0;
            uint16_t mn1 = 0xFFFF, mx1 = 0, mn2 = 0xFFFF, mx2 = 0;
            for (int i = 0; i < 64; i++) {
                uint16_t v1 = bsp_adc1_read();
                uint16_t v2 = bsp_adc2_read();
                s1 += v1; s2 += v2;
                if (v1 > mx1) mx1 = v1;
                if (v1 < mn1) mn1 = v1;
                if (v2 > mx2) mx2 = v2;
                if (v2 < mn2) mn2 = v2;
            }

            int n = sprintf(line, "D %02u %4u %4u %4u %4u %4u %4u\r\n",
                            (unsigned)idx, (unsigned)(s1 / 64), (unsigned)mn1, (unsigned)mx1,
                            (unsigned)(s2 / 64), (unsigned)mn2, (unsigned)mx2);
            HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)n, 100);
        }
    }
}
#endif

#if DEMOD_18K_ENABLE
// ==================== 18kHz 锁相解调（2µs 均匀密采, 28点/54µs）====================
// 采样点时刻表(µs, 1µs 定时器网格): 2µs 均匀, 窗口 54µs(≈0.97 周期) ——
//   原 16 点(3/4µs 稀疏)在窄脉冲光源下, 解调增益随"脉冲/采样相对相位"漂移
//   (实测幅值呈乘法噪声: 16% 乘法 + 10LSB 加法, 且与扫描时间相关的倾斜);
//   同样 54µs 窗口内加密到 28 点, 相干累加更接近连续积分, 相位敏感降约 2 倍。
// 参考表: cos/sin(2π*18000*t_n) × 18864, t_n = demod_at[], 运行时无浮点/三角函数。
//   表幅值按 8.06/14.0 缩放, 使 I/Q>>15 的输出尺度与旧 16 点表一致(阈值/显示不用改)。
static const uint16_t demod_at[DEMOD_SAMPLES] = {
     3,  5,  7,  9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29,
    31, 33, 35, 37, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57 };
static const int16_t ref_cos[DEMOD_SAMPLES] = {
     17789,  15927,  13255,   9907,   6054,   1893,  -2364,
     -6501, -10307, -13588, -16176, -17941, -18791, -18684,
    -17625, -15668, -12913,  -9500,  -5603,  -1421,   2834,
      6944,  10701,  13912,  16415,  18082,  18827,  18613 };
static const int16_t ref_sin[DEMOD_SAMPLES] = {
      6278,  10108,  13422,  16053,  17866,  18769,  18715,
     17708,  15799,  13085,   9704,   5829,   1657,  -2599,
     -6723, -10505, -13751, -16297, -18013, -18810, -18650,
    -17539, -15535, -12739,  -9295,  -5377,  -1184,   3068 };

// ==================== 自适应双相锁相（默认 17974Hz / 4 周期；测频/相位扫描仅主控调用）====================
static uint32_t g_laser_freq_hz;
static uint16_t g_laser_peak_idx = 0xFFFF;
static uint8_t  g_laser_ready;

static uint16_t demod_at_rt[DEMOD_ADAPT_SAMPLES];
static int16_t  ref_cos_rt[DEMOD_ADAPT_SAMPLES];
static int16_t  ref_sin_rt[DEMOD_ADAPT_SAMPLES];
#if DEMOD_SPECTRAL_GUARD_ENABLE
static int16_t  ref_cos_guard_rt[DEMOD_ADAPT_SAMPLES];
static int16_t  ref_sin_guard_rt[DEMOD_ADAPT_SAMPLES];
#endif
static uint8_t  g_demod_phase_us = 19;   /* 1 周期标定得到的脉冲相位（4 周期沿用，激光开时自动重扫） */

uint32_t bsp_sensor_get_laser_freq(void) { return g_laser_freq_hz; }
uint16_t bsp_sensor_get_laser_peak_idx(void) { return g_laser_peak_idx; }

static void build_adapt_tables(uint32_t f_hz)
{
    /* 参考表按 28/111 缩放，使长窗口输出尺度与原来 28 点一致 */
    float scale = 18864.0f;
    if (DEMOD_ADAPT_SAMPLES > DEMOD_SAMPLES) {
        scale *= (float)DEMOD_SAMPLES / (float)DEMOD_ADAPT_SAMPLES;
    }
    for (uint16_t n = 0; n < DEMOD_ADAPT_SAMPLES; n++) {
        uint16_t t = (uint16_t)(3u + g_demod_phase_us + 2u * n);
        demod_at_rt[n] = t;
        float ph = 6.28318530718f * (float)f_hz * (float)t * 1e-6f;
        ref_cos_rt[n] = (int16_t)(cosf(ph) * scale);
        ref_sin_rt[n] = (int16_t)(sinf(ph) * scale);
#if DEMOD_SPECTRAL_GUARD_ENABLE
        /* One DFT bin above the target across the complete window.  It is
           orthogonal to the target for a periodic signal, while switching
           impulses and ADC noise excite both bins. */
        {
            float guard_hz = (float)f_hz *
                             (1.0f + 1.0f / (float)DEMOD_ADAPT_CYCLES);
            float pg = 6.28318530718f * guard_hz * (float)t * 1e-6f;
            ref_cos_guard_rt[n] = (int16_t)(cosf(pg) * scale);
            ref_sin_guard_rt[n] = (int16_t)(sinf(pg) * scale);
        }
#endif
    }
    g_laser_freq_hz = f_hz;
    g_laser_ready = 1;
}

/* 无激光时也先用 18kHz 的 4 周期长窗口；测到真实激光频率后再覆盖 */
void bsp_sensor_enable_default_long(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    build_adapt_tables(DEMOD_FREQ_HZ);
    g_laser_freq_hz = 0;   /* 0 表示尚未测到真实激光频率 */
}

/* 在指定传感器上采 10ms TIA 原始波形，过零检测估频；成功则重建参考表 */
uint32_t bsp_sensor_measure_laser(uint16_t idx)
{
    if (idx >= MY_SENSOR_COUNT) return 0;

#if MY_MIRROR
    idx = (uint16_t)(MY_SENSOR_COUNT - 1 - idx);
#endif

    uint8_t upper = (uint8_t)(idx / MUX_LOWER_MAX);
    uint8_t lower = (uint8_t)(idx % MUX_LOWER_MAX);
    uint32_t f_hz = 0;

    __disable_irq();
    mux_select(upper, lower);
    bsp_delay_us(MUX_BANK_EXTRA_US);
    bsp_delay_us(MUX_STABLE_CUR);
    for (int i = 0; i < ADC_DISCARD_COUNT; i++) bsp_adc2_read();

    /* 第一遍：1ms，测峰峰值 */
    uint16_t mn = 0xFFFF, mx = 0;
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    for (int n = 0; n < 512; n++) {
        uint32_t target = (uint32_t)(3 + 2 * n);
        uint32_t guard = 1000000u;
        while ((uint32_t)__HAL_TIM_GET_COUNTER(&htim1) < target && guard--) { }
        if (guard == 0u) { __enable_irq(); return 0u; }
        uint16_t v = bsp_adc2_read();
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }

    uint16_t amp = (uint16_t)(mx - mn);
    if (amp >= 200) {
        int32_t mid = ((int32_t)mn + (int32_t)mx) / 2;
        int32_t hys = (int32_t)amp / 4;
        int32_t prev = 0;
        uint8_t have_prev = 0, armed = 1;
        int count = 0;
        uint32_t t_first = 0, t_last = 0;

        /* 第二遍：10ms，带迟滞的上升过零计数 */
        __HAL_TIM_SET_COUNTER(&htim1, 0);
        for (int n = 0; n < 5000; n++) {
            uint32_t target = (uint32_t)(3 + 2 * n);
            uint32_t guard = 1000000u;
            while ((uint32_t)__HAL_TIM_GET_COUNTER(&htim1) < target && guard--) { }
            if (guard == 0u) { __enable_irq(); return 0u; }
            int32_t v = (int32_t)bsp_adc2_read();

            if (have_prev) {
                if (armed && v > mid + hys) {
                    uint32_t tp = (uint32_t)(3 + 2 * (n - 1));
                    uint32_t cross = tp;
                    int32_t dv = v - prev;
                    if (dv > 0) {
                        int32_t num = mid - prev;
                        if (num < 0) num = 0;
                        if (num > dv) num = dv;
                        cross = tp + (uint32_t)(((uint64_t)num * 2u) / (uint32_t)dv);
                    }
                    if (count == 0) t_first = cross;
                    t_last = cross;
                    count++;
                    armed = 0;
                } else if (!armed && v < mid - hys) {
                    armed = 1;
                }
            }
            prev = v;
            have_prev = 1;
        }

        if (count >= 8 && t_last > t_first) {
            uint64_t num = (uint64_t)(count - 1) * 1000000ULL;
            uint64_t den = (uint64_t)(t_last - t_first);
            f_hz = (uint32_t)((num + den / 2) / den);
        }
    }
    __enable_irq();

    if (f_hz < 10000u || f_hz > 30000u) return 0;

    build_adapt_tables(f_hz);
    g_laser_peak_idx = idx;
    return f_hz;
}

// 对单通道做 18kHz 同步解调, 返回调制光幅值（与 v2 同尺度）
// dc_offset: 用背景值把直流去掉，保证 int32 不溢出
// 去均值: 只对窗口内“恒定”的直流完全免疫；阶跃/斜坡/纹波会留下残差。
static uint16_t demod18k_channel(uint8_t use_adc2, int16_t dc_offset)
{
    int16_t s[DEMOD_ADAPT_SAMPLES];   /* 最大窗口 */
    const uint16_t *pat;
    const int16_t *pc, *ps;
    uint16_t ns;
    int32_t sum = 0;

    if (g_laser_ready) {
        pat = demod_at_rt;
        pc  = ref_cos_rt;
        ps  = ref_sin_rt;
        ns  = DEMOD_ADAPT_SAMPLES;
    } else
    {
        pat = demod_at;
        pc  = ref_cos;
        ps  = ref_sin;
        ns  = DEMOD_SAMPLES;
    }

    __HAL_TIM_SET_COUNTER(&htim1, 0);
    for (uint16_t n = 0; n < ns; n++) {
        uint32_t target = pat[n];
        uint32_t guard = 1000000u;
        while ((uint32_t)__HAL_TIM_GET_COUNTER(&htim1) < target && guard--) { }
        if (guard == 0u) return 0u;
        int16_t v = (int16_t)(use_adc2 ? bsp_adc2_read() : bsp_adc1_read()) - dc_offset;
        s[n] = v;
        sum += v;
    }

    int32_t mean = sum / ns;
    int64_t I = 0, Q = 0;
#if DEMOD_SPECTRAL_GUARD_ENABLE
    int64_t Ig = 0, Qg = 0;
#endif
    for (uint16_t n = 0; n < ns; n++) {
        int32_t v = s[n] - mean;
        I += (int64_t)v * pc[n];
        Q += (int64_t)v * ps[n];
#if DEMOD_SPECTRAL_GUARD_ENABLE
        if (g_laser_ready) {
            Ig += (int64_t)v * ref_cos_guard_rt[n];
            Qg += (int64_t)v * ref_sin_guard_rt[n];
        }
#endif
    }
    I >>= 15;
    Q >>= 15;
#if DEMOD_SPECTRAL_GUARD_ENABLE
    Ig >>= 15;
    Qg >>= 15;
#endif
    /* 真幅值 sqrt(I²+Q²), 与相位无关:
       旧 |I|+|Q| 会随"光与参考的相位差"在 1.0~1.414 间波动(均值1.273, std≈0.125);
       光实测 17962.9Hz vs 参考 18000Hz -> 37Hz 拍频, 每帧相位随机 -> 通道读数 ±10% 乘法噪声,
       且扫描期间相位漂移会让光斑剖面倾斜 -> 质心抖动(实测 sd=0.34点/相对噪声12.8%, std∝mean corr=0.94)。
       ×4/π 保持与旧读数的平均尺度一致 */
    float mag = sqrtf((float)I * (float)I + (float)Q * (float)Q) * 1.27324f;
#if DEMOD_SPECTRAL_GUARD_ENABLE
    if (g_laser_ready) {
        float guard = sqrtf((float)Ig * (float)Ig + (float)Qg * (float)Qg) * 1.27324f;
        mag -= DEMOD_SPECTRAL_GUARD_RATIO * guard;
        if (mag < 0.0f) mag = 0.0f;
    }
#endif
    if (ns < DEMOD_SAMPLES) {
        mag *= (float)DEMOD_SAMPLES / (float)ns;
    }
    uint32_t amp = (uint32_t)(mag + 0.5f);
    return (uint16_t)(amp > 65535u ? 65535u : amp);
}

#if MCU_IS_MASTER
/* 3.2µs 窄脉冲：在强通道上扫描采样相位 0~55µs，选解调幅值最大的网格 */
void bsp_sensor_align_pulse(uint16_t idx, uint32_t f_hz)
{
    if (idx >= MY_SENSOR_COUNT) return;

    /* 主控自己那段的峰值是镜像后的逻辑下标；换回物理 mux 下标 */
#if MY_MIRROR
    idx = (uint16_t)(MY_SENSOR_COUNT - 1 - idx);
#endif

    uint8_t upper = (uint8_t)(idx / MUX_LOWER_MAX);
    uint8_t lower = (uint8_t)(idx % MUX_LOWER_MAX);

    __disable_irq();
    mux_select(upper, lower);
    bsp_delay_us(MUX_BANK_EXTRA_US);
    bsp_delay_us(MUX_STABLE_CUR);
    for (int i = 0; i < ADC_DISCARD_COUNT; i++) bsp_adc2_read();

    uint16_t best_mag = 0, best_phase = 0;
    for (uint16_t ph = 0; ph < 56; ph++) {
        g_demod_phase_us = (uint8_t)ph;
        build_adapt_tables(f_hz);
        uint16_t mag = demod18k_channel(1, bg[1][idx]);
        if (mag > best_mag) {
            best_mag = mag;
            best_phase = ph;
        }
    }
    g_demod_phase_us = (uint8_t)best_phase;
    build_adapt_tables(f_hz);
    g_laser_peak_idx = idx;
    __enable_irq();
}
#endif
#endif

#if (SENSOR_PATH == SENSOR_PATH_TIA) && !DEMOD_18K_ENABLE
static uint16_t bg_sub(uint16_t raw_val, int16_t b)
{
    int32_t v = (int32_t)raw_val - b;
    return (uint16_t)((v > 0) ? v : 0);
}
#endif

#if DYN_BG_ENABLE
// 动态背景：每帧用最小的 1/8 通道估底（光斑只占少数通道，不会污染估计），
// 单帧最多减峰值的 30%，防止估计被污染时削掉真实信号
static void dyn_bg_sub(uint16_t *buf, uint16_t n)
{
    uint16_t tmp[SEG_SENSOR_COUNT];
    if (n > SEG_SENSOR_COUNT) n = SEG_SENSOR_COUNT;
    memcpy(tmp, buf, n * sizeof(uint16_t));

    for (uint16_t i = 1; i < n; i++) {
        uint16_t v = tmp[i];
        int16_t j = (int16_t)i - 1;
        while (j >= 0 && tmp[j] > v) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = v;
    }

    uint16_t k = n / 8;
    if (k < 4) k = 4;
    uint32_t sum = 0;
    for (uint16_t i = 0; i < k; i++) sum += tmp[i];
    uint16_t floor = (uint16_t)(sum / k);

    uint16_t peak = 0;
    for (uint16_t i = 0; i < n; i++)
        if (buf[i] > peak) peak = buf[i];

    uint16_t max_drop = (uint16_t)((uint32_t)peak * 3 / 10);
    if (floor > max_drop) floor = max_drop;

    if (floor)
        for (uint16_t i = 0; i < n; i++)
            buf[i] = (buf[i] > floor) ? (uint16_t)(buf[i] - floor) : 0;
}
#endif

#if NOISE_GATE_K > 0
// 自适应噪声门: 中位数估暗底 -> 门=K×中位 -> 低于门归零
// 暗态: 只剩零星尾部也会被门压平; 光斑: 幅值远大于门, 只损失翼部极低样本
static void noise_gate_sub(uint16_t *buf, uint16_t n)
{
    uint16_t tmp[SEG_SENSOR_COUNT];
    if (n > SEG_SENSOR_COUNT) n = SEG_SENSOR_COUNT;
    memcpy(tmp, buf, n * sizeof(uint16_t));

    for (uint16_t i = 1; i < n; i++) {
        uint16_t v = tmp[i];
        int16_t j = (int16_t)i - 1;
        while (j >= 0 && tmp[j] > v) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = v;
    }

    uint16_t med = tmp[n / 2];
    if (med > NOISE_GATE_MAX_MEDIAN) return;   /* 多数通道有光(泛光): 不设门, 防削信号 */

    uint32_t gate = (uint32_t)med * NOISE_GATE_K;
    if (gate == 0) return;

    for (uint16_t i = 0; i < n; i++)
        if (buf[i] < gate) buf[i] = 0;
}
#endif

#if ACQ_SMOOTH_ENABLE
// 5 抽头空间平滑 [1,2,3,2,1]/9 (与 v2 质心管线同款抽头) + 减去本帧基线
// 原理:
//   锁相输出 |I|+|Q| 取模 -> 对噪声有正偏置(暗态均值~76, 尾巴~250), 看起来"噪声很强";
//   噪声逐通道独立、光斑翼部空间连片 -> 平滑压噪声 ~50%, 翼部基本无损;
//   再减本帧中位数(光斑只占少数通道, 中位数=暗底) -> 暗态归 0, 弱翼部完整保留
static void smooth_baseline_sub(uint16_t *buf, uint16_t n)
{
    uint16_t sm[SEG_SENSOR_COUNT];
    uint16_t tmp[SEG_SENSOR_COUNT];
    if (n > SEG_SENSOR_COUNT) n = SEG_SENSOR_COUNT;

    for (uint16_t i = 0; i < n; i++) {
        uint32_t acc = 3u * buf[i];
        if (i > 0)     acc += 2u * buf[i - 1];
        if (i > 1)     acc +=      buf[i - 2];
        if (i + 1 < n) acc += 2u * buf[i + 1];
        if (i + 2 < n) acc +=      buf[i + 2];
        sm[i] = (uint16_t)(acc / 9u);
    }

    memcpy(tmp, sm, n * sizeof(uint16_t));
    for (uint16_t i = 1; i < n; i++) {
        uint16_t v = tmp[i];
        int16_t j = (int16_t)i - 1;
        while (j >= 0 && tmp[j] > v) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = v;
    }
    uint16_t base = tmp[n / 2];

    for (uint16_t i = 0; i < n; i++) {
        uint16_t v = (sm[i] > base) ? (uint16_t)(sm[i] - base) : 0;
#if NOISE_GATE_ABS > 0
        if (v < NOISE_GATE_ABS) v = 0;
#endif
        buf[i] = v;
    }
}
#endif

void bsp_sensor_acquire_pair(uint16_t *buf, uint16_t *before,
                             uint16_t count, uint8_t mode)
{
    (void)mode;
    uint8_t last_upper = 0xFF;   /* 用于检测换 bank */

    for (uint8_t upper = 0; upper < MUX_UPPER_MAX; upper++) {
        for (uint8_t lower = 0; lower < MUX_LOWER_MAX; lower++) {
            // 最后一个 bank 只接了 SEG_TAIL_COUNT 颗
            if (upper == MUX_UPPER_MAX - 1 && lower >= SEG_TAIL_COUNT) continue;

            mux_select(upper, lower);

            /* 换 bank：上层开关切换台阶更大（电荷注入更多），额外多等一段 */
            if (upper != last_upper) {
                bsp_delay_us(MUX_BANK_EXTRA_US);
                last_upper = upper;
            }
            bsp_delay_us(MUX_STABLE_CUR);   /* 通道建立时间（config.h 按路径定义） */

            uint16_t idx = (uint16_t)upper * MUX_LOWER_MAX + lower;

#if SENSOR_PATH == SENSOR_PATH_TIA
            /* ============ 路径A: TIA 裸信号 (ADC2/PB2) ============
               优点: 没有带通 → 切换尖峰 1.4µs 就衰减完, 不会被 330µs 振铃污染;
                     18kHz 选择性由固件锁相提供(数字滤波), 同样抗环境光 */
            for (int i = 0; i < ADC_DISCARD_COUNT; i++) {
                bsp_adc2_read();
            }
  #if DEMOD_18K_ENABLE
            buf[idx] = demod18k_channel(1, bg[1][idx]);   /* 锁相解调 */
  #else
            {
                uint32_t s = 0;
                uint16_t mn = 0xFFFF, mx = 0;
                for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
                    uint16_t v = bsp_adc2_read();
                    s += v;
                    if (v > mx) mx = v;
                    if (v < mn) mn = v;
                }
                buf[idx] = bg_sub((s - mn - mx) / (ADC_SAMPLE_COUNT - 2), bg[1][idx]);
            }
  #endif
#else
            /* ============ 路径B: 带通+包络 (ADC1/PA0) ============
               ⚠ 切换尖峰会激励带通(17.5~22kHz), 振铃约 330µs,
                 建立时间(MUX_STABLE_US)必须 ≥400us 才够, 否则读到上一通道振铃
               原理图实测: 包络检波 C30=1nF × R17=1K → 放电 τ=1µs << 周期 55µs,
                 输出是窄锯齿, 普通平均只剩峰值 ~1.8%(灵敏度损失~50倍)。
               优化: 建立后以 ADC 最快速度连采 ENV_BURST_SAMPLES 点
                 取 max-min = 调制幅值。自动去直流/底光, 无光读数≈噪声 */
            for (int i = 0; i < ADC_DISCARD_COUNT; i++) {
                bsp_adc1_read();
            }
            {
                uint16_t mn = 0xFFFF, mx = 0;
                for (int i = 0; i < ENV_BURST_SAMPLES; i++) {
                    uint16_t v = bsp_adc1_read();
                    if (v > mx) mx = v;
                    if (v < mn) mn = v;
                }
                buf[idx] = (uint16_t)(mx - mn);
            }
#endif
        }
    }

    /* Preserve the non-spatially-processed layer for the diagnostic UI.
       This is the demodulated/amplitude layer, not the ADC time series. */
    if (before != NULL) {
        uint16_t n = count;
        if (n > SEG_SENSOR_COUNT) n = SEG_SENSOR_COUNT;
        memcpy(before, buf, n * sizeof(uint16_t));
    }

#if DYN_BG_ENABLE
    dyn_bg_sub(buf, SEG_SENSOR_COUNT);
#endif
#if NOISE_GATE_K > 0
    noise_gate_sub(buf, SEG_SENSOR_COUNT);
#endif
#if ACQ_SMOOTH_ENABLE
    smooth_baseline_sub(buf, SEG_SENSOR_COUNT);
#endif
}

void bsp_sensor_acquire(uint16_t *buf, uint16_t count, uint8_t mode)
{
    bsp_sensor_acquire_pair(buf, NULL, count, mode);
}
