/**
  ******************************************************************************
  * @file    servo.c
  * @brief   舵机控制层实现
  *
  *  ┌──────────── 关键设计 ────────────┐
  *  1) 角度用 int32 毫度表示，映射用 Q24 定点增益 + 64bit 乘法，
  *     1kHz 中断里只有一次 64 位乘法和移位，没有除法；
  *  2) 斜率限制在 1kHz 时基上做，累加器保证低速时也不丢步；
  *  3) 所有"改参数"的动作都在临界区里完成，中断里读到的永远是完整状态；
  *  4) 输出只在数值真的变化时才写 CCR，避免无谓的寄存器写入。
  ******************************************************************************
  */
#include "servo.h"
#include "board.h"

/*----------------------------------------------------------------------------
 * 内部状态
 *--------------------------------------------------------------------------*/
static servo_cfg_t s_cfg;

/* 每 1 毫度对应的脉宽(us)，Q24 定点：gain_q24 = (us 跨度 << 24) / 角度跨度 */
static int32_t s_gain_q24;

static volatile int32_t  s_pos_mdeg;     /* 当前角度 */
static volatile int32_t  s_tgt_mdeg;     /* 目标角度 */
static volatile uint16_t s_pulse_us;     /* 当前脉宽 */
static volatile uint8_t  s_enabled;      /* 是否输出脉冲 */
static volatile uint8_t  s_moving;       /* 是否在运动 */
static uint32_t          s_slew_acc;     /* 斜率累加器（避免低速时取整丢步） */

/* 分档位点动的步长表与当前档位（档位 1 起） */
static const int32_t k_jog_step[SERVO_JOG_LEVELS] = SERVO_JOG_STEP_TABLE;
static uint8_t       s_jog_level = SERVO_JOG_LEVEL_DEF;

#define SPEED_MAX_MDEG_S     3600000UL   /* 3600°/s 上限 */
#define ANGLE_LIMIT_MDEG     360000L     /* 角度参数允许 ±360° */
#define TRIM_LIMIT_MDEG      180000L     /* 零点修正允许 ±180° */
#define ANGLE_MIN_SPAN_MDEG  1000L       /* 角度跨度至少 1° */
#define PULSE_MIN_SPAN_US    100U        /* 脉宽跨度至少 100us */

/*----------------------------------------------------------------------------
 * 临界区：保存/恢复 PRIMASK，可嵌套
 *--------------------------------------------------------------------------*/
static uint32_t crit_enter(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void crit_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/*----------------------------------------------------------------------------
 * 映射
 *--------------------------------------------------------------------------*/
/* 重新计算定点增益（必须在临界区内或初始化阶段调用） */
static void update_gain(void)
{
    int32_t span_us = (int32_t)s_cfg.pulse_max_us - (int32_t)s_cfg.pulse_min_us;
    int32_t span_md = s_cfg.angle_max_mdeg - s_cfg.angle_min_mdeg;

    if (span_md <= 0) {
        s_gain_q24 = 0;
        return;
    }
    s_gain_q24 = (int32_t)(((int64_t)span_us << 24) / (int64_t)span_md);
}

/* 角度（已含 trim、已限幅）-> 脉宽，Q24 四舍五入 */
static uint16_t map_angle_to_pulse(int32_t mdeg)
{
    int64_t delta = (int64_t)mdeg - (int64_t)s_cfg.angle_min_mdeg;
    int32_t us;

    if (delta < 0) {
        delta = 0;
    }
    us = (int32_t)s_cfg.pulse_min_us
       + (int32_t)((delta * (int64_t)s_gain_q24 + (int64_t)(1UL << 23)) >> 24);

    if (us > (int32_t)s_cfg.pulse_max_us) {
        us = (int32_t)s_cfg.pulse_max_us;
    }
    if (us < (int32_t)s_cfg.pulse_min_us) {
        us = (int32_t)s_cfg.pulse_min_us;
    }
    return (uint16_t)us;
}

uint16_t servo_angle_to_pulse(int32_t cmd_mdeg)
{
    int32_t eff = cmd_mdeg + s_cfg.trim_mdeg;

    if (eff < s_cfg.angle_min_mdeg) {
        eff = s_cfg.angle_min_mdeg;
    }
    if (eff > s_cfg.angle_max_mdeg) {
        eff = s_cfg.angle_max_mdeg;
    }
    return map_angle_to_pulse(eff);
}

int32_t servo_pulse_to_angle(uint16_t us)
{
    int32_t span_us = (int32_t)s_cfg.pulse_max_us - (int32_t)s_cfg.pulse_min_us;
    int32_t span_md = s_cfg.angle_max_mdeg - s_cfg.angle_min_mdeg;
    int32_t d;

    if ((span_us <= 0) || (span_md <= 0)) {
        return s_cfg.angle_min_mdeg;
    }
    d = (int32_t)us - (int32_t)s_cfg.pulse_min_us;
    return s_cfg.angle_min_mdeg + (int32_t)(((int64_t)d * (int64_t)span_md) / (int64_t)span_us);
}

/* 把当前角度重新算成脉宽并送进 CCR（内部用，调用者已进临界区） */
static void apply_locked(void)
{
    s_pulse_us = servo_angle_to_pulse(s_pos_mdeg);
    if (s_enabled != 0U) {
        board_servo_pwm_set_pulse(s_pulse_us);
    }
}

void servo_apply_now(void)
{
    uint32_t pm = crit_enter();
    apply_locked();
    crit_exit(pm);
}

/*----------------------------------------------------------------------------
 * 初始化与默认参数
 *--------------------------------------------------------------------------*/
void servo_defaults(void)
{
    uint32_t pm = crit_enter();

    s_cfg.pulse_min_us   = SERVO_PULSE_MIN_US_DEF;
    s_cfg.pulse_max_us   = SERVO_PULSE_MAX_US_DEF;
    s_cfg.angle_min_mdeg = SERVO_ANGLE_MIN_MDEG_DEF;
    s_cfg.angle_max_mdeg = SERVO_ANGLE_MAX_MDEG_DEF;
    s_cfg.speed_mdeg_s   = SERVO_SPEED_MDEG_S_DEF;
    s_cfg.trim_mdeg      = SERVO_TRIM_MDEG_DEF;
    update_gain();

    /* 范围可能变小，把位置夹回范围内 */
    if (s_pos_mdeg < s_cfg.angle_min_mdeg) {
        s_pos_mdeg = s_cfg.angle_min_mdeg;
    }
    if (s_pos_mdeg > s_cfg.angle_max_mdeg) {
        s_pos_mdeg = s_cfg.angle_max_mdeg;
    }
    if (s_tgt_mdeg < s_cfg.angle_min_mdeg) {
        s_tgt_mdeg = s_cfg.angle_min_mdeg;
    }
    if (s_tgt_mdeg > s_cfg.angle_max_mdeg) {
        s_tgt_mdeg = s_cfg.angle_max_mdeg;
    }
    s_slew_acc = 0U;
    apply_locked();

    crit_exit(pm);
}

void servo_init(void)
{
    uint32_t pm;

    s_pos_mdeg = 0;
    s_tgt_mdeg = 0;
    s_pulse_us = 0;
    s_enabled  = 0U;
    s_moving   = 0U;
    s_slew_acc = 0U;

    servo_defaults();               /* 内部已经 apply_locked 过一次 */

    pm = crit_enter();
    s_pos_mdeg = SERVO_START_ANGLE_MDEG;
    if (s_pos_mdeg < s_cfg.angle_min_mdeg) {
        s_pos_mdeg = s_cfg.angle_min_mdeg;
    }
    if (s_pos_mdeg > s_cfg.angle_max_mdeg) {
        s_pos_mdeg = s_cfg.angle_max_mdeg;
    }
    s_tgt_mdeg = s_pos_mdeg;
    s_pulse_us = servo_angle_to_pulse(s_pos_mdeg);
    board_servo_pwm_set_pulse(s_pulse_us);   /* 先把脉宽装好，再开 CC1E，避免上电毛刺 */
    crit_exit(pm);
}

/*----------------------------------------------------------------------------
 * 1ms 时基：速度规划 + 输出
 *--------------------------------------------------------------------------*/
/* ---- 360 度连续旋转（无位置反馈舵机）：PWM 控制转速，不控制角度 ---- */
#ifndef SERVO_CR_NEUTRAL_US
#define SERVO_CR_NEUTRAL_US     1500U
#endif
#ifndef SERVO_CR_HALF_RANGE_US
#define SERVO_CR_HALF_RANGE_US  500U
#endif

static uint8_t  s_cr_mode;
static int32_t  s_cr_permille;
static uint32_t s_cr_ms;
static uint16_t s_cr_neutral = SERVO_CR_NEUTRAL_US;

/* permille: -1000..+1000 转速；ms: 0=一直转，>0=转 ms 后自动停 */
void servo_spin(int32_t permille, uint32_t ms)
{
    int32_t delta;

    if (permille > 1000)  { permille = 1000;  }
    if (permille < -1000) { permille = -1000; }

    s_cr_mode     = 1U;
    s_cr_permille = permille;
    s_cr_ms       = ms;

    delta = (permille * (int32_t)SERVO_CR_HALF_RANGE_US) / 1000;
    board_servo_pwm_set_pulse((uint16_t)((int32_t)s_cr_neutral + delta));
    board_servo_pwm_enable(1U);
}

void servo_spin_neutral(uint16_t us)
{
    s_cr_neutral = us;
    if ((s_cr_mode != 0U) && (s_cr_permille == 0)) {
        board_servo_pwm_set_pulse(us);
    }
}

uint16_t servo_spin_neutral_get(void) { return s_cr_neutral; }
int32_t  servo_spin_permille(void)   { return s_cr_permille; }
uint8_t  servo_is_spin_mode(void)    { return s_cr_mode; }

void servo_tick_1ms(void)
{
    int32_t  pos, tgt;
    uint32_t speed, step;
    uint16_t pulse;

    /* 360 模式：只做倒计时，不参与位置闭环 */
    if (s_cr_mode != 0U) {
        if (s_cr_ms != 0U) {
            s_cr_ms--;
            if (s_cr_ms == 0U) {
                s_cr_permille = 0;
                board_servo_pwm_set_pulse(s_cr_neutral);
            }
        }
        return;
    }

    if (s_enabled == 0U) {
        return;                     /* 未使能时保持状态，使能后从原位置继续 */
    }

    pos = s_pos_mdeg;
    tgt = s_tgt_mdeg;

    if (pos == tgt) {
        if (s_moving != 0U) {
            s_moving = 0U;
        }
        return;
    }

    speed = s_cfg.speed_mdeg_s;
    if (speed == 0U) {
        pos = tgt;                                      /* 不限速：一步到位 */
    } else {
        s_slew_acc += speed;                            /* 单位：毫度/秒 -> 每秒累加 */
        step = s_slew_acc / 1000U;                      /* 每毫秒应该走的毫度数 */
        s_slew_acc -= step * 1000U;                     /* 余数留到下一毫秒，低速也不丢步 */
        if (step == 0U) {
            return;                                     /* 这一毫秒还不够走 1 毫度 */
        }
        if (pos < tgt) {
            uint32_t remain = (uint32_t)(tgt - pos);
            pos = (step >= remain) ? tgt : (pos + (int32_t)step);
        } else {
            uint32_t remain = (uint32_t)(pos - tgt);
            pos = (step >= remain) ? tgt : (pos - (int32_t)step);
        }
    }

    s_pos_mdeg = pos;

    pulse = servo_angle_to_pulse(pos);
    if (pulse != s_pulse_us) {
        s_pulse_us = pulse;
        board_servo_pwm_set_pulse(pulse);
    }
}

/*----------------------------------------------------------------------------
 * 控制接口
 *--------------------------------------------------------------------------*/
void servo_set_angle(int32_t mdeg)
{
    uint32_t pm = crit_enter();

    if (mdeg < s_cfg.angle_min_mdeg) {
        mdeg = s_cfg.angle_min_mdeg;
    }
    if (mdeg > s_cfg.angle_max_mdeg) {
        mdeg = s_cfg.angle_max_mdeg;
    }
    s_tgt_mdeg = mdeg;
    if (s_tgt_mdeg != s_pos_mdeg) {
        s_moving = 1U;
    } else {
        s_moving = 0U;
    }
    crit_exit(pm);
}

void servo_set_pulse(uint16_t us)
{
    uint32_t pm = crit_enter();

    /* 直接脉宽模式是给标定/示波器验证用的，可以越过软件行程，
     * 但绝不越过绝对安全边界（200~3000us）。 */
    if (us < SERVO_PULSE_ABS_MIN_US) {
        us = SERVO_PULSE_ABS_MIN_US;
    }
    if (us > SERVO_PULSE_ABS_MAX_US) {
        us = SERVO_PULSE_ABS_MAX_US;
    }
    s_pulse_us = us;
    if (s_enabled != 0U) {
        board_servo_pwm_set_pulse(us);
    }
    /* 位置跟着脉宽走，之后切回角度模式不会有突跳 */
    s_pos_mdeg = servo_pulse_to_angle(us);
    s_tgt_mdeg = s_pos_mdeg;
    s_slew_acc = 0U;
    s_moving   = 0U;
    crit_exit(pm);
}

void servo_set_enabled(uint8_t enable)
{
    uint32_t pm = crit_enter();

    if (enable != 0U) {
        board_servo_pwm_set_pulse(s_pulse_us);   /* 先装脉宽，再开输出 */
        board_servo_pwm_enable(1U);
        s_enabled = 1U;
    } else {
        board_servo_pwm_enable(0U);
        s_enabled = 0U;
        s_tgt_mdeg = s_pos_mdeg;                 /* 停住，重新使能时不跳 */
        s_moving = 0U;
    }
    crit_exit(pm);
}

/*----------------------------------------------------------------------------
 * 标定
 *--------------------------------------------------------------------------*/
uint8_t servo_set_pulse_range(uint16_t min_us, uint16_t max_us)
{
    uint32_t pm;

    if (min_us < SERVO_PULSE_ABS_MIN_US) {
        return 0U;
    }
    if (max_us > SERVO_PULSE_ABS_MAX_US) {
        return 0U;
    }
    if ((uint16_t)(max_us - min_us) < PULSE_MIN_SPAN_US) {
        return 0U;
    }

    pm = crit_enter();
    s_cfg.pulse_min_us = min_us;
    s_cfg.pulse_max_us = max_us;
    update_gain();
    /* 标定时保持当前脉宽不动，把角度反算回来，舵机不会因为改参数而乱走 */
    s_pos_mdeg = servo_pulse_to_angle(s_pulse_us);
    if (s_pos_mdeg < s_cfg.angle_min_mdeg) {
        s_pos_mdeg = s_cfg.angle_min_mdeg;
    }
    if (s_pos_mdeg > s_cfg.angle_max_mdeg) {
        s_pos_mdeg = s_cfg.angle_max_mdeg;
    }
    s_tgt_mdeg = s_pos_mdeg;
    s_slew_acc = 0U;
    s_moving   = 0U;
    apply_locked();
    crit_exit(pm);
    return 1U;
}

uint8_t servo_set_angle_range(int32_t min_mdeg, int32_t max_mdeg)
{
    uint32_t pm;

    if (min_mdeg >= max_mdeg) {
        return 0U;
    }
    if ((max_mdeg - min_mdeg) < ANGLE_MIN_SPAN_MDEG) {
        return 0U;
    }
    if ((min_mdeg < -ANGLE_LIMIT_MDEG) || (max_mdeg > ANGLE_LIMIT_MDEG)) {
        return 0U;
    }

    pm = crit_enter();
    s_cfg.angle_min_mdeg = min_mdeg;
    s_cfg.angle_max_mdeg = max_mdeg;
    update_gain();

    if (s_pos_mdeg < min_mdeg) {
        s_pos_mdeg = min_mdeg;
    }
    if (s_pos_mdeg > max_mdeg) {
        s_pos_mdeg = max_mdeg;
    }
    if (s_tgt_mdeg < min_mdeg) {
        s_tgt_mdeg = min_mdeg;
    }
    if (s_tgt_mdeg > max_mdeg) {
        s_tgt_mdeg = max_mdeg;
    }
    s_slew_acc = 0U;
    apply_locked();
    crit_exit(pm);
    return 1U;
}

void servo_set_speed(uint32_t mdeg_s)
{
    uint32_t pm;

    if (mdeg_s > SPEED_MAX_MDEG_S) {
        mdeg_s = SPEED_MAX_MDEG_S;
    }
    pm = crit_enter();
    s_cfg.speed_mdeg_s = mdeg_s;
    s_slew_acc = 0U;
    crit_exit(pm);
}

void servo_set_trim(int32_t mdeg)
{
    uint32_t pm;

    if (mdeg < -TRIM_LIMIT_MDEG) {
        mdeg = -TRIM_LIMIT_MDEG;
    }
    if (mdeg > TRIM_LIMIT_MDEG) {
        mdeg = TRIM_LIMIT_MDEG;
    }
    pm = crit_enter();
    s_cfg.trim_mdeg = mdeg;
    apply_locked();                 /* 立刻按新零点输出，方便边调边看 */
    crit_exit(pm);
}

/*----------------------------------------------------------------------------
 * 分档位点动（多级指令转动）
 *   相对当前"目标"角度累加，而不是相对实际位置，
 *   这样连续快速点动不会因为舵机还没走到位而丢步。
 *--------------------------------------------------------------------------*/
uint8_t servo_get_jog_level(void)
{
    return s_jog_level;
}

int32_t servo_get_jog_step(void)
{
    return k_jog_step[s_jog_level - 1U];
}

uint8_t servo_set_jog_level(uint8_t level)
{
    if ((level == 0U) || (level > SERVO_JOG_LEVELS)) {
        return 0U;
    }
    s_jog_level = level;
    return 1U;
}

int32_t servo_jog(int32_t steps)
{
    uint32_t pm;
    int64_t  delta;
    int32_t  tgt;

    pm    = crit_enter();
    delta = (int64_t)steps * (int64_t)k_jog_step[s_jog_level - 1U];
    tgt   = s_tgt_mdeg + (int32_t)delta;

    if (tgt < s_cfg.angle_min_mdeg) {
        tgt = s_cfg.angle_min_mdeg;
    }
    if (tgt > s_cfg.angle_max_mdeg) {
        tgt = s_cfg.angle_max_mdeg;
    }
    s_tgt_mdeg = tgt;
    s_moving   = (uint8_t)((s_tgt_mdeg != s_pos_mdeg) ? 1U : 0U);
    crit_exit(pm);
    return tgt;
}

/*----------------------------------------------------------------------------
 * 查询
 *--------------------------------------------------------------------------*/
servo_state_t servo_get_state(void)
{
    servo_state_t st;
    uint32_t pm = crit_enter();

    st.pos_mdeg    = s_pos_mdeg;
    st.target_mdeg = s_tgt_mdeg;
    st.pulse_us    = s_pulse_us;
    st.enabled     = s_enabled;
    st.moving      = s_moving;

    crit_exit(pm);
    return st;
}

servo_cfg_t servo_get_cfg(void)
{
    servo_cfg_t cfg;
    uint32_t pm = crit_enter();

    cfg = s_cfg;

    crit_exit(pm);
    return cfg;
}

int32_t servo_angle_min(void)
{
    return s_cfg.angle_min_mdeg;
}

int32_t servo_angle_max(void)
{
    return s_cfg.angle_max_mdeg;
}

uint32_t servo_get_speed(void)
{
    return s_cfg.speed_mdeg_s;
}

int32_t servo_get_trim(void)
{
    return s_cfg.trim_mdeg;
}

void servo_wait_done(uint32_t timeout_ms)
{
    uint32_t start = board_millis();

    while (s_moving != 0U) {
        if ((uint32_t)(board_millis() - start) >= timeout_ms) {
            break;
        }
        board_delay_ms(1U);
    }
}
