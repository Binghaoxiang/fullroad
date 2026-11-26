// bsp_adc.c
#include "bsp_adc.h"
#include "adc.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

static uint8_t adc_fast_ok = 0;
volatile uint32_t bsp_adc_timeout_count = 0;

static uint8_t adc_enable(ADC_TypeDef *adc)
{
    if ((adc->CR & ADC_CR_ADEN) != 0U) {
        return 1;
    }
    adc->ISR |= ADC_ISR_ADRDY;      /* 清 ADRDY */
    adc->CR |= ADC_CR_ADEN;
    uint32_t guard = 100000U;
    while (((adc->ISR & ADC_ISR_ADRDY) == 0U) && guard--) {
    }
    return (adc->ISR & ADC_ISR_ADRDY) != 0U;
}

void bsp_adc_init(void)
{
    ADC_ChannelConfTypeDef ch = {0};

    /* 采样时间分路设置:
       ADC2(TIA/PB2): 保持 24.5 周期(≈0.58µs)长孔径 ——
         采样电容充分建立(灵敏度) + 电荷回踢有时间恢复(消除切换噪声);
       ADC1(包络/PA0): 源是 LMV321 运放+1nF 保持电容(低阻), 与 TIA 路不同 ——
         包络锯齿尖峰只有 ~1µs 宽, 长孔径会把尖峰"平均"掉(丢灵敏度),
         改 6.5 周期(0.15µs)短孔径尖峰采得准 */
    ch.Rank = ADC_REGULAR_RANK_1;
    ch.SingleDiff = ADC_SINGLE_ENDED;
    ch.OffsetNumber = ADC_OFFSET_NONE;
    ch.Offset = 0;

    ch.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    ch.Channel = ADC_CHANNEL_1;      /* ADC1: PA0 / ENV */
    HAL_ADC_ConfigChannel(&hadc1, &ch);

    ch.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    ch.Channel = ADC_CHANNEL_12;     /* ADC2: PB2 / TIA */
    HAL_ADC_ConfigChannel(&hadc2, &ch);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    /* 使能 ADC：后面的寄存器直读依赖 ADEN 已置位 */
    if (adc_enable(ADC1) && adc_enable(ADC2)) {
        adc_fast_ok = 1;
    }
}

/* 直接寄存器读取：单次转换约 0.4µs + 几条指令，
   比 HAL_ADC_Start/PollForConversion/GetValue 快约 5 倍，
   把整段(50/58 通道)的扫描开销压到最小 */
static inline uint16_t adc_read_fast(ADC_TypeDef *adc)
{
    uint32_t guard = 100000U;
    adc->CR |= ADC_CR_ADSTART;
    while ((adc->ISR & ADC_ISR_EOC) == 0U && guard--) {
    }
    if ((adc->ISR & ADC_ISR_EOC) == 0U) {
        /* A peripheral fault must not freeze the whole product/UART forever.
           Stop the stuck conversion and return a zero sample; subsequent
           frames can recover normally or expose the bad channel as invalid. */
        bsp_adc_timeout_count++;
        if (adc->CR & ADC_CR_ADSTART) {
            adc->CR |= ADC_CR_ADSTP;
            guard = 100000U;
            while ((adc->CR & ADC_CR_ADSTP) != 0U && guard--) { }
        }
        adc->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
        return 0u;
    }
    return (uint16_t)adc->DR;       /* 读 DR 自动清 EOC */
}

uint16_t bsp_adc1_read(void)
{
    if (adc_fast_ok) {
        return adc_read_fast(ADC1);
    }
    /* 兜底：直读不可用时退回 HAL（保证功能不受影响） */
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    return HAL_ADC_GetValue(&hadc1);
}

uint16_t bsp_adc2_read(void)
{
    if (adc_fast_ok) {
        return adc_read_fast(ADC2);
    }
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 1);
    return HAL_ADC_GetValue(&hadc2);
}
