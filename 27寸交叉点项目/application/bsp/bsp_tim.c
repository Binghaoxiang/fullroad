// bsp_tim.c
#include "bsp_tim.h"
#include "tim.h"

extern TIM_HandleTypeDef htim1;

void bsp_tim_init(void)
{
    HAL_TIM_Base_Start(&htim1);
}

void bsp_delay_us(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}