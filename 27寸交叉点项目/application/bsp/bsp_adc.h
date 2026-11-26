// bsp_adc.h
#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

void bsp_adc_init(void);
uint16_t bsp_adc1_read(void);
uint16_t bsp_adc2_read(void);

#endif