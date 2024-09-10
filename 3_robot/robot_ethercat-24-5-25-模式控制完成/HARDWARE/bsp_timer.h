#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H


#include "stm32f4xx.h"

void bsp_InitTimer(void);
uint32_t bsp_GetTickCount(void);
void TIM2_Int_Init(u32 arr,u16 psc);
void Delay_us(uint32_t time);
void Delay_ms(uint32_t time);
#endif


