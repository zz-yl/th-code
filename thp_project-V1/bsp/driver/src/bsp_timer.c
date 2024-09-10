/**@file   bsp_timer.c
* @brief   定时器\PWM驱动
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_timer.h"
#include "bsp_cfg.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/


/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

uint32_t SysTimer = 0;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/* TIM4初始化 */
void TIM4Init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    NVIC_InitTypeDef        NVIC_InitStruct = {0};

    TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  ///使能TIM4时钟

    TIM_TimeBaseStruct.TIM_Period        = arr;  //自动重装载值
    TIM_TimeBaseStruct.TIM_Prescaler     = psc;  //定时器分频
    TIM_TimeBaseStruct.TIM_CounterMode   = TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1; 

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);//初始化TIM5

    TIM_ITConfig(TIM4, TIM_IT_Update,ENABLE); //允许定时器更新中断
    TIM_Cmd(TIM4, ENABLE); //使能定时器

    NVIC_InitStruct.NVIC_IRQChannel                   = TIM4_IRQn; //定时器4中断
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01; //抢占优先级1
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x03; //子优先级3
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}
/* TIM4中断 */
void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) //溢出中断,1ms的中断
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //清除中断标志位
        
        SysTimer++;
    }
}

