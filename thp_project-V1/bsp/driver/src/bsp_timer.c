/**@file   bsp_timer.c
* @brief   ��ʱ��\PWM����
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

/* TIM4��ʼ�� */
void TIM4Init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    NVIC_InitTypeDef        NVIC_InitStruct = {0};

    TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  ///ʹ��TIM4ʱ��

    TIM_TimeBaseStruct.TIM_Period        = arr;  //�Զ���װ��ֵ
    TIM_TimeBaseStruct.TIM_Prescaler     = psc;  //��ʱ����Ƶ
    TIM_TimeBaseStruct.TIM_CounterMode   = TIM_CounterMode_Up; //���ϼ���ģʽ
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1; 

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);//��ʼ��TIM5

    TIM_ITConfig(TIM4, TIM_IT_Update,ENABLE); //����ʱ�������ж�
    TIM_Cmd(TIM4, ENABLE); //ʹ�ܶ�ʱ��

    NVIC_InitStruct.NVIC_IRQChannel                   = TIM4_IRQn; //��ʱ��4�ж�
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01; //��ռ���ȼ�1
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x03; //�����ȼ�3
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}
/* TIM4�ж� */
void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) //����ж�,1ms���ж�
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //����жϱ�־λ
        
        SysTimer++;
    }
}

