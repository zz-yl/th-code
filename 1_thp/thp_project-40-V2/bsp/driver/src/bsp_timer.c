/**@file   bsp_timer.c
* @brief   ��ʱ��\PWM����
*          ������ʱ��:TIM6,TIM7
*          ͨ�ö�ʱ��:TIM2~TIM5,TIM12~TIM17
*          �߼���ʱ��:TIM1,TIM8
*          �͹��Ķ�ʱ��:LPTIM1~LPTIM5
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

uint32_t sys_timer = 0;
uint32_t FreeRTOSRunTimeTicks = 0;
/* �ṹ���ʼ�� */
TIM_HandleTypeDef tim5_handle = {0};
TIM_HandleTypeDef tim6_handle = {0};
TIM_HandleTypeDef tim7_handle = {0};

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  TIM5��ʼ��
* @attention 
*/
void tim5_init(uint16_t arr, uint16_t psc)
{
    /* ʹ��ʱ�� */
    __HAL_RCC_TIM5_CLK_ENABLE();
    /* ��ʱ������ */
    tim5_handle.Instance               = TIM5;               /* ��ʱ��x */
    tim5_handle.Init.Prescaler         = psc;                /* ��Ƶϵ�� */
    tim5_handle.Init.CounterMode       = TIM_COUNTERMODE_UP; /* ��������ģʽ */
    tim5_handle.Init.Period            = arr;                /* �Զ�װ��ֵ */
    tim5_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;  /* ʱ�ӷ�Ƶ���� */
    tim5_handle.Init.RepetitionCounter = 0;                       /* �ظ������� */
    tim5_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //�Զ�����Ԥװ��ʹ��
    HAL_TIM_Base_Init(&tim5_handle);
    /* ��ʱ���ж����� */
    HAL_NVIC_SetPriority(TIM5_IRQn, 2, 0);  /* ���ȼ� */
    HAL_NVIC_EnableIRQ(TIM5_IRQn);          /* ����ITMx�ж� */
    /* ���� */
    HAL_TIM_Base_Start_IT(&tim5_handle);  /* ʹ�ܶ�ʱ��x�Ͷ�ʱ�������ж� */
}
/**
* @brief  TIM5�ж�
* @attention 
*/
void TIM5_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&tim5_handle);  /* ��ʱ���ص����� */
}
/**
* @brief  TIM6��ʼ��
* @attention 
*/
void tim6_init(uint16_t arr, uint16_t psc)
{
    /* ʹ��ʱ�� */
    __HAL_RCC_TIM6_CLK_ENABLE();
    /* ��ʱ������ */
    tim6_handle.Instance               = TIM6;               /* ��ʱ��x */
    tim6_handle.Init.Prescaler         = psc;                /* ��Ƶϵ�� */
    tim6_handle.Init.CounterMode       = TIM_COUNTERMODE_UP; /* ��������ģʽ */
    tim6_handle.Init.Period            = arr;                /* �Զ�װ��ֵ */
    tim6_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;  /* ʱ�ӷ�Ƶ���� */
    tim6_handle.Init.RepetitionCounter = 0;                       /* �ظ������� */
    tim6_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //�Զ�����Ԥװ��ʹ��
    HAL_TIM_Base_Init(&tim6_handle);
    /* ��ʱ���ж����� */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);  /* ���ȼ� */
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);          /* ����ITMx�ж� */
    /* ���� */
    HAL_TIM_Base_Start_IT(&tim6_handle);  /* ʹ�ܶ�ʱ��x�Ͷ�ʱ�������ж� */
}
/**
* @brief  TIM6�ж�
* @attention 
*/
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&tim6_handle);  /* ��ʱ���ص����� */
}
/**
* @brief  TIM7��ʼ��
* @attention 
*/
void tim7_init(uint16_t arr, uint16_t psc)
{
    /* ʹ��ʱ�� */
    __HAL_RCC_TIM7_CLK_ENABLE();
    /* ��ʱ������ */
    tim7_handle.Instance               = TIM7;               /* ��ʱ��x */
    tim7_handle.Init.Prescaler         = psc;                /* ��Ƶϵ�� */
    tim7_handle.Init.CounterMode       = TIM_COUNTERMODE_UP; /* ��������ģʽ */
    tim7_handle.Init.Period            = arr;                /* �Զ�װ��ֵ */
    tim7_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;  /* ʱ�ӷ�Ƶ���� */
    tim7_handle.Init.RepetitionCounter = 0;                       /* �ظ������� */
    tim7_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //�Զ�����Ԥװ��ʹ��
    HAL_TIM_Base_Init(&tim7_handle);
    /* ��ʱ���ж����� */
    HAL_NVIC_SetPriority(TIM7_IRQn, 2, 0);  /* ���ȼ� */
    HAL_NVIC_EnableIRQ(TIM7_IRQn);          /* ����ITMx�ж� */
    /* ���� */
    HAL_TIM_Base_Start_IT(&tim7_handle);  /* ʹ�ܶ�ʱ��x�Ͷ�ʱ�������ж� */
}
/**
* @brief  TIM6�ж�
* @attention 
*/
void TIM7_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&tim7_handle);  /* ��ʱ���ص����� */
}
/**
 * @brief       �ص���������ʱ���жϷ���������
 * @param       htim  : ��ʱ�����
 * @retval      ��
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
//        encoder_call();
    }
    if (htim->Instance == TIM6)
    {
        sys_timer++;  /* ϵͳʱ��ms */
    }
    else if (htim->Instance == TIM7)
    {
        FreeRTOSRunTimeTicks++;  /* freertos����ͳ��cpuռ���ʼ�ʱ */
    }
}
/**
* @brief  freertos����ͳ��cpuռ���ʵĳ�ʼ������
* @attention 
*/
void ConfigureTimeForRunTimeStats(void)
{
    tim7_init(6-1, 4000-1);
}
