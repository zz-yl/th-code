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

uint32_t sys_timer = 0;

/* �ṹ���ʼ�� */
TIM_HandleTypeDef tim1_handle = {0};

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void bsp_valve_ctrl(uint16_t cmp)
{
    if(cmp > VALVE_MAX)
    {
        cmp = VALVE_MAX;
    }
    __HAL_TIM_SET_COMPARE(&tim1_handle, TIM_CHANNEL_1, cmp);
}

/**
* @brief  TIM1PWM��ʼ��
* @attention 
*/
void bsp_tim1_pwm_init(uint16_t arr, uint16_t psc)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init = {0};
    TIM_OC_InitTypeDef tim_oc_init = {0};
    HAL_TIM_Base_DeInit(&tim1_handle);
    /* ʹ��ʱ�� */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    /* IO���� */
    gpio_init.Pin       = GPIO_PIN_8;        //���ź�
    gpio_init.Mode      = GPIO_MODE_AF_PP;   //ģʽ
    gpio_init.Pull      = GPIO_PULLUP;       //����������
    gpio_init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;  //IO��������·��Ӧ�ٶ�
    gpio_init.Alternate = GPIO_AF1_TIM1;        //����
    HAL_GPIO_Init(GPIOA, &gpio_init);  //��ʼ��
    /* pwm tim���� */
    tim1_handle.Instance               = TIM1;               /* ��ʱ��x */
    tim1_handle.Init.Prescaler         = psc;                /* ��Ƶϵ�� */
    tim1_handle.Init.CounterMode       = TIM_COUNTERMODE_UP; /* ��������ģʽ */
    tim1_handle.Init.Period            = arr;                /* �Զ�װ��ֵ */
    tim1_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;  /* ʱ�ӷ�Ƶ���� */
    tim1_handle.Init.RepetitionCounter = 0;                       /* �ظ������� */
    tim1_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //�Զ�����Ԥװ��ʹ��
    HAL_TIM_PWM_Init(&tim1_handle);
    /* oc���� */
    tim_oc_init.OCMode       = TIM_OCMODE_PWM1;
    tim_oc_init.Pulse        = 0;
    tim_oc_init.OCPolarity   = TIM_OCPOLARITY_HIGH;
    tim_oc_init.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    tim_oc_init.OCFastMode   = TIM_OCFAST_DISABLE;
    tim_oc_init.OCIdleState  = TIM_OCIDLESTATE_RESET;
    tim_oc_init.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&tim1_handle, &tim_oc_init, TIM_CHANNEL_1);
    /* ���� */
    HAL_TIM_PWM_Start(&tim1_handle, TIM_CHANNEL_1);  /* ʹ�ܶ�ʱ��x�Ͷ�ʱ�������ж� */
}

