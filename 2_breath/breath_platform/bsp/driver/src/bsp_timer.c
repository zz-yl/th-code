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

uint32_t sys_timer = 0;

/* 结构体初始化 */
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
* @brief  TIM1PWM初始化
* @attention 
*/
void bsp_tim1_pwm_init(uint16_t arr, uint16_t psc)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init = {0};
    TIM_OC_InitTypeDef tim_oc_init = {0};
    HAL_TIM_Base_DeInit(&tim1_handle);
    /* 使能时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    /* IO配置 */
    gpio_init.Pin       = GPIO_PIN_8;        //引脚号
    gpio_init.Mode      = GPIO_MODE_AF_PP;   //模式
    gpio_init.Pull      = GPIO_PULLUP;       //上下拉电阻
    gpio_init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;  //IO口驱动电路响应速度
    gpio_init.Alternate = GPIO_AF1_TIM1;        //复用
    HAL_GPIO_Init(GPIOA, &gpio_init);  //初始化
    /* pwm tim配置 */
    tim1_handle.Instance               = TIM1;               /* 定时器x */
    tim1_handle.Init.Prescaler         = psc;                /* 分频系数 */
    tim1_handle.Init.CounterMode       = TIM_COUNTERMODE_UP; /* 递增计数模式 */
    tim1_handle.Init.Period            = arr;                /* 自动装载值 */
    tim1_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;  /* 时钟分频因子 */
    tim1_handle.Init.RepetitionCounter = 0;                       /* 重复计数器 */
    tim1_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //自动重载预装载使能
    HAL_TIM_PWM_Init(&tim1_handle);
    /* oc配置 */
    tim_oc_init.OCMode       = TIM_OCMODE_PWM1;
    tim_oc_init.Pulse        = 0;
    tim_oc_init.OCPolarity   = TIM_OCPOLARITY_HIGH;
    tim_oc_init.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    tim_oc_init.OCFastMode   = TIM_OCFAST_DISABLE;
    tim_oc_init.OCIdleState  = TIM_OCIDLESTATE_RESET;
    tim_oc_init.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&tim1_handle, &tim_oc_init, TIM_CHANNEL_1);
    /* 启动 */
    HAL_TIM_PWM_Start(&tim1_handle, TIM_CHANNEL_1);  /* 使能定时器x和定时器更新中断 */
}

