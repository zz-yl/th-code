/**@file   bsp_timer.c
* @brief   定时器\PWM驱动
*          基本定时器:TIM6,TIM7
*          通用定时器:TIM2~TIM5,TIM12~TIM17
*          高级定时器:TIM1,TIM8
*          低功耗定时器:LPTIM1~LPTIM5
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_timer.h"

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
/* 结构体初始化 */
TIM_HandleTypeDef tim6_handle = {0};
TIM_HandleTypeDef tim7_handle = {0};

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  TIM6初始化
* @attention 
*/
void tim6_init(uint16_t arr, uint16_t psc)
{
    /* 使能时钟 */
    __HAL_RCC_TIM6_CLK_ENABLE();
    /* 定时器配置 */
    tim6_handle.Instance               = TIM6;               /* 定时器x */
    tim6_handle.Init.Prescaler         = psc;                /* 分频系数 */
    tim6_handle.Init.CounterMode       = TIM_COUNTERMODE_UP; /* 递增计数模式 */
    tim6_handle.Init.Period            = arr;                /* 自动装载值 */
    tim6_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;  /* 时钟分频因子 */
    tim6_handle.Init.RepetitionCounter = 0;                       /* 重复计数器 */
    tim6_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //自动重载预装载使能
    HAL_TIM_Base_Init(&tim6_handle);
    /* 定时器中断配置 */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 3);  /* 抢占1，子优先级3 */
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);          /* 开启ITMx中断 */
    /* 启动 */
    HAL_TIM_Base_Start_IT(&tim6_handle);  /* 使能定时器x和定时器更新中断 */
}
/**
* @brief  TIM6中断
* @attention 
*/
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&tim6_handle);  /* 定时器回调函数 */
}
/**
* @brief  TIM7初始化
* @attention 
*/
void tim7_init(uint16_t arr, uint16_t psc)
{
    /* 使能时钟 */
    __HAL_RCC_TIM7_CLK_ENABLE();
    /* 定时器配置 */
    tim7_handle.Instance               = TIM7;               /* 定时器x */
    tim7_handle.Init.Prescaler         = psc;                /* 分频系数 */
    tim7_handle.Init.CounterMode       = TIM_COUNTERMODE_UP; /* 递增计数模式 */
    tim7_handle.Init.Period            = arr;                /* 自动装载值 */
    tim7_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;  /* 时钟分频因子 */
    tim7_handle.Init.RepetitionCounter = 0;                       /* 重复计数器 */
    tim7_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //自动重载预装载使能
    HAL_TIM_Base_Init(&tim7_handle);
    /* 定时器中断配置 */
    HAL_NVIC_SetPriority(TIM7_IRQn, 1, 3);  /* 抢占1，子优先级3 */
    HAL_NVIC_EnableIRQ(TIM7_IRQn);          /* 开启ITMx中断 */
    /* 启动 */
    HAL_TIM_Base_Start_IT(&tim7_handle);  /* 使能定时器x和定时器更新中断 */
}
/**
* @brief  TIM6中断
* @attention 
*/
void TIM7_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&tim7_handle);  /* 定时器回调函数 */
}
/**
 * @brief       回调函数，定时器中断服务函数调用
 * @param       htim  : 定时器句柄
 * @retval      无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        sys_timer++;  /* 系统时间ms */
    }
    else if (htim->Instance == TIM7)
    {
        FreeRTOSRunTimeTicks++;  /* freertos用于统计cpu占用率计时 */
    }
}
/**
* @brief  freertos用于统计cpu占用率的初始化函数
* @attention 
*/
void ConfigureTimeForRunTimeStats(void)
{
    tim7_init(100-1, 240-1);  // frq = 240M/(psc+1)/(arr+1)
}
