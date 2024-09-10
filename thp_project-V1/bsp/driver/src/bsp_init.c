/**@file   bsp_init.c
* @brief   bsp初始化
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_init.h"
#include "bsp_cfg.h"
#include "control.h"
#include "memory.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/


/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/


/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  bsp初始化
* @attention 
*/
void BspInit(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //中断分组配置
    
    DelayInit();
    BspIOInit();
    TIM4Init(1000-1, 84-1);
    
    if(GPIO_ReadInputDataBit(IN1_GPIO_Port,IN1_Pin)==0)
    {
//        calibration();
    }
    else
    {
        MemRead();
    }

    Usart1Init();
    Usart2Init();
    Usart3Init();
    Uart4Init();
    Usart6Init();
    
    Ads131Init();
    Spi1Init();
    adcStartup();
}
