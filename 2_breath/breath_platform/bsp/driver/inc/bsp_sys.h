/**@file   bsp_sys.h
* @brief   ϵͳ����
* @author  ��׿��
* @date    2024/3/28
* @version 1.00.0.0
**************************************************************************************************/

#ifndef BSP_SYS_H_
#define BSP_SYS_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "core_cm4.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

#define TIM_CLOCK     (SystemCoreClock / 2)

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void sys_cache_enable(void);
uint8_t system_clock_config(void);

#endif /* BSP_SYS_H_ */
