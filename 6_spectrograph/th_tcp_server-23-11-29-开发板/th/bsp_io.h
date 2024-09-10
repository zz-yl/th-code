/**@file   bsp_io.h
* @brief   IO¿ÚÇý¶¯
* @author  ³Â×¿ÕÜ
* @date    2023/10/26
* @version 1.00.0.0
**************************************************************************************************/

#ifndef BSP_IO_H_
#define BSP_IO_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

//#include "bsp_sys.h"
#include "./SYSTEM/sys/sys.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/* LEDÐ¡µÆ */
#define IO_LED_RUN(cmd)     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, (GPIO_PinState)cmd)
#define IO_LED_RUN_TOGGLE   HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3)

/* Êä³ö */
#define IO_OUT1(x)   do{ x ? HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET); }while(0)
#define IO_OUT2(x)   do{ x ? HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET); }while(0)
#define IO_OUT3(x)   do{ x ? HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET); }while(0)

/* ÊäÈë */
#define IO_FOOTREST1    (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))  //½ÅÌ¤1
#define IO_FOOTREST2    (!HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_9))  //½ÅÌ¤2

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void bsp_io_init(void);

#endif /* BSP_IO_H_ */
