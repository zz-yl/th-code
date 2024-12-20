/**@file   bsp_io.h
* @brief   IO口驱动
* @author  陈卓哲
* @date    2023/10/26
* @version 1.00.0.0
**************************************************************************************************/

#ifndef BSP_IO_H_
#define BSP_IO_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_sys.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/* LED小灯 */
#define IO_LED_RUN(cmd)     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define IO_LED_RUN_TOGGLE   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3)

/* 输出 */
#define IO_LED1(cmd)        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED2(cmd)        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED3(cmd)        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED4(cmd)        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED5(cmd)        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED6(cmd)        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED7(cmd)        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED8(cmd)        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,  (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED9(cmd)        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED10(cmd)       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED11(cmd)       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,  (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED12(cmd)       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,  (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED13(cmd)       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,  (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED14(cmd)       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED15(cmd)       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED16(cmd)       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED17(cmd)       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define IO_LED18(cmd)       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, (cmd ? GPIO_PIN_RESET : GPIO_PIN_SET))

#define IO_LED1_TOGGLE      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14)
#define IO_LED2_TOGGLE      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15)
#define IO_LED3_TOGGLE      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12)
#define IO_LED4_TOGGLE      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13)
#define IO_LED5_TOGGLE      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14)
#define IO_LED6_TOGGLE      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15)
#define IO_LED7_TOGGLE      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6)
#define IO_LED8_TOGGLE      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7)
#define IO_LED9_TOGGLE      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8)
#define IO_LED10_TOGGLE     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9)
#define IO_LED11_TOGGLE     HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6)
#define IO_LED12_TOGGLE     HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7)
#define IO_LED13_TOGGLE     HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9)
#define IO_LED14_TOGGLE     HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11)
#define IO_LED15_TOGGLE     HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13)
#define IO_LED16_TOGGLE     HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14)
#define IO_LED17_TOGGLE     HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10)
#define IO_LED18_TOGGLE     HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11)

/* 输入 */
#define IO_KEY1     (!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0))
#define IO_KEY2     (!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1))
#define IO_KEY3     (!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2))
#define IO_KEY4     (!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3))

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
