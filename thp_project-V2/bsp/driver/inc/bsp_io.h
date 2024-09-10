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
#define IO_LED_RUN(cmd)     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define IO_LED_RUN_TOGGLE   HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_5)

/* 输出 */
#define IO_LED_R(cmd)   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))  //RGB灯-R
#define IO_LED_G(cmd)   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))  //RGB灯-G
#define IO_LED_B(cmd)   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))  //RGB灯-B
#define IO_POWER(cmd)   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))   //电机电源控制
#define IO_BREAK(cmd)   HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))   //抱闸控制
#define IO_BUZZER(cmd)  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))   //蜂鸣器
#define IO_E2_WP(cmd)   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))   //E2读写使能

#define IO_BUZZER_TOGGLE  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1)   //蜂鸣器

/* 输入 */
#define IO_MOTOR_EN     HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)  //电机使能(脚踏)
#define IO_SCRAM        HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)  //急停按钮

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
