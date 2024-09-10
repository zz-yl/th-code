/**@file   bsp_motor.h
* @brief   电机驱动相关外设配置
* @author  陈卓哲
* @date    2023/11/15
* @section 参考文档
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef BSP_MOTOR_H_
#define BSP_MOTOR_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_sys.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define M1_DIR(cmd)   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define M1_EN(cmd)    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define M1_SLEEP(cmd) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define M1_FAULT      HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6)

#define M2_DIR(cmd)   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define M2_EN(cmd)    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define M2_SLEEP(cmd) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define M2_FAULT      HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)

#define M3_DIR(cmd)   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define M3_EN(cmd)    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define M3_SLEEP(cmd) HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define M3_FAULT      HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2)

#define M4_DIR(cmd)   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define M4_EN(cmd)    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define M4_SLEEP(cmd) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define M4_FAULT      HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)

#define M5_DIR(cmd)   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define M5_EN(cmd)    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define M5_SLEEP(cmd) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define M5_FAULT      HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14)

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void motor1_init(void);
void motor2_init(void);
void motor3_init(void);
void motor4_init(void);
void motor5_init(void);
void motor1_step_frq(float frq);
void motor2_step_frq(float frq);
void motor3_step_frq(float frq);
void motor4_step_frq(float frq);
void motor5_step_frq(float frq);

#endif /* BSP_MOTOR_H_ */
