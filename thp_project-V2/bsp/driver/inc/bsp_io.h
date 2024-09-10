/**@file   bsp_io.h
* @brief   IO������
* @author  ��׿��
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

/* LEDС�� */
#define IO_LED_RUN(cmd)     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define IO_LED_RUN_TOGGLE   HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_5)

/* ��� */
#define IO_LED_R(cmd)   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))  //RGB��-R
#define IO_LED_G(cmd)   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))  //RGB��-G
#define IO_LED_B(cmd)   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))  //RGB��-B
#define IO_POWER(cmd)   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))   //�����Դ����
#define IO_BREAK(cmd)   HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))   //��բ����
#define IO_BUZZER(cmd)  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))   //������
#define IO_E2_WP(cmd)   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))   //E2��дʹ��

#define IO_BUZZER_TOGGLE  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1)   //������

/* ���� */
#define IO_MOTOR_EN     HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)  //���ʹ��(��̤)
#define IO_SCRAM        HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)  //��ͣ��ť

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
