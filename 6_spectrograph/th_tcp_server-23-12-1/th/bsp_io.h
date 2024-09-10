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

//#include "bsp_sys.h"
#include "./SYSTEM/sys/sys.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/* LEDС�� */
#define IO_LED_RUN(x)       do{ HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, (x ? GPIO_PIN_SET : GPIO_PIN_RESET)); }while(0)
#define IO_LED_RUN_TOGGLE   HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2)

/* ��� */
#define IO_OUT1(x)   do{ HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, ((!x) ? GPIO_PIN_SET : GPIO_PIN_RESET)); }while(0)
#define IO_OUT2(x)   do{ HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, ((!x) ? GPIO_PIN_SET : GPIO_PIN_RESET)); }while(0)
#define IO_OUT3(x)   do{ HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, ((!x) ? GPIO_PIN_SET : GPIO_PIN_RESET)); }while(0)

/* ���� */
#define IO_FOOTREST1    (!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1))  //��̤1
#define IO_FOOTREST2    (!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2))  //��̤2

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
