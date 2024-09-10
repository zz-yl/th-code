/**@file   bsp_io.h
* @brief   IO口驱动
* @author  陈卓哲
* @date    2023/8/30
* @section 参考文档
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef BSP_IO_H_
#define BSP_IO_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "stm32f4xx.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**
* @enum 	IO_ID 
* @brief  	IO控制ID
*/
typedef enum
{
    IO_FREE,         ///< 空
    IO_LED_RUN    = 1,  ///< 运行灯
    IO_POWER_ON   = 2,  ///< 电源
    IO_OUT1_ON    = 3,  ///< 输出1
    IO_OUT2_ON    = 4,  ///< 输出2
    IO_OUT3_ON    = 5,  ///< 输出3
    IO_OUT4_ON    = 6,  ///< 输出4
    IO_RS485_TX   = 7,  ///< 485发送使能
    IO_READ_IN1   = 8,  ///< 输入1
    IO_READ_IN2   = 9,  ///< 输入2
    
    IO_END
}IO_ID;

/* LED小灯 */
#define LED_RUN_ON      GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define LED_RUN_OFF     GPIO_ResetBits(GPIOC, GPIO_Pin_13)
#define LED_RUN_TOGGLE  GPIO_ToggleBits(GPIOC, GPIO_Pin_13)

/* 输出 */
#define POWER_ON        GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define POWER_OFF       GPIO_ResetBits(GPIOB, GPIO_Pin_1)
#define OUT1_ON         GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define OUT1_OFF        GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define OUT2_ON         GPIO_SetBits(GPIOB, GPIO_Pin_13)
#define OUT2_OFF        GPIO_ResetBits(GPIOB, GPIO_Pin_13)
#define OUT3_ON         GPIO_SetBits(GPIOB, GPIO_Pin_14)
#define OUT3_OFF        GPIO_ResetBits(GPIOB, GPIO_Pin_14)
#define OUT4_ON         GPIO_SetBits(GPIOB, GPIO_Pin_15)
#define OUT4_OFF        GPIO_ResetBits(GPIOB, GPIO_Pin_15)
#define RS485_TX        GPIO_SetBits(GPIOC, GPIO_Pin_8)
#define RS485_RX        GPIO_ResetBits(GPIOC, GPIO_Pin_8)

/* 输入 */
#define READ_IN1        GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)
#define READ_IN2        GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void BspIOInit(void);
uint8_t BspIOCtrl(uint16_t id, uint8_t cmd);

#endif /* BSP_IO_H_ */
