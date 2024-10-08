/**@file   bsp_usart.h
* @brief   串口驱动
* @author  陈卓哲
* @date    2023/11/1
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef BSP_USART_H_
#define BSP_USART_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_sys.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define UART_DMA_RX_SIZE  600  //串口DMA接收数据缓存长度
#define UART_DMA_TX_SIZE  600  //串口DMA发送数据缓存长度

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void uart3_init(void);
void uart4_init(void);
void uart5_init(void);
void uart6_init(void);
void uart7_init(void);
void uart8_init(void);
uint16_t uart3_send(uint8_t *data, uint16_t len);
uint16_t uart4_send(uint8_t *data, uint16_t len);
uint16_t uart5_send(uint8_t *data, uint16_t len);
uint16_t uart6_send(uint8_t *data, uint16_t len);
uint16_t uart7_send(uint8_t *data, uint16_t len);
uint16_t uart8_send(uint8_t *data, uint16_t len);

#endif /* BSP_USART_H_ */
