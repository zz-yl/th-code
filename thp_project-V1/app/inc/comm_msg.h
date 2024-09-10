/**@file   comm_msg.h
* @brief   通信,发送信息处理
* @author  陈卓哲
* @date    2023/9/27
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef COMM_MSG_H_
#define COMM_MSG_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "stdint.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void CommTxFillMsg(uint8_t cmd);
void comm_msg_data(uint8_t cmd, uint8_t value);

#endif /* COMM_MSG_H_ */
