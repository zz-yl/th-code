/**@file   comm.h
* @brief   通信模块
* @author  陈卓哲
* @date    2023/9/5
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef COMM_H_
#define COMM_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_cfg.h"
#include "string.h"
#include "data_queue.h"
#include "comm_protocol.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define COMM_QUEUE_SIZE  600  //队列大小

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  comm_data_t
* @brief   通信数据
*/
typedef struct
{
	queue_list_t tx;  //发送队列
    queue_list_t rx;  //接收队列
}comm_data_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void comm_fill_msg(uint8_t cmd, uint8_t *data, uint8_t len);
void comm_run(void);
void comm_init(void);

#endif /* COMM_H_ */
