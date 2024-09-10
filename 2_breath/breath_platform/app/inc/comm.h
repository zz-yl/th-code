/**@file   comm.h
* @brief   通信模块
* @author  陈卓哲
* @date    2023/3/29
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

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define COMM_QUEUE_SIZE  100  //队列大小

/**
* @enum  comm_ctrl_type_t
* @brief 控制类型
*/
typedef enum
{
    COMM_SPEED,
    COMM_FLOW,
    COMM_FLOW_VALVE
}comm_ctrl_type_t;

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

/**
* @struct  comm_cmd_t
* @brief   通信指令
*/
typedef struct
{
	float flow;
    
    comm_ctrl_type_t type;
}comm_cmd_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern comm_cmd_t comm_cmd;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void comm_wifi_send(uint8_t *data, uint8_t len);
void comm_motor_speed(float speed);
void comm_run(void);
void comm_init(void);

#endif /* COMM_H_ */
