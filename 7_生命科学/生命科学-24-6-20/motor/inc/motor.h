/**@file   motor.h
* @brief   485电机控制
* @author  陈卓哲
* @date    2024/6/20
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef MOTOR_H_
#define MOTOR_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_usart.h"
#include "string.h"
#include "data_queue.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define COMM_QUEUE_SIZE  200  //队列大小
#define MOTOR_ADDR  0x01

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
* @struct  motor_data_t
* @brief   电机数据
*/
typedef struct
{
	float cmd_speed;  //速度指令rpm
    uint8_t cmd_en;   //使能指令,1:使能; 0:失能
    
    float fb_speed;   //实际速度rpm
    float fb_cmd_speed; //实际速度指令rpm
    uint8_t fb_en;    //使能状态,1:使能; 0:失能
    union
    {
        uint8_t all;
        struct
        {
            uint8_t pos_err :1;          // 跟随误差报警
            uint8_t over_c_or_low_v :1;  // 过流/低压报警
            uint8_t null :1;
            uint8_t open :1;             // 泵头开盖报警
            uint8_t enc_err :1;          // 编码器错误报警
            uint8_t fall :1;             // 滚轮脱落报警
        }bits;
    }fb_alarm;    //报警:0:正常; 1:跟随误差报警; 2:过流/低压报警; 8:泵头开盖报警; 0x10:编码器错误报警; 0x20:滚轮脱落报警
}motor_data_t;
/**
* @struct test_ctrl_t 
* @brief  控制器测试
*/
typedef struct
{
    float ctrl;
    uint8_t id;
    uint8_t state;
    uint32_t cnt;
    uint8_t arr[50];
    uint16_t arr_len;
}test_ctrl_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern motor_data_t motor_data;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void motor_run(void);
void motor_init(void);

#endif /* MOTOR_H_ */
