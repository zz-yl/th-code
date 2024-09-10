/**@file   motor_comm.h
* @brief   串口电机控制(韩国电机)
* @author  陈卓哲
* @date    2024/2/20
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef MOTOR_COMM_H_
#define MOTOR_COMM_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_cfg.h"
#include "data_queue.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**
* @enum    motor_id_t
* @brief   电机id
*/
typedef enum
{
    MOTOR_1,
    MOTOR_2,
    MOTOR_3,
    MOTOR_4,
    MOTOR_5,
}motor_id_t;
/**
* @enum    motor_cmd_mode_t
* @brief   电机指令模式
*/
typedef enum
{
    MOTOR_POS,   //位置模式
    MOTOR_SPEED, //速度模式
}motor_cmd_mode_t;
/**
* @enum    motor_state_run_t
* @brief   电机运行状态
*/
typedef enum
{
    MOTOR_STOP,
    MOTOR_RUN,
    MOTOR_FAULT,
}motor_state_run_t;

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  motor_t
* @brief   电机描述符
*/
typedef struct motor_node
{
    float cmd_pos;       //位置指令(mm)
    float pos;           //电机位置(mm)
    float speed;         //电机速度(mm/s)
    uint32_t pos_all;    //电机总行程(mm)
    motor_state_run_t state_r; //电机运行状态
    uint32_t tim_run;   //电机运行总时间

    struct motor_node *next;
}motor_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern motor_t motor_data[5];

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void motor_init(void);
void motor_run(void);
void motor_get_pos_all(void);
void motor_send_pos_all(void);
void motor_ctrl_pos(motor_id_t id, float pos);
void motor_monitor(void);
void motor_fill_msg(motor_id_t id, uint16_t cmd, uint8_t *data, uint8_t len);

#endif /* MOTOR_COMM_H_ */
