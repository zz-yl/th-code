/**@file   motor.h
* @brief   步进电机控制
* @author  陈卓哲
* @date    2023/11/15
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

#include "bsp_cfg.h"
#include "pid.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define MOTOR_MICROSTEP  256   //步进电机驱动器细分

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
/**
* @enum    motor_state_lock_t
* @brief   电机堵转状态
*/
typedef enum
{
    MOTOR_UNLOCKED,
    MOTOR_LOCKED,
}motor_state_lock_t;

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  motor_t
* @brief   电机描述符
*/
typedef struct motor_node
{
    motor_id_t id;       //电机id
    float pos;           //电机位置(mm)
    float pos_init;      //电机上电初始位置(mm)
    float pos_last;      //电机上次位置(mm)
    float speed;         //电机速度(mm/s)
    uint32_t pos_all;    //电机总行程(mm)
    motor_state_run_t state_r; //电机运行状态
    motor_state_lock_t state_l; //电机堵转状态
    uint16_t cnt_lock;   //堵转计数
    uint16_t cnt_stop;   //停止计数
    uint32_t cnt_recover; //堵转恢复次数计数
    motor_cmd_mode_t cmd_mode;  //电机指令模式
    float cmd_pos;       //位置指令(mm)
    float cmd_speed;     //速度指令(mm/s)
    float cmd_pos_last;  //上次位置指令(mm)
    float cmd_speed_last;//上次速度指令(mm/s)
    float cmd_speed_limit;//速度指令上限(mm/s)
    float cmd_frq;       //频率指令(Hz)
    float cmd_speed_target;//速度目标
    pid_data_t *pid;     //pid参数
    void (*init)(void);  //电机初始化函数
    void (*pid_pos)(struct motor_node *);  //电机位置闭环
    void (*set_frq)(float);  //脉冲频率输出
    
    struct motor_node *next;
}motor_t;

/**
* @struct  motor_data_t
* @brief   电机数据
*/
typedef struct
{
    motor_t m1;
    motor_t m2;
    motor_t m3;
    motor_t m4;
    motor_t m5;
    float tim_run;  //单次运行的时间
}motor_data_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern motor_data_t motor_data;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void motor_write_mode(motor_cmd_mode_t mode);
void motor_clear_lock(void);
void motor_stop_all(void);
void motor_init(void);
void motor_run(void);
void motor_monitor(void);
void motor_init_pos(void);
void motor_en(uint8_t cmd);
void motor_dir(motor_id_t id, uint8_t cmd);
uint8_t motor_read_lock(void);

void motor_pos_ctrl(motor_id_t id, float pos);
void motor_speed_ctrl(motor_id_t id, float speed);

#endif /* MOTOR_H_ */
