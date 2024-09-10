/**@file   motor.c
* @brief   motor
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "motor.h"
#include "comm.h"
#include "device.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/


/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

motor_data_t motor_data = 
{
    .pid.kp = 0.0f,
    .pid.ki = 0.3f,
    .pid.kd = 0.0f,
    .pid.err_dead  = 1.0f,
    .pid.sum_limit = 38000.0f,
    .pid.out_limit = 38000.0f,
    .pid.out_dead  = 1.0f,
};

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  电机加速度控制
* @attention 
*/
static float motor_aspeed(float speed)
{
    if((speed - motor_data.cmd_real) > 20)
    {
        motor_data.cmd_real += 20;
    }
    else if((speed - motor_data.cmd_real) < -20)
    {
        motor_data.cmd_real -= 20;
    }
    else
    {
        motor_data.cmd_real = speed;
    }
    
    return motor_data.cmd_real;
}
/**
* @brief  电机速度控制
* @attention 
*/
static void motor_speed_ctrl(void)
{
    float speed = motor_data.cmd;
    float speed_real = 0;
    static uint8_t cnt = 0;
    
    if((speed < 5000) || (speed > 60000))
    {
        speed = 5000;
    }
    
    speed -= 5000;
    speed_real = motor_aspeed(speed);
    
    if(speed_real == speed)
    {
        cnt++;
        if(cnt > 2)
        {
            cnt = 2;
            return;
        }
    }
    else
    {
        cnt = 0;
    }
    
    comm_motor_speed(speed_real);
}
/**
* @brief  电机pid控制
* @attention 
*/
void motor_speed_pid(void)
{
    static float cmd = 0;
    
    motor_data.cmd_flow = comm_cmd.flow;
    if(cmd != motor_data.cmd_flow)
    {
        cmd = motor_data.cmd_flow;
        pid_clear(&motor_data.pid);
    }
    
    motor_data.speed = pid_control(&motor_data.pid, motor_data.cmd_flow, dev_data.flow_real);
    comm_motor_speed(motor_data.speed);
}
/**
* @brief  电机运行
* @attention 
*/
void motor_run(void)
{
    switch(comm_cmd.type)
    {
        case COMM_SPEED: motor_speed_ctrl(); break;
        case COMM_FLOW:  motor_speed_pid(); break;
        case COMM_FLOW_VALVE:  motor_speed_ctrl(); break;
        default: break;
    }
}

