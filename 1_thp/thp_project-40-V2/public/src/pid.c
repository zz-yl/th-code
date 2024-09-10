/**@file   pid.c
* @brief   PID算法
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "pid.h"
#include "data_algorithm.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/


/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/


/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  PID控制
*/
float pid_control(pid_data_t *pid, float target, float freedback)
{
    float err = 0;
    float out = 0;
    
    if(pid == NULL)
    {
        return 0;
    }

    err = target - freedback;
    if((err > -pid->err_dead) && (err < pid->err_dead))
    {
        err = 0;
    }
    
    pid->sum_err += pid->ki * err;
    pid->sum_err = limit(pid->sum_err, pid->sum_limit, -pid->sum_limit);
    
    out = pid->kp * err + pid->sum_err + pid->kd * (err - pid->err_last);
    out = limit(out, pid->out_limit, -pid->out_limit);
    
    if(out > 0)
    {
        if(out < pid->out_dead)
        {
            out = pid->out_dead;
        }
    }
    else if(out < 0)
    {
        if(out > -pid->out_dead)
        {
            out = -pid->out_dead;
        }
    }

    pid->err_last = err;
    pid->err = err;
    pid->out = out;
    return out;
}
/**
* @brief  PI控制
*/
float pi_control(pid_data_t *pid, float target, float freedback)
{
    float err = 0;
    float out = 0;
    
    if(pid == NULL)
    {
        return 0;
    }

    err = target - freedback;
    if((err > -pid->err_dead) && (err < pid->err_dead))
    {
        err = 0;
    }
    
    pid->sum_err += pid->ki * err;
    pid->sum_err = limit(pid->sum_err, pid->sum_limit, -pid->sum_limit);
    
    out = pid->kp * err + pid->sum_err;
    out = limit(out, pid->out_limit, -pid->out_limit);
    
    if(out > 0)
    {
        if(out < pid->out_dead)
        {
            out = pid->out_dead;
        }
    }
    else if(out < 0)
    {
        if(out > -pid->out_dead)
        {
            out = -pid->out_dead;
        }
    }

    pid->err = err;
    pid->out = out;
    return out;
}
/**
* @brief  PID清空
*/
void pid_clear(pid_data_t *pid)
{
    pid->err = 0;
    pid->err_last = 0;
    pid->sum_err = 0;
    pid->out = 0;
}
