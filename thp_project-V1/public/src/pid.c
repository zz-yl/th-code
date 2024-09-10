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
float PidControl(PID_DATA *pid, float target, float freedback)
{
    float err = 0;
    float out = 0;
    
    if(pid == NULL)
    {
        return 0;
    }

    err = target - freedback;
    if((err > -pid->ErrDead) && (err < pid->ErrDead))
    {
        err = 0;
    }
    
    pid->SumE += pid->Ki * err;
    pid->SumE = Limit(pid->SumE, pid->SumELimit, -pid->SumELimit);
    
    out = pid->Kp * err + pid->SumE + pid->Kd * (err - pid->ErrLast);
    out = Limit(out, pid->OutLimit, -pid->OutLimit);
    
    if(out > 0)
    {
        if(out < pid->OutDead)
        {
            out = pid->OutDead;
        }
    }
    else if(out < 0)
    {
        if(out > -pid->OutDead)
        {
            out = -pid->OutDead;
        }
    }

    pid->ErrLast = err;
    pid->Err = err;
    pid->Out = out;
    return out;
}
/**
* @brief  PI控制
*/
float PiControl(PID_DATA *pid, float target, float freedback)
{
    float err = 0;
    float out = 0;
    
    if(pid == NULL)
    {
        return 0;
    }

    err = target - freedback;
    if((err > -pid->ErrDead) && (err < pid->ErrDead))
    {
        err = 0;
    }
    
    pid->SumE += pid->Ki * err;
    pid->SumE = Limit(pid->SumE, pid->SumELimit, -pid->SumELimit);
    
    out = pid->Kp * err + pid->SumE;
    out = Limit(out, pid->OutLimit, -pid->OutLimit);
    
    if(out > 0)
    {
        if(out < pid->OutDead)
        {
            out = pid->OutDead;
        }
    }
    else if(out < 0)
    {
        if(out > -pid->OutDead)
        {
            out = -pid->OutDead;
        }
    }

    pid->Err = err;
    pid->Out = out;
    return out;
}
/**
* @brief  PID清空
*/
void PidClear(PID_DATA *pid)
{
    pid->Err = 0;
    pid->ErrLast = 0;
    pid->SumE = 0;
    pid->Out = 0;
}
