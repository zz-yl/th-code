/**@file   motor.c
* @brief   串口步进电机驱动板控制
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "motor.h"
#include "pid.h"
#include "device.h"
#include "comm.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/


/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

struct
{
    PID_DATA M1;
    PID_DATA M2;
    PID_DATA M3;
    PID_DATA M4;
}MotorPid = 
{
.M1 = 
{
    .Kp        = 5,
    .Ki        = 0,
    .Kd        = 0,
    .ErrDead   = 0.002,
    .SumELimit = 10000,
    .OutLimit  = 10,
    .OutDead   = 0.1,
},
.M2 = 
{
    .Kp        = 5,
    .Ki        = 0,
    .Kd        = 0,
    .ErrDead   = 0.002,
    .SumELimit = 10000,
    .OutLimit  = 10,
    .OutDead   = 0.1,
},
.M3 = 
{
    .Kp        = 5,
    .Ki        = 0,
    .Kd        = 0,
    .ErrDead   = 0.002,
    .SumELimit = 10000,
    .OutLimit  = 10,
    .OutDead   = 0.1,
},
.M4 = 
{
    .Kp        = 5,
    .Ki        = 0,
    .Kd        = 0,
    .ErrDead   = 0.002,
    .SumELimit = 10000,
    .OutLimit  = 10,
    .OutDead   = 0.1,
},
};

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  电机1位置控制
* @attention 
*/
uint8_t Motor1PosCtrl(float tar_pos)
{
    float cmd = 0;
    static uint8_t cnt = 0;
    
    cmd = PidControl(&MotorPid.M1, tar_pos, OutData.m_pos1);
    
    //10次连续指令为0则控制完成,不再发送速度指令
    if(cmd == 0)
    {
        cnt++;
        if(cnt > 10)
        {
            cnt = 10;
            return 1;
        }
    }
    else
    {
        cnt = 0;
    }
    
    MotorSpeedCtrl(cmd, 1);
    
    return 0;
}
/**
* @brief  电机2位置控制
* @attention tar_pos:位置目标mm,返回值:1:控制中,0:控制完成
*/
uint8_t Motor2PosCtrl(float tar_pos)
{
    float cmd = 0;
    static uint8_t cnt = 0;

    cmd = PidControl(&MotorPid.M2, tar_pos, OutData.m_pos2);
    
    //10次连续指令为0则控制完成,不再发送速度指令
    if(cmd == 0)
    {
        cnt++;
        if(cnt > 10)
        {
            cnt = 10;
            return 1;
        }
    }
    else
    {
        cnt = 0;
    }

    MotorSpeedCtrl(cmd, 2);
    
    return 0;
}
/**
* @brief  电机3位置控制
* @attention 
*/
uint8_t Motor3PosCtrl(float tar_pos)
{
    float cmd = 0;
    static uint8_t cnt = 0;
    
    cmd = PidControl(&MotorPid.M3, tar_pos, OutData.m_pos3);
    
    //10次连续指令为0则控制完成,不再发送速度指令
    if(cmd == 0)
    {
        cnt++;
        if(cnt > 10)
        {
            cnt = 10;
            return 1;
        }
    }
    else
    {
        cnt = 0;
    }
    
    MotorSpeedCtrl(cmd, 3);
    
    return 0;
}
/**
* @brief  电机4位置控制
* @attention 
*/
uint8_t Motor4PosCtrl(float tar_pos)
{
    float cmd = 0;
    static uint8_t cnt = 0;
    
    cmd = PidControl(&MotorPid.M4, tar_pos, OutData.m_pos4);
    
    //10次连续指令为0则控制完成,不再发送速度指令
    if(cmd == 0)
    {
        cnt++;
        if(cnt > 10)
        {
            cnt = 10;
            return 1;
        }
    }
    else
    {
        cnt = 0;
    }
    
    MotorSpeedCtrl(cmd, 4);
    
    return 0;
}

