/**@file   motor.c
* @brief   ���ڲ���������������
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
* @brief  ���1λ�ÿ���
* @attention 
*/
uint8_t Motor1PosCtrl(float tar_pos)
{
    float cmd = 0;
    static uint8_t cnt = 0;
    
    cmd = PidControl(&MotorPid.M1, tar_pos, OutData.m_pos1);
    
    //10������ָ��Ϊ0��������,���ٷ����ٶ�ָ��
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
* @brief  ���2λ�ÿ���
* @attention tar_pos:λ��Ŀ��mm,����ֵ:1:������,0:�������
*/
uint8_t Motor2PosCtrl(float tar_pos)
{
    float cmd = 0;
    static uint8_t cnt = 0;

    cmd = PidControl(&MotorPid.M2, tar_pos, OutData.m_pos2);
    
    //10������ָ��Ϊ0��������,���ٷ����ٶ�ָ��
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
* @brief  ���3λ�ÿ���
* @attention 
*/
uint8_t Motor3PosCtrl(float tar_pos)
{
    float cmd = 0;
    static uint8_t cnt = 0;
    
    cmd = PidControl(&MotorPid.M3, tar_pos, OutData.m_pos3);
    
    //10������ָ��Ϊ0��������,���ٷ����ٶ�ָ��
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
* @brief  ���4λ�ÿ���
* @attention 
*/
uint8_t Motor4PosCtrl(float tar_pos)
{
    float cmd = 0;
    static uint8_t cnt = 0;
    
    cmd = PidControl(&MotorPid.M4, tar_pos, OutData.m_pos4);
    
    //10������ָ��Ϊ0��������,���ٷ����ٶ�ָ��
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

