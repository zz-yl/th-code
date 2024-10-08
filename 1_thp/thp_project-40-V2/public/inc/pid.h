/**@file   pid.h
* @brief   PID算法
* @author  陈卓哲
* @date    2023/9/5
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef PID_H_
#define PID_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "math.h"
#include "stdio.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  pid_data_t
* @brief   PID数据
*/
typedef struct
{
    float err;      //< 本次误差
    float err_last; //< 上次误差
    float kp;       //< kp
    float ki;       //< ki
    float kd;       //< kd
    float sum_err;  //< 误差积分
    float out;      //< 输出

    const float err_dead;   //< 误差死区
    const float sum_limit;  //< 误差积分边界
    const float out_limit;  //< 输出边界
    const float out_dead;   //< 输出死区
}pid_data_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

float pid_control(pid_data_t *pid, float target, float freedback);
float pi_control(pid_data_t *pid, float target, float freedback);
void pid_clear(pid_data_t *pid);

#endif /* PID_H_ */
