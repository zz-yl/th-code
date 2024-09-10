/**@file   valve.c
* @brief   valve
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "valve.h"
#include "comm.h"
#include "device.h"
#include "motor.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/


/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

valve_data_t valve_data = 
{
    .pid.kp = 0.0f,
    .pid.ki = 0.01f,
    .pid.kd = 0.0f,
    .pid.err_dead  = 0.0f,
    .pid.sum_limit = 1000.0f,
    .pid.out_limit = 100.0f,
    .pid.out_dead  = 0.1f,
};

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  比例阀开度控制
* @attention 
*/
static void valve_ctrl(void)
{
    float per = valve_data.out;

    if(per > 100)
    {
        per = 100;
    }
    valve_data.cmp = per * 0.01f * VALVE_MAX;
    bsp_valve_ctrl(valve_data.cmp);
}
/**
* @brief  比例阀pid控制
* @attention 
*/
void valve_pid(void)
{
    static float cmd = 0;
    
    valve_data.cmd_flow = comm_cmd.flow;
    if(cmd != valve_data.cmd_flow)
    {
        cmd = valve_data.cmd_flow;
//        pid_clear(&valve_data.pid);
    }
    
    valve_data.out = pid_control(&valve_data.pid, valve_data.cmd_flow, dev_data.flow_real);
    valve_data.out += 30;
    valve_ctrl();
}
/**
* @brief  比例阀运行
* @attention 
*/
void valve_run(void)
{
//    static uint8_t tim = 0;
//    
//    tim++;
//    if(tim < 10)
//    {
//        return;
//    }
//    tim = 0;

    switch(comm_cmd.type)
    {
        case COMM_SPEED: valve_ctrl(); break;
        case COMM_FLOW:  valve_pid(); break;
        case COMM_FLOW_VALVE:  valve_pid(); break;
        default: break;
    }
}

