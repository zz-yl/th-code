/**@file   sys_state_machine.c
* @brief   系统状态机
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "sys_state_machine.h"
#include "state_machine.h"
#include "stddef.h"
#include "motor_comm.h"

#include "motor_state_machine.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

sm_sys_port_t sm_sys_port;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/* self_check */
static void work_self_check(void)
{
    sm_sys_port.cmd = SYS_STANDBY;
}
static uint16_t trig_self_check(void)
{
    if(sm_sys_port.cmd == SYS_STANDBY)
    {
        return sm_sys_port.cmd;
    }
    return STATE_NO_TRIG;
}
static void entry_self_check(void)
{
    
}
static void exit_self_check(void)
{
    
}
/* standby */
static void work_standby(void)
{
    
}
static uint16_t trig_standby(void)
{
    if(sm_sys_port.cmd != SYS_STANDBY)
    {
        return sm_sys_port.cmd;
    }
    return STATE_NO_TRIG;
}
static void entry_standby(void)
{
//    motor_en(0);
    sm_motor_toggle(MOTOR_STATE_STOP);
}
static void exit_standby(void)
{
    
}
/* calibrate */
static void work_calibrate(void)
{
    
}
static uint16_t trig_calibrate(void)
{
    if(sm_sys_port.cmd == SYS_STANDBY)
    {
        return sm_sys_port.cmd;
    }
    return STATE_NO_TRIG;
}
static void entry_calibrate(void)
{
//    motor_en(0);
    sm_motor_toggle(MOTOR_STATE_STOP);
}
static void exit_calibrate(void)
{
    
}
/* engineer */
static void work_engineer(void)
{
    
}
static uint16_t trig_engineer(void)
{
    if(sm_sys_port.cmd == SYS_STANDBY)
    {
        return sm_sys_port.cmd;
    }
    return STATE_NO_TRIG;
}
static void entry_engineer(void)
{
    
}
static void exit_engineer(void)
{
    
}
/* test */
static void work_test(void)
{
    
}
static uint16_t trig_test(void)
{
    if(sm_sys_port.cmd == SYS_STANDBY)
    {
        return sm_sys_port.cmd;
    }
    return STATE_NO_TRIG;
}
static void entry_test(void)
{
    
}
static void exit_test(void)
{
    
}
/**
* @brief  系统状态机描述符
*/
state_machine_t sm_sys;
/**
* @brief  系统状态列表
*/
static state_node_t sm_sys_table[] = 
{
    {.index = SYS_SELF_CHECK,   .work = work_self_check,    .trig = trig_self_check,    .entry = entry_self_check,  .exit = exit_self_check},
    {.index = SYS_STANDBY,      .work = work_standby,       .trig = trig_standby,       .entry = entry_standby,     .exit = exit_standby},
    {.index = SYS_CALIBRATE,    .work = work_calibrate,     .trig = trig_calibrate,     .entry = entry_calibrate,   .exit = exit_calibrate},
    {.index = SYS_ENGINEER,     .work = work_engineer,      .trig = trig_engineer,      .entry = entry_engineer,    .exit = exit_engineer},
    {.index = SYS_TEST,         .work = work_test,          .trig = trig_test,          .entry = entry_test,        .exit = exit_test},
};
/**
* @brief  系统状态机初始化
*/
void sm_sys_init(void)
{
    state_register(&sm_sys, sm_sys_table, sizeof(sm_sys_table) / sizeof(sm_sys_table[0]));
    sm_sys_port.state = (sm_sys_state_t)sm_sys.current->index;
}
/**
* @brief  系统状态机运行
*/
void sm_sys_run(void)
{
    if(state_machine_run(&sm_sys) != STATE_NO_TRIG)
    {
        sm_sys_port.state = (sm_sys_state_t)sm_sys.current->index;
    }
}
/**
* @brief  系统状态机切换
* @attention 
*/
void sm_sys_toggle(sm_sys_state_t state)
{
    sm_sys_port.cmd = state;
}
/**
* @brief  获取系统状态机状态
* @attention 
*/
sm_sys_state_t sm_sys_state(void)
{
    return sm_sys_port.state;
}
