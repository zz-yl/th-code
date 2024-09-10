/**@file   motor_state_machine.c
* @brief   电机控制状态机
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "motor_state_machine.h"
#include "state_machine.h"
#include "stddef.h"

#include "motor.h"
#include "control.h"
#include "sys_state_machine.h"

#include "thp_cfg.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

sm_motor_port_t sm_motor_port;
uint16_t sm_lock_tims = 0;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/* stop */
static void work_stop(void)
{
    
}
static uint16_t trig_stop(void)
{
    if(sm_motor_port.cmd != MOTOR_STATE_STOP)
    {
        return sm_motor_port.cmd;
    }
    return STATE_NO_TRIG;
}
static void entry_stop(void)
{
    motor_stop_all();
    ctrl_led(CTRL_GREEN);
    IO_BREAK(1);
}
static void exit_stop(void)
{
    
}
/* run */
static void work_run(void)
{
    motor_run();
}
static uint16_t trig_run(void)
{
    static uint16_t run_tim = 0;
    
    motor_data.tim_run++;
#ifdef MOTOR_RECOVER
    if(sm_lock_tims)
    {
        run_tim++;
        if(run_tim > 5000)
        {
            run_tim = 0;
            sm_lock_tims = 0;
        }
    }
#endif
    if((motor_read_lock()) && (sm_sys_state() != SYS_CALIBRATE) && (sm_sys_state() != SYS_TEST))
    {
#ifdef MOTOR_RECOVER
        if(sm_lock_tims >= 15)
        {
            sm_lock_tims = 0;
#endif
            sm_motor_port.cmd = MOTOR_STATE_LOCKED;
#ifdef MOTOR_RECOVER
        }
        else
        {
            sm_motor_port.cmd = MOTOR_STATE_RECOVER;
        }
#endif
    }
    if((motor_data.m1.state_r == MOTOR_STOP)
        && (motor_data.m2.state_r == MOTOR_STOP)
        && (motor_data.m3.state_r == MOTOR_STOP)
        && (motor_data.m4.state_r == MOTOR_STOP))
    {
        sm_motor_toggle(MOTOR_STATE_STOP);
    }
    if(sm_motor_port.cmd != MOTOR_STATE_RUN)
    {
        return sm_motor_port.cmd;
    }
    return STATE_NO_TRIG;
}
static void entry_run(void)
{
    ctrl_led(CTRL_BLUE);
    IO_BREAK(0);
    motor_data.m1.state_r = MOTOR_RUN;
    motor_data.m2.state_r = MOTOR_RUN;
    motor_data.m3.state_r = MOTOR_RUN;
    motor_data.m4.state_r = MOTOR_RUN;
}
static void exit_run(void)
{

}
/* locked */
static void work_locked(void)
{
    
}
static uint16_t trig_locked(void)
{
    if(motor_read_lock() == 0)
    {
        sm_motor_port.cmd = MOTOR_STATE_STOP;
    }
    if(sm_motor_port.cmd == MOTOR_STATE_STOP)
    {
        return sm_motor_port.cmd;
    }
    return STATE_NO_TRIG;
}
static void entry_locked(void)
{
    motor_stop_all();
    ctrl_led(CTRL_YELLOW);
}
static void exit_locked(void)
{
    
}
/* recover */
static void work_recover(void)
{
    
}
static uint16_t trig_recover(void)
{
    static uint16_t tim = 0;
    
    tim++;
    if(tim > 100)
    {
        //motor_en(1);
        IO_POWER(1);
    }
    if(tim > 200)
    {
        tim = 0;
        sm_motor_port.cmd = MOTOR_STATE_RUN;
    }
    if(sm_motor_port.cmd != MOTOR_STATE_RECOVER)
    {
        return sm_motor_port.cmd;
    }
    return STATE_NO_TRIG;
}
static void entry_recover(void)
{
    motor_stop_all();
    if(motor_data.m1.state_l == MOTOR_LOCKED)
    {
        motor_data.m1.cnt_recover++;
    }
    if(motor_data.m2.state_l == MOTOR_LOCKED)
    {
        motor_data.m2.cnt_recover++;
    }
    if(motor_data.m3.state_l == MOTOR_LOCKED)
    {
        motor_data.m3.cnt_recover++;
    }
    if(motor_data.m4.state_l == MOTOR_LOCKED)
    {
        motor_data.m4.cnt_recover++;
    }
    
    motor_clear_lock();
    //motor_en(0);
    IO_POWER(0);
    sm_lock_tims++;
}
static void exit_recover(void)
{
    
}
/* fault */
static void work_fault(void)
{
    
}
static uint16_t trig_fault(void)
{
    if(sm_motor_port.cmd == MOTOR_STATE_STOP)
    {
        return sm_motor_port.cmd;
    }
    return STATE_NO_TRIG;
}
static void entry_fault(void)
{
    motor_stop_all();
}
static void exit_fault(void)
{
    
}

/**
* @brief  电机状态机描述符
*/
state_machine_t sm_motor;
/**
* @brief  电机状态列表
*/
static state_node_t sm_motor_table[] = 
{
    {.index = MOTOR_STATE_STOP,     .work = work_stop,      .trig = trig_stop,      .entry = entry_stop,        .exit = exit_stop},
    {.index = MOTOR_STATE_RUN,      .work = work_run,       .trig = trig_run,       .entry = entry_run,         .exit = exit_run},
    {.index = MOTOR_STATE_LOCKED,   .work = work_locked,    .trig = trig_locked,    .entry = entry_locked,      .exit = exit_locked},
    {.index = MOTOR_STATE_RECOVER,  .work = work_recover,   .trig = trig_recover,   .entry = entry_recover,     .exit = exit_recover},
    {.index = MOTOR_STATE_FAULT,    .work = work_fault,     .trig = trig_fault,     .entry = entry_fault,       .exit = exit_fault},
};
/**
* @brief  电机状态机初始化
*/
void sm_motor_init(void)
{
    state_register(&sm_motor, sm_motor_table, sizeof(sm_motor_table) / sizeof(sm_motor_table[0]));
    sm_motor_port.state = (sm_motor_state_t)sm_motor.current->index;
}
/**
* @brief  电机状态机删除
*/
void sm_motor_delete(void)
{
    sm_motor.current = &sm_motor.table[0];
    sm_motor_port.cmd = MOTOR_STATE_STOP;
    sm_motor_port.state = MOTOR_STATE_STOP;
}
/**
* @brief  电机状态机运行
*/
void sm_motor_run(void)
{
    if(state_machine_run(&sm_motor) != STATE_NO_TRIG)
    {
        sm_motor_port.state = (sm_motor_state_t)sm_motor.current->index;
    }
}
/**
* @brief  电机状态机切换
* @attention 
*/
void sm_motor_toggle(sm_motor_state_t cmd)
{
    if(sm_motor_port.state == MOTOR_STATE_LOCKED)
    {
        return;
    }
    sm_motor_port.cmd = cmd;
}
/**
* @brief  获取电机状态机状态
* @attention 
*/
sm_motor_state_t sm_motor_state(void)
{
    return sm_motor_port.state;
}
