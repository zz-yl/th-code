/**@file   motor.c
* @brief   步进电机控制
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "motor.h"
#include "device_core.h"
#include "device_io.h"

#include "sys_state_machine.h"
#include "motor_state_machine.h"
#include "control.h"
#include "thp_cfg.h"

#include "FreeRTOS.h"
#include "task.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/


/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

uint8_t g_lock_open = 1;

/* pid位置闭环
 * 控制频率为1kHz,则输出脉冲频率最低为1kHz,即在64细分下最低转速为1000/200/64=0.078125(mm/s),
 * 设定最低转速:out_dead=0.1(mm/s)
 * 在1kHz控制频率下,每周期1ms内电机最低行程0.0001(mm),预留10倍控制余量
 * 设定位置死区:err_dead=0.001(mm) */
struct
{
    pid_data_t m1;
    pid_data_t m2;
    pid_data_t m3;
    pid_data_t m4;
    pid_data_t m5;
}motor_pid = 
{
.m1 = 
{
    .kp        = 5,
    .ki        = 0,
    .kd        = 0,
    .err_dead  = 0.001,
    .sum_limit = 10000,
    .out_limit = 5,
    .out_dead  = 0.2,
},
.m2 = 
{
    .kp        = 5,
    .ki        = 0,
    .kd        = 0,
    .err_dead  = 0.001,
    .sum_limit = 10000,
    .out_limit = 5,
    .out_dead  = 0.2,
},
.m3 = 
{
    .kp        = 5,
    .ki        = 0,
    .kd        = 0,
    .err_dead  = 0.001,
    .sum_limit = 10000,
    .out_limit = 5,
    .out_dead  = 0.2,
},
.m4 = 
{
    .kp        = 5,
    .ki        = 0,
    .kd        = 0,
    .err_dead  = 0.001,
    .sum_limit = 10000,
    .out_limit = 5,
    .out_dead  = 0.2,
},
.m5 = 
{
    .kp        = 5,
    .ki        = 0,
    .kd        = 0,
    .err_dead  = 0.001,
    .sum_limit = 10000,
    .out_limit = 5,
    .out_dead  = 0.1,
},
};
motor_t motor_head;
motor_data_t motor_data;
float *motor_cmd[] =
{
    &motor_data.m1.cmd_pos,
    &motor_data.m2.cmd_pos,
    &motor_data.m3.cmd_pos,
    &motor_data.m4.cmd_pos,
    &motor_data.m5.cmd_pos,
};

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  电机位置控制
* @attention 
*/
void motor_pos_ctrl(motor_id_t id, float pos)
{
    motor_t *motor = NULL;
    
    switch(id)
    {
        case MOTOR_1: motor = &motor_data.m1; break;
        case MOTOR_2: motor = &motor_data.m2; break;
        case MOTOR_3: motor = &motor_data.m3; break;
        case MOTOR_4: motor = &motor_data.m4; break;
        case MOTOR_5: motor = &motor_data.m5; break;
        default: break;
    }
    motor->cmd_pos = pos;
    motor_write_mode(MOTOR_POS);
    sm_motor_toggle(MOTOR_STATE_RUN);
}
/**
* @brief  电机速度控制
* @attention 
*/
void motor_speed_ctrl(motor_id_t id, float speed)
{
    motor_t *motor = NULL;
    
    switch(id)
    {
        case MOTOR_1: motor = &motor_data.m1; break;
        case MOTOR_2: motor = &motor_data.m2; break;
        case MOTOR_3: motor = &motor_data.m3; break;
        case MOTOR_4: motor = &motor_data.m4; break;
        case MOTOR_5: motor = &motor_data.m5; break;
        default: break;
    }
    motor->cmd_speed = speed;
    motor_write_mode(MOTOR_SPEED);
    sm_motor_toggle(MOTOR_STATE_RUN);
}
/**
* @brief  电机运动方向控制
* @attention 
*/
void motor_dir(motor_id_t id, uint8_t cmd)
{
    switch(id)
    {
        case MOTOR_1: M1_DIR(cmd); break;
        case MOTOR_2: M2_DIR(cmd); break;
        case MOTOR_3: M3_DIR(cmd); break;
        case MOTOR_4: M4_DIR(cmd); break;
        case MOTOR_5: M5_DIR(cmd); break;
        default: break;
    }
}
/**
* @brief  电机使能控制
* @attention 
*/
void motor_en(uint8_t cmd)
{
    M1_EN(cmd);
    M2_EN(cmd);
    M3_EN(cmd);
    M4_EN(cmd);
    M5_EN(cmd);
}
///**
//* @brief  电机速度转换为脉冲频率
//* @attention 
//*/
//static float motor_speed_to_frq(float speed)
//{
//    return MOTOR_MICROSTEP * 200.0f * speed;
//}
/**
* @brief  速度渐变
* @attention 
*/
static void motor_speed_change(motor_t *motor)
{
    
}
#ifdef MOTOR_TIM_CTRL
/**
* @brief  速度上限控制,用于电机运动同步
* @attention 
*/
static void motor_speed_limit(void)
{
    float pos_max = 0;
    float pos[4] = {0};
    float tim_max = 0;
    
    if((motor_data.m1.cmd_pos_last != motor_data.m1.cmd_pos)
        || (motor_data.m2.cmd_pos_last != motor_data.m2.cmd_pos)
        || (motor_data.m3.cmd_pos_last != motor_data.m3.cmd_pos)
        || (motor_data.m4.cmd_pos_last != motor_data.m4.cmd_pos))
    {
        motor_data.m1.cmd_pos_last = motor_data.m1.cmd_pos;
        motor_data.m2.cmd_pos_last = motor_data.m2.cmd_pos;
        motor_data.m3.cmd_pos_last = motor_data.m3.cmd_pos;
        motor_data.m4.cmd_pos_last = motor_data.m4.cmd_pos;
        
        motor_data.m1.cmd_speed_limit = 0;
        motor_data.m2.cmd_speed_limit = 0;
        motor_data.m3.cmd_speed_limit = 0;
        motor_data.m4.cmd_speed_limit = 0;

        pos[0] = fabs(motor_data.m1.cmd_pos - motor_data.m1.pos);
        pos[1] = fabs(motor_data.m2.cmd_pos - motor_data.m2.pos);
        pos[2] = fabs(motor_data.m3.cmd_pos - motor_data.m3.pos);
        pos[3] = fabs(motor_data.m4.cmd_pos - motor_data.m4.pos);
        if(pos[0] < pos[1])
        {
            pos_max = pos[1];
        }
        else
        {
            pos_max = pos[0];
        }
        if(pos_max < pos[2])
        {
            pos_max = pos[2];
        }
        if(pos_max < pos[3])
        {
            pos_max = pos[3];
        }
        tim_max = pos_max / motor_data.m1.pid->out_limit;  //s
        if(tim_max == 0)
        {
            return;
        }
        motor_data.m1.cmd_speed_limit = pos[0] / tim_max;
        motor_data.m2.cmd_speed_limit = pos[1] / tim_max;
        motor_data.m3.cmd_speed_limit = pos[2] / tim_max;
        motor_data.m4.cmd_speed_limit = pos[3] / tim_max;
    }
}
#endif
/**
* @brief  电机速度转换为脉冲频率
* @attention 
*/
static void motor_set_speed(motor_t *motor)
{
    if(motor->state_l == MOTOR_LOCKED)
    {
        motor->cmd_speed = 0;
    }
#ifdef MOTOR_TIM_CTRL
    /* tim ctrl */
    if((fabs(motor->cmd_speed) > motor->cmd_speed_limit) && (motor->cmd_mode == MOTOR_POS))
    {
        if(motor->cmd_speed < 0)
        {
            motor->cmd_speed = -motor->cmd_speed_limit;
        }
        else
        {
            motor->cmd_speed = motor->cmd_speed_limit;
        }
    }
#endif
    /* state */
    if(motor->cmd_speed != 0)
    {
        motor->state_r = MOTOR_RUN;
    }
    /* dir */
    if(motor->cmd_speed < 0)
    {
        motor->cmd_frq = -motor->cmd_speed * MOTOR_MICROSTEP * 200.0f;
        motor_dir(motor->id, 0);
    }
    else
    {
        motor->cmd_frq = motor->cmd_speed * MOTOR_MICROSTEP * 200.0f;
        motor_dir(motor->id, 1);
    }
    /* frq */
    if(motor->cmd_speed_last != motor->cmd_speed)
    {
        motor->set_frq(motor->cmd_frq);
        motor->cmd_speed_last = motor->cmd_speed;
    }
    /* stop */
    if(motor->cmd_frq == 0)
    {
        motor->cnt_stop++;
    }
    else
    {
        motor->cnt_stop = 0;
    }
    if(motor->cnt_stop > 10)
    {
        motor->cnt_stop = 10;
        motor->state_r = MOTOR_STOP;
    }
}
/**
* @brief  电机位置pid控制
* @attention 
*/
static void motor_pos_pid(motor_t *motor)
{
    /* pid */
    motor->cmd_speed = pid_control(motor->pid, motor->cmd_pos, motor->pos);
    /* speed */
    motor_set_speed(motor);
}
/**
* @brief     电机注册
* @param[io] p_new:电机描述符
*/
static void motor_regist(motor_t *p_new, uint8_t i)
{
    if(p_new == NULL)
    {
        return;
    }
    p_new->next = motor_head.next;
    motor_head.next = p_new;
    
    p_new->id = (motor_id_t)i;
    p_new->pid = (&motor_pid.m1 + i);
    p_new->pid_pos = motor_pos_pid;
    p_new->init();
}
/**
* @brief  电机写控制模式
* @attention 
*/
void motor_write_mode(motor_cmd_mode_t mode)
{
    motor_t *head = &motor_head;
    
    while(head->next != NULL)
    {
        head->next->cmd_mode = mode;
        head = head->next;
    }
}
/**
* @brief  电机停止
* @attention 
*/
void motor_stop(motor_id_t id)
{
    motor_t *motor = NULL;
    
    switch(id)
    {
        case MOTOR_1: motor = &motor_data.m1; break;
        case MOTOR_2: motor = &motor_data.m2; break;
        case MOTOR_3: motor = &motor_data.m3; break;
        case MOTOR_4: motor = &motor_data.m4; break;
        case MOTOR_5: motor = &motor_data.m5; break;
        default: break;
    }
    motor->cmd_speed = 0;
    motor->cmd_speed_last = 0;
    motor->cmd_frq = 0;
    motor->set_frq(0);
    pid_clear(motor->pid);
    motor->cnt_stop = 0;
    motor->cnt_lock = 0;
    if(motor->state_r == MOTOR_RUN)
    {
        motor->state_r = MOTOR_STOP;
    }
}
/**
* @brief  电机停止
* @attention 
*/
void motor_stop_all(void)
{
    motor_t *head = &motor_head;
    
    while(head->next != NULL)
    {
        head->next->cmd_speed = 0;
        head->next->cmd_speed_last = 0;
        head->next->cmd_frq = 0;
        head->next->set_frq(0);
        pid_clear(head->next->pid);
        head->next->cnt_stop = 0;
        head->next->cnt_lock = 0;
        if(head->next->state_r == MOTOR_RUN)
        {
            head->next->state_r = MOTOR_STOP;
        }
        head = head->next;
    }
}
/**
* @brief  电机位置初始化
* @attention 
*/
void motor_init_pos(void)
{
    encoder_clear();
    motor_data.m1.pos_init = out_filter_data.pot1;
    motor_data.m2.pos_init = out_filter_data.pot2;
    motor_data.m3.pos_init = out_filter_data.pot3;
    motor_data.m4.pos_init = out_filter_data.pot4;
    motor_data.m1.pos = motor_data.m1.pos_init;
    motor_data.m2.pos = motor_data.m2.pos_init;
    motor_data.m3.pos = motor_data.m3.pos_init;
    motor_data.m4.pos = motor_data.m4.pos_init;
    /* 避免开机后走不到位置0 */
    motor_data.m1.cmd_pos_last = motor_data.m1.pos_init;
    motor_data.m2.cmd_pos_last = motor_data.m2.pos_init;
    motor_data.m3.cmd_pos_last = motor_data.m3.pos_init;
    motor_data.m4.cmd_pos_last = motor_data.m4.pos_init;
}
/**
* @brief  电机模块初始化
* @attention 
*/
void motor_init(void)
{
    uint8_t size = sizeof(motor_data) / sizeof(motor_data.m1);
    uint8_t i = 0;

    motor_data.m1.init = motor1_init;
    motor_data.m2.init = motor2_init;
    motor_data.m3.init = motor3_init;
    motor_data.m4.init = motor4_init;
    motor_data.m5.init = motor5_init;
    motor_data.m1.set_frq = motor1_step_frq;
    motor_data.m2.set_frq = motor2_step_frq;
    motor_data.m3.set_frq = motor3_step_frq;
    motor_data.m4.set_frq = motor4_step_frq;
    motor_data.m5.set_frq = motor5_step_frq;
    
    while(size--)
    {
        motor_regist((&motor_data.m1)+i, i);
        i++;
    }
    /* 先使能,稳定后读取开机位置 */
    motor_en(1);
    vTaskDelay(1000);
    motor_init_pos();
    ctrl_led(CTRL_GREEN);
}
/**
* @brief  电机模块运行
* @attention 
*/
void motor_run(void)
{
    motor_t *head = &motor_head;
#ifdef MOTOR_TIM_CTRL
    motor_speed_limit();
#endif
    while(head->next != NULL)
    {
        if(head->next->cmd_mode == MOTOR_SPEED)
        {
            motor_set_speed(head->next);
        }
        else
        {
            head->next->pid_pos(head->next);
        }
        head = head->next;
    }
}
/**
* @brief  电机读取是否有堵转
* @attention 
*/
uint8_t motor_read_lock(void)
{
    motor_t *head = &motor_head;
    
    while(head->next != NULL)
    {
        if(head->next->state_l == MOTOR_LOCKED)
        {
            return 1;
        }
        head = head->next;
    }
    return 0;
}
/**
* @brief  电机清堵转计数
* @attention 
*/
static void motor_clear_lock_cnt(void)
{
    motor_t *head = &motor_head;
    
    while(head->next != NULL)
    {
        head->next->cnt_lock = 0;
        head = head->next;
    }
}
/**
* @brief  电机清堵转
* @attention 
*/
void motor_clear_lock(void)
{
    motor_t *head = &motor_head;
    
    while(head->next != NULL)
    {
        head->next->cnt_lock = 0;
        head->next->state_r = MOTOR_STOP;
        head->next->state_l = MOTOR_UNLOCKED;
        head = head->next;
    }
}
/**
* @brief  电机堵转监测
* @attention 
*/
static void motor_monitor_lock(motor_t *motor)
{
    if(motor->cmd_speed > 0)
    {
        if(motor->speed < 0)
        {
            motor->cnt_lock++;
        }
    }
    else if(motor->cmd_speed < 0)
    {
        if(motor->speed > 0)
        {
            motor->cnt_lock++;
        }
    }
    if(motor->cnt_lock > 100)
    {
        motor->cnt_lock = 100;
        motor->state_l = MOTOR_LOCKED;
        motor_stop(motor->id);
    }
}
/**
* @brief  电机速度监测
* @attention 
*/
void motor_monitor_speed(motor_t *motor)
{
    float speed  = 0;
    float f_per  = 0.9f;  //滤波强度
    
    speed = (motor->pos - motor->pos_last) / TAST_TIMS_DEV * DATA_RPMS_TO_RPS;  //r/s
    motor->speed = motor->speed * f_per + speed * (1 - f_per);
    if((sm_motor_state() == MOTOR_STATE_RUN) && g_lock_open)
    {
        motor_monitor_lock(motor);
    }
    motor->pos_last = motor->pos;
}
/**
* @brief  电机监测
* @attention 
*/
void motor_monitor(void)
{
    motor_t *head = &motor_head;
    static uint16_t tim_ms = 0;

    motor_data.m1.pos = out_data.encoder1 + motor_data.m1.pos_init;
    motor_data.m2.pos = out_data.encoder2 + motor_data.m2.pos_init;
    motor_data.m3.pos = out_data.encoder3 + motor_data.m3.pos_init;
    motor_data.m4.pos = out_data.encoder4 + motor_data.m4.pos_init;
    motor_data.m5.pos = out_data.encoder5 + motor_data.m5.pos_init;

    while(head->next != NULL)
    {
        motor_monitor_speed(head->next);
        head = head->next;
    }
    
    tim_ms++;
    if(tim_ms > 1500)
    {
        tim_ms = 0;
        motor_clear_lock_cnt();
    }
}
