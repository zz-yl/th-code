/**@file   sys_calibrate.c
* @brief   校准模块
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "sys_calibrate.h"

#include "interp_table.h"
#include "control.h"
#include "device_io.h"
#include "motor.h"
#include "motor_state_machine.h"
#include "thp_cfg.h"
#include "memory.h"
#include "sys_state_machine.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

calibrate_ctrl_t cal_ctrl;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

static void cal_tim_add(void)
{
    cal_ctrl.cur_tim++;
}
static void cal_tim_clear(void)
{
    cal_ctrl.cur_tim = 0;
}
static void cal_reset_dev(void)
{
    
}
/**
* @brief  电位器校准
* @attention 
*/
void cal_pot(void)
{
    static uint16_t i = 0;
    
    switch(cal_ctrl.state)
    {
        case CAL_STATE_FREE:
            cal_ctrl.state = CAL_STATE_START;
            break;
        case CAL_STATE_START:  //打开抱闸
        {
            cal_reset_dev();
            cal_tim_clear();

            motor_data.m1.pos = 0.0f;
            motor_data.m2.pos = 0.0f;
            motor_data.m3.pos = 0.0f;
            motor_data.m4.pos = 0.0f;
            motor_data.m5.pos = 0.0f;
            IO_BREAK(0);
            motor_en(1);
            
            cal_ctrl.state = CAL_STATE_STEP1;
            break;
        }
        case CAL_STATE_STEP1:  //速度模式向外走到头
            if(cal_ctrl.cur_tim >= 1000)
            {
                cal_tim_clear();
                motor_data.m1.cmd_speed = 5.0f;
                motor_data.m2.cmd_speed = 5.0f;
                motor_data.m3.cmd_speed = 5.0f;
                motor_data.m4.cmd_speed = 5.0f;
                
                motor_write_mode(MOTOR_SPEED);
                sm_motor_toggle(MOTOR_STATE_RUN);

                cal_ctrl.state = CAL_STATE_STEP2;
            }
            break;
        case CAL_STATE_STEP2:  //清堵转,清编码器数据
            if((cal_ctrl.cur_tim >= 30000) || (sm_motor_state() == MOTOR_STATE_STOP))
            {
                cal_tim_clear();
                motor_clear_lock();
                encoder_clear();

                cal_ctrl.state = CAL_STATE_STEP3;
            }
            break;
        case CAL_STATE_STEP3:  //速度模式向内走到头
            if(cal_ctrl.cur_tim >= 10)
            {
                cal_tim_clear();
                motor_data.m1.cmd_speed = -5.0f;
                motor_data.m2.cmd_speed = -5.0f;
                motor_data.m3.cmd_speed = -5.0f;
                motor_data.m4.cmd_speed = -5.0f;
                
                motor_write_mode(MOTOR_SPEED);
                sm_motor_toggle(MOTOR_STATE_RUN);

                cal_ctrl.state = CAL_STATE_STEP4;
            }
            break;
        case CAL_STATE_STEP4:  //清堵转,清编码器数据，根据长度判断装置类型，并写E2
            if((cal_ctrl.cur_tim >= 30000) || (sm_motor_state() == MOTOR_STATE_STOP))
            {
                cal_tim_clear();
                motor_clear_lock();
                if(fabs(out_data.encoder1) > length_max[SYS_LENGTH_150])
                {
                    device_type = SYS_LENGTH_150;
                }
                else if(fabs(out_data.encoder1) > length_max[SYS_LENGTH_100])
                {
                    device_type = SYS_LENGTH_100;
                }
                else
                {
                    device_type = SYS_LENGTH_60;
                }
                mem_buf[0] = device_type;
                mem_e2prom_write(mem_buf, 1, MEM_ADDR_TYPE);
                encoder_clear();

                cal_ctrl.state = CAL_STATE_STEP5;
            }
            break;
        case CAL_STATE_STEP5:  //清编码器数据，清电机初始位置，向前走0.3mm作为预留
            if(cal_ctrl.cur_tim >= 100)
            {
                cal_tim_clear();
                
                encoder_clear();
                motor_data.m1.pos_init = 0;
                motor_data.m2.pos_init = 0;
                motor_data.m3.pos_init = 0;
                motor_data.m4.pos_init = 0;
                
                motor_data.m1.cmd_pos = 0.3f;
                motor_data.m2.cmd_pos = 0.3f;
                motor_data.m3.cmd_pos = 0.3f;
                motor_data.m4.cmd_pos = 0.3f;
                
                motor_write_mode(MOTOR_POS);
                sm_motor_toggle(MOTOR_STATE_RUN);

                cal_ctrl.state = CAL_STATE_STEP6;
            }
            break;
        case CAL_STATE_STEP6:  //寻找电位器有效位置
            if(cal_ctrl.cur_tim >= 800)
            {
                cal_tim_clear();
                motor_clear_lock();
//                if(in_filter_data.pot1 > 0x790000)
                if((in_filter_data.pot1 > 0x790000) || (in_filter_data.pot2 > 0x790000)
                    || (in_filter_data.pot3 > 0x790000) || (in_filter_data.pot4 > 0x790000))
                {
                    if(in_filter_data.pot1 > 0x790000)
                    {
                        motor_data.m1.cmd_pos += 0.1f;
                    }
                    if(in_filter_data.pot2 > 0x790000)
                    {
                        motor_data.m2.cmd_pos += 0.1f;
                    }
                    if(in_filter_data.pot3 > 0x790000)
                    {
                        motor_data.m3.cmd_pos += 0.1f;
                    }
                    if(in_filter_data.pot4 > 0x790000)
                    {
                        motor_data.m4.cmd_pos += 0.1f;
                    }
                    motor_write_mode(MOTOR_POS);
                    sm_motor_toggle(MOTOR_STATE_RUN);
                }
                else
                {
                    i = 0;
                    cal_ctrl.state = CAL_STATE_STEP7;
                }
            }
            break;
        case CAL_STATE_STEP7:  //前30个点0.1mm
            if(cal_ctrl.cur_tim >= 1500)
            {
                cal_tim_clear();
                device_list.x_pos1[i] = in_filter_data.pot1;
                device_list.x_pos2[i] = in_filter_data.pot2;
                device_list.x_pos3[i] = in_filter_data.pot3;
                device_list.x_pos4[i] = in_filter_data.pot4;
                
                motor_data.m1.cmd_pos += 0.1f;
                motor_data.m2.cmd_pos += 0.1f;
                motor_data.m3.cmd_pos += 0.1f;
                motor_data.m4.cmd_pos += 0.1f;
                
                motor_write_mode(MOTOR_POS);
                sm_motor_toggle(MOTOR_STATE_RUN);
                i++;
                if(i >= 30)
                {
                    cal_ctrl.state = CAL_STATE_STEP8;
                }
            }
            break;
        case CAL_STATE_STEP8:  //中间点1mm
            if(cal_ctrl.cur_tim >= 1500)
            {
                cal_tim_clear();
                device_list.x_pos1[i] = in_filter_data.pot1;
                device_list.x_pos2[i] = in_filter_data.pot2;
                device_list.x_pos3[i] = in_filter_data.pot3;
                device_list.x_pos4[i] = in_filter_data.pot4;
                
                motor_data.m1.cmd_pos += 1.0f;
                motor_data.m2.cmd_pos += 1.0f;
                motor_data.m3.cmd_pos += 1.0f;
                motor_data.m4.cmd_pos += 1.0f;
                
                motor_write_mode(MOTOR_POS);
                sm_motor_toggle(MOTOR_STATE_RUN);
                i++;
                if(i >= (table_max[device_type] - 31))
                {
                    cal_ctrl.state = CAL_STATE_STEP9;
                }
            }
            break;
        case CAL_STATE_STEP9:  //后30个点0.1mm
            if(cal_ctrl.cur_tim >= 1500)
            {
                cal_tim_clear();
                device_list.x_pos1[i] = in_filter_data.pot1;
                device_list.x_pos2[i] = in_filter_data.pot2;
                device_list.x_pos3[i] = in_filter_data.pot3;
                device_list.x_pos4[i] = in_filter_data.pot4;
                
                motor_data.m1.cmd_pos += 0.1f;
                motor_data.m2.cmd_pos += 0.1f;
                motor_data.m3.cmd_pos += 0.1f;
                motor_data.m4.cmd_pos += 0.1f;
                
                motor_write_mode(MOTOR_POS);
                sm_motor_toggle(MOTOR_STATE_RUN);
                i++;
                if(i >= (table_max[device_type]))
                {
                    cal_ctrl.state = CAL_STATE_STEP10;
                }
            }
            break;
        case CAL_STATE_STEP10:  //记录数据,关闭抱闸,编码器清零,初始化电机位置
            if(cal_ctrl.cur_tim >= 1500)
            {
                cal_tim_clear();
                sm_motor_toggle(MOTOR_STATE_STOP);
                IO_BREAK(1);
                
                for(i=0; i<150; i++)
                {
                    mem_buf_pot[i]     = (uint32_t)device_list.x_pos1[i];
                    mem_buf_pot[150+i] = (uint32_t)device_list.x_pos2[i];
                    mem_buf_pot[300+i] = (uint32_t)device_list.x_pos3[i];
                    mem_buf_pot[450+i] = (uint32_t)device_list.x_pos4[i];
                }
                motor_init_pos();
                cal_ctrl.state = CAL_STATE_RECORD;
            }
            break;
        case CAL_STATE_RECORD:
            mem_e2prom_write((uint8_t *)mem_buf_pot, 2400, MEM_ADDR_POT);
            motor_en(0);
            cal_ctrl.state = CAL_STATE_FREE;
            cal_ctrl.cmd = CAL_CMD_CANCEL;
            break;
        default: break;
    }
}

/**
* @brief  校准运行
* @attention 
*/
void cal_run(void)
{
    switch(cal_ctrl.cmd)
    {
        case CAL_CMD_POT: cal_pot(); break;
        default: sm_sys_toggle(SYS_STANDBY); cal_ctrl.state = CAL_STATE_FREE; break;
    }
    cal_tim_add();
}

