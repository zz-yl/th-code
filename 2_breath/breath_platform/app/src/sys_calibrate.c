/**@file   sys_calibrate.c
* @brief   У׼ģ��
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "sys_calibrate.h"

#include "interp_table.h"
#include "device.h"
#include "motor.h"
#include "comm.h"
#include "valve.h"

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
    comm_cmd.flow = 0;
}
/**
* @brief  ����������У׼
* @attention 
*/
void cal_flow(void)
{
    switch(cal_ctrl.state)
    {
        case CAL_STATE_FREE:
            cal_ctrl.state = CAL_STATE_START;
            break;
        case CAL_STATE_START:
        {
            cal_reset_dev();
            cal_tim_clear();

            cal_ctrl.start = 70;
            cal_ctrl.up = 1;
            cal_ctrl.end = 750;
            cal_ctrl.tim_c = 60000;
            cal_ctrl.tim_pre = 0.5;
            
            cal_ctrl.data = cal_ctrl.start;
            comm_cmd.type = COMM_FLOW;
            comm_cmd.flow = cal_ctrl.data;
            
            cal_ctrl.state = CAL_STATE_STEP1;
            break;
        }
        case CAL_STATE_STEP1:
            /* ѹ���¼ֵ���� */
            if(cal_ctrl.cur_tim <= (cal_ctrl.tim_c * cal_ctrl.tim_pre))
            {
                dev_data.ff_flag = 1;
            }
//            /* ��ͣ���ֿ��� */
//            if(cal_ctrl.cur_tim == (cal_ctrl.tim_c * 0.5))
//            {
//                comm_cmd.type = COMM_SPEED;
//            }
            if(cal_ctrl.cur_tim >= cal_ctrl.tim_c)
            {
                cal_tim_clear();
                /* ��¼���� */
                cal_ctrl.buf1[cal_ctrl.count] = dev_data.press_lff;
                cal_ctrl.buf2[cal_ctrl.count] = dev_data.flow_real_lff;
                cal_ctrl.buf3[cal_ctrl.count] = dev_data.temp_ff;
                cal_ctrl.count++;
                if(cal_ctrl.count >= CAL_BUF_MAX)
                {
                    cal_ctrl.state = CAL_STATE_RECORD;
                }
                
                /* �仯��ǰ��������ʱ�� */
                if(cal_ctrl.data > 90)
                {
                    cal_ctrl.tim_c = 60000;
                    cal_ctrl.tim_pre = 0.15;
                }
                /* �仯����������� */
                if(cal_ctrl.data >= 400)
                {
                    cal_ctrl.up = 2;
                }
                /* ���Ƶ�� */
                cal_ctrl.data += cal_ctrl.up;
                comm_cmd.flow = cal_ctrl.data;
                comm_cmd.type = COMM_FLOW;
                /* �������� */
                if(cal_ctrl.data > cal_ctrl.end)
                {
                    cal_ctrl.state = CAL_STATE_RECORD;
                }
            }
            break;
        case CAL_STATE_RECORD:
            cal_reset_dev();
            cal_tim_clear();
            cal_ctrl.state = CAL_STATE_FREE;
            cal_ctrl.cmd = CAL_CMD_CANCEL;
            break;
        default: break;
    }
}
/**
* @brief  ����������У׼B-С��������������
* @attention 
*/
void cal_flow_b(void)
{
    switch(cal_ctrl.state)
    {
        case CAL_STATE_FREE:
            cal_ctrl.state = CAL_STATE_START;
            break;
        case CAL_STATE_START:
        {
            cal_reset_dev();
            cal_tim_clear();

            cal_ctrl.start = 0.5;
            cal_ctrl.up = 0.5;
            cal_ctrl.end = 100;
            cal_ctrl.tim_c = 60000;
            cal_ctrl.tim_pre = 0.5;
            
            cal_ctrl.data = cal_ctrl.start;
            comm_cmd.type = COMM_FLOW_VALVE;
            comm_cmd.flow = cal_ctrl.data;
            
            cal_ctrl.state = CAL_STATE_STEP1;
            break;
        }
        case CAL_STATE_STEP1:
            /* ѹ���¼ֵ���� */
            if(cal_ctrl.cur_tim <= (cal_ctrl.tim_c * cal_ctrl.tim_pre))
            {
                dev_data.ff_flag = 1;
            }
//            /* ��ͣ���������� */
//            if(cal_ctrl.cur_tim == (cal_ctrl.tim_c * 0.5))
//            {
//                comm_cmd.type = COMM_SPEED;
//            }
            if(cal_ctrl.cur_tim >= cal_ctrl.tim_c)
            {
                cal_tim_clear();
                /* ��¼���� */
                cal_ctrl.buf1[cal_ctrl.count] = dev_data.press_lff;
                cal_ctrl.buf2[cal_ctrl.count] = dev_data.flow_real_lff;
                cal_ctrl.buf3[cal_ctrl.count] = dev_data.temp_ff;
                cal_ctrl.count++;
                if(cal_ctrl.count >= CAL_BUF_MAX)
                {
                    cal_ctrl.state = CAL_STATE_RECORD;
                }
                
                /* �仯��ǰ��������ʱ�� */
                if(cal_ctrl.data >= 5)
                {
                    cal_ctrl.tim_c = 50000;
                    cal_ctrl.tim_pre = 0.15;
                    cal_ctrl.up = 1;
                }
                /* ��������ת�� */
                if(cal_ctrl.data > 50)
                {
                    motor_data.cmd = 10000;
                }
                /* ���Ʊ����� */
                cal_ctrl.data += cal_ctrl.up;
                comm_cmd.flow = cal_ctrl.data;
//                comm_cmd.type = COMM_FLOW_VALVE;
                /* �������� */
                if(cal_ctrl.data > cal_ctrl.end)
                {
                    cal_ctrl.state = CAL_STATE_RECORD;
                }
            }
            break;
        case CAL_STATE_RECORD:
            cal_reset_dev();
            cal_tim_clear();
            cal_ctrl.state = CAL_STATE_FREE;
            cal_ctrl.cmd = CAL_CMD_CANCEL;
            break;
        default: break;
    }
}
/**
* @brief  sfm3300����������У׼
* @attention 
*/
void cal_flow_c(void)
{
    switch(cal_ctrl.state)
    {
        case CAL_STATE_FREE:
            cal_ctrl.state = CAL_STATE_START;
            break;
        case CAL_STATE_START:
        {
            cal_reset_dev();
            cal_tim_clear();

            cal_ctrl.start = 30;
            cal_ctrl.up = 1;
            cal_ctrl.end = 250;
            cal_ctrl.tim_c = 30000;
            cal_ctrl.tim_pre = 0.5;
            
            cal_ctrl.data = cal_ctrl.start;
            comm_cmd.type = COMM_FLOW;
            comm_cmd.flow = cal_ctrl.data;
            
            cal_ctrl.state = CAL_STATE_STEP1;
            break;
        }
        case CAL_STATE_STEP1:
            /* ѹ���¼ֵ���� */
            if(cal_ctrl.cur_tim <= (cal_ctrl.tim_c * cal_ctrl.tim_pre))
            {
                dev_data.ff_flag = 1;
            }
            /* ��ͣ���ֿ��� */
            if(cal_ctrl.cur_tim == (cal_ctrl.tim_c * 0.6))
            {
                comm_cmd.type = COMM_SPEED;
            }
            if(cal_ctrl.cur_tim >= cal_ctrl.tim_c)
            {
                cal_tim_clear();
                /* ��¼���� */
                cal_ctrl.buf1[cal_ctrl.count] = dev_data.flow_t1_lff;
                cal_ctrl.buf2[cal_ctrl.count] = dev_data.flow_t2_lff;
//                cal_ctrl.buf3[cal_ctrl.count] = dev_data.flow_t3_lff;
//                cal_ctrl.buf4[cal_ctrl.count] = dev_data.flow_t4_lff;
                cal_ctrl.count++;
                if(cal_ctrl.count >= CAL_BUF_MAX)
                {
                    cal_ctrl.state = CAL_STATE_RECORD;
                }
                
                /* �仯��ǰ��������ʱ�� */
                if(cal_ctrl.data > 50)
                {
                    cal_ctrl.tim_c = 30000;
                    cal_ctrl.tim_pre = 0.1;
                }
                /* ���Ƶ�� */
                cal_ctrl.data += cal_ctrl.up;
                comm_cmd.flow = cal_ctrl.data;
                comm_cmd.type = COMM_FLOW;
                /* �������� */
                if(cal_ctrl.data > cal_ctrl.end)
                {
                    cal_ctrl.state = CAL_STATE_RECORD;
                }
            }
            break;
        case CAL_STATE_RECORD:
            cal_reset_dev();
            cal_tim_clear();
            cal_ctrl.state = CAL_STATE_FREE;
            cal_ctrl.cmd = CAL_CMD_CANCEL;
            break;
        default: break;
    }
}
/**
* @brief  У׼����
* @attention 
*/
void cal_run(void)
{
    switch(cal_ctrl.cmd)
    {
        case CAL_CMD_FLOW: cal_flow(); break;
        case CAL_CMD_FLOW_B: cal_flow_b(); break;
        case CAL_CMD_FLOW_C: cal_flow_c(); break;
        default: cal_ctrl.state = CAL_STATE_FREE; break;
    }
    cal_tim_add();
}

