/**@file   test.c
* @brief   测试
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "test.h"

#include "motor_comm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor_state_machine.h"
#include "memory.h"
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

test_ctrl_t test_ctrl = 
{
    .data1 = 20,
    .arr = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
    .arr_len = 9,
};

uint8_t print_buf[100];
uint16_t print_cnt = 0;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void test_print_fill(float fdata)
{
    union
    {
        uint8_t data[4];
        float   fdata;
    }f_to_u8;
    
    f_to_u8.fdata = fdata;
    print_buf[print_cnt++] = f_to_u8.data[0];
    print_buf[print_cnt++] = f_to_u8.data[1];
    print_buf[print_cnt++] = f_to_u8.data[2];
    print_buf[print_cnt++] = f_to_u8.data[3];
}
void test_print(void)
{
    print_buf[print_cnt++] = 0x00;
    print_buf[print_cnt++] = 0x00;
    print_buf[print_cnt++] = 0x80;
    print_buf[print_cnt++] = 0x7F;
    uart8_send(print_buf, print_cnt);
    print_cnt = 0;
}
static void test_xyz_40(void)
{
    static uint8_t i = 0;
    
    float pos[9][5] = 
    {
        {20, 20, 20, 20},
        {12.356, 0.627, 12.356, 0.627},
        {39.757, 30.512, 39.757, 30.512},
        {30.512, 39.757, 30.512, 39.757},
        {0.627, 12.356, 0.627, 12.356},
    };
    
    if(sm_motor_state() == MOTOR_STATE_STOP)
    {
        motor_ctrl_pos(MOTOR_1, pos[i][0]);
        motor_ctrl_pos(MOTOR_2, pos[i][1]);
        motor_ctrl_pos(MOTOR_3, pos[i][2]);
        motor_ctrl_pos(MOTOR_4, pos[i][3]);
        i++;
        if(i > 4) //4\8
        {
            i = 1;
            test_ctrl.cnt++;
        }
    }
    else
    {
        return;
    }
}

/**
* @brief  测试功能运行
* @attention 
*/
void test_run(void)
{
    static uint16_t tim = 0;

    switch(test_ctrl.id)
    {
        case 1: motor_send_pos_all(); break;
        case 2: motor_get_pos_all(); break;
        case 3: motor_fill_msg(test_ctrl.ctrl, 0xF28C, test_ctrl.arr, test_ctrl.arr_len); break;
        case 11: test_xyz_40(); break;
//        case 12: uart4_send(test_ctrl.arr, test_ctrl.arr_len); break;
//        case 13: uart5_send(test_ctrl.arr, test_ctrl.arr_len); break;
//        case 14: uart6_send(test_ctrl.arr, test_ctrl.arr_len); break;
//        case 15: uart7_send(test_ctrl.arr, test_ctrl.arr_len); break;
        case 16: uart8_send(test_ctrl.arr, test_ctrl.arr_len); break;
        case 31: i2c3_read(test_ctrl.arr, test_ctrl.arr_len, test_ctrl.ctrl); break;
        case 32: i2c3_write(test_ctrl.arr, test_ctrl.arr_len, test_ctrl.ctrl); break;
        case 33: mem_e2prom_read(test_ctrl.arr, test_ctrl.arr_len, (uint16_t)test_ctrl.ctrl); break;
        case 34: mem_e2prom_write(test_ctrl.arr, test_ctrl.arr_len, (uint16_t)test_ctrl.ctrl); break;
        case 91: IO_LED_R(test_ctrl.ctrl);  break;
        case 92: IO_LED_G(test_ctrl.ctrl);  break;
        case 93: IO_LED_B(test_ctrl.ctrl);  break;
        case 94: IO_POWER(test_ctrl.ctrl);  break;
        case 95: IO_BREAK(test_ctrl.ctrl);  break;
        case 97: IO_E2_WP(test_ctrl.ctrl);  break;
        default: break;
    }
    if(test_ctrl.state == 0)
    {
        test_ctrl.id = 0;
    }
#ifdef TEST_PRINT
    tim++;
    if(tim > 500)
    {
        tim = 0;
//        vTaskGetRunTimeStats((char *)&test_ctrl.cpu_state1);
//        printf("sta:\n%s\n", test_ctrl.cpu_state1);
//        vTaskList((char *)&test_ctrl.cpu_state2);
//        printf("list:\n%s\n", test_ctrl.cpu_state2);
        
        printf("run times:%d; m1:%0.4f; m2:%0.4f; m3:%0.4f; m4:%0.4f; state:%d,%d,%d,%d; lock_r:%d,%d,%d,%d; time:%d\r\n",
               test_ctrl.cnt, motor_data.m1.pos, motor_data.m2.pos, motor_data.m3.pos, motor_data.m4.pos,
               motor_data.m1.state_l, motor_data.m2.state_l, motor_data.m3.state_l, motor_data.m4.state_l, 
               motor_data.m1.cnt_recover, motor_data.m2.cnt_recover, motor_data.m3.cnt_recover, motor_data.m4.cnt_recover, sys_timer);
    }
    
//    test_print_fill(motor_data.m1.pos);
//    test_print_fill(motor_data.m1.speed);
//    test_print_fill(motor_data.m1.cmd_speed);
//    test_print_fill(vol_data.ex0);
    
//    test_print_fill(out_data.encoder1);
//    test_print_fill(out_data.encoder2);
//    test_print_fill(out_data.encoder3);
//    test_print_fill(out_data.encoder4);
//    
//    test_print_fill(in_filter_data.pot1);
    
//    test_print_fill(motor_data.m1.pos);
//    test_print_fill(motor_data.m2.pos);
//    test_print_fill(motor_data.m3.pos);
//    test_print_fill(motor_data.m4.pos);
    
//    test_print_fill(motor_data.m4.cmd_speed);
    
//    test_print();

#endif
}
