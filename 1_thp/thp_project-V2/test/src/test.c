/**@file   test.c
* @brief   ≤‚ ‘
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "test.h"

#include "FreeRTOS.h"
#include "task.h"
#include "motor.h"
#include "motor_state_machine.h"
#include "device_io.h"
#include "memory.h"
#include "thp_cfg.h"
#include "comm_cmd.h"
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

test_ctrl_t test_ctrl = 
{
    .data1 = 20,
    .arr = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
    .arr_len = 9,
    .sn = "SP01002024010001",
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
static void test_spi2(uint16_t len)
{
    uint16_t i = 0;
    
    for(; i<len; i++)
    {
        spi2_read_write_byte(test_ctrl.arr[i]);
    }
}
static void test_motor_step(void)
{
    static float frq = 0;
    static float frq_last = 0;
    
    if(test_ctrl.cnt == 0)
    {
        frq = 0;
    }
    else
    {
        frq = test_ctrl.ctrl;
        test_ctrl.cnt--;
    }
    if(frq_last != frq)
    {
        frq_last = frq;
        motor4_step_frq(frq);
    }
}
static void test_motor_cycle(void)
{
    static float speed_last = 0;

    if(speed_last != test_ctrl.ctrl)
    {
        speed_last = test_ctrl.ctrl;
//        motor_speed_ctrl(MOTOR_1, test_ctrl.ctrl);
//        motor_speed_ctrl(MOTOR_2, test_ctrl.ctrl);
//        motor_speed_ctrl(MOTOR_3, test_ctrl.ctrl);
        motor_speed_ctrl(MOTOR_4, test_ctrl.ctrl);
        
        return;
    }
    
    if(sm_motor_state() == MOTOR_STATE_STOP)
    {
        test_ctrl.cnt++;
        motor_clear_lock();
        test_ctrl.ctrl = -test_ctrl.ctrl;
        vTaskDelay(500);
    }
}
static void test_motor_all_step(void)
{
    motor_en(1);
    if(test_ctrl.ctrl >= 0)
    {
        motor_dir(MOTOR_1, 1);
        motor_dir(MOTOR_2, 1);
        motor_dir(MOTOR_3, 1);
        motor_dir(MOTOR_4, 1);
    }
    else
    {
        test_ctrl.ctrl = -test_ctrl.ctrl;
        motor_dir(MOTOR_1, 0);
        motor_dir(MOTOR_2, 0);
        motor_dir(MOTOR_3, 0);
        motor_dir(MOTOR_4, 0);
    }
    motor1_step_frq(test_ctrl.ctrl);
    motor2_step_frq(test_ctrl.ctrl);
    motor3_step_frq(test_ctrl.ctrl);
    motor4_step_frq(test_ctrl.ctrl);
}
static void test_xyz_100(void)
{
    static uint8_t i = 0;
    
    float pos[9][5] = 
    {
        {25, 25, 25, 25},
        {25.68, 1.733, 25.68, 1.733},
        {89.332, 71.559, 89.332, 71.559},
        {71.559, 89.332, 71.559, 89.332},
        {1.733, 25.68, 1.733, 25.68},
        {48.555, 48.555, 25.374, 25.374}, //∫Û«„35°„
        {48.555, 48.555, 71.892, 71.892}, //«∞«„35°„
        {45.000, 45.000, 40.100, 52.287}, //◊Û«„35°„
        {45.000, 45.000, 52.287, 40.100}, //”“«„35°„
    };
    
    if(sm_motor_state() == MOTOR_STATE_STOP)
    {
        motor_pos_ctrl(MOTOR_1, pos[i][0]);
        motor_pos_ctrl(MOTOR_2, pos[i][1]);
        motor_pos_ctrl(MOTOR_3, pos[i][2]);
        motor_pos_ctrl(MOTOR_4, pos[i][3]);
        i++;
        if(i > 8) //4\8
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
static void test_xyz_60(void)
{
    static uint8_t i = 0;
    
    float pos[9][5] = 
    {
        {25, 25, 25, 25},
        {12.356, 0.627, 12.356, 0.627},
        {49.757, 40.512, 49.757, 40.512},
        {40.512, 49.757, 40.512, 49.757},
        {0.627, 12.356, 0.627, 12.356},
        {26.006, 26.006, 2.643, 2.643},   //∫Û«„35°„
        {26.006, 26.006, 49.531, 49.531}, //«∞«„35°„
        {22.423, 22.423, 18.682, 29.456}, //◊Û«„35°„
        {22.423, 22.423, 29.456, 18.682}, //”“«„35°„
    };
    
    if(sm_motor_state() == MOTOR_STATE_STOP)
    {
        motor_pos_ctrl(MOTOR_1, pos[i][0]);
        motor_pos_ctrl(MOTOR_2, pos[i][1]);
        motor_pos_ctrl(MOTOR_3, pos[i][2]);
        motor_pos_ctrl(MOTOR_4, pos[i][3]);
        i++;
        if(i > 8) //4\8
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
static void test_xyz_60_B(void)
{
    static uint8_t i = 0;
    
    float pos[9][5] = 
    {
        {0.2, 0.2, 0.2, 0.2},
        {49.8, 49.8, 49.8, 49.8},
    };
    
    if(sm_motor_state() == MOTOR_STATE_STOP)
    {
        motor_pos_ctrl(MOTOR_1, pos[i][0]);
        motor_pos_ctrl(MOTOR_2, pos[i][1]);
        motor_pos_ctrl(MOTOR_3, pos[i][2]);
        motor_pos_ctrl(MOTOR_4, pos[i][3]);
        i++;
        if(i > 1) //4\8
        {
            i = 0;
            test_ctrl.cnt++;
        }
    }
    else
    {
        return;
    }
}

/**
* @brief  ≤‚ ‘π¶ƒ‹‘À––
* @attention 
*/
void test_run(void)
{
    static uint16_t tim = 0;

    switch(test_ctrl.id)
    {
        case 1: spi1_read_write(test_ctrl.arr, test_ctrl.arr_len); break;
        case 2: spi2_read_write(test_ctrl.arr, test_ctrl.arr_len); break;
        case 3: spi3_read_write(test_ctrl.arr, test_ctrl.arr_len); break;
        case 4: spi4_read_write(test_ctrl.arr, test_ctrl.arr_len); break;
        case 5: qspi_read_write(test_ctrl.arr, test_ctrl.arr_len); break;
        case 6: spi6_read_write(test_ctrl.arr, test_ctrl.arr_len); break;
        case 7: test_spi2(test_ctrl.arr_len); break;
        case 8: encoder_call(); break;
        case 9: motor_en(test_ctrl.ctrl); break;
//        case 11: uart3_send(test_ctrl.arr, test_ctrl.arr_len); break;
//        case 12: uart4_send(test_ctrl.arr, test_ctrl.arr_len); break;
//        case 13: uart5_send(test_ctrl.arr, test_ctrl.arr_len); break;
//        case 14: uart6_send(test_ctrl.arr, test_ctrl.arr_len); break;
//        case 15: uart7_send(test_ctrl.arr, test_ctrl.arr_len); break;
        case 16: uart8_send(test_ctrl.arr, test_ctrl.arr_len); break;
        case 17: test_xyz_100(); break;
        case 18: test_xyz_60(); break;
        case 19: test_xyz_60_B(); break;
        case 21: motor1_step_frq(test_ctrl.ctrl);  break;
        case 22: motor2_step_frq(test_ctrl.ctrl);  break;
        case 23: motor3_step_frq(test_ctrl.ctrl);  break;
        case 24: motor4_step_frq(test_ctrl.ctrl);  break;
        case 25: motor5_step_frq(test_ctrl.ctrl);  break;
        case 26: tim_buzzer_frq(test_ctrl.ctrl);   break;
        case 27: test_motor_step(); break;
        case 28: test_motor_cycle(); break;
        case 29: test_motor_all_step(); break;
        case 31: i2c3_read(test_ctrl.arr, test_ctrl.arr_len, test_ctrl.ctrl); break;
        case 32: i2c3_write(test_ctrl.arr, test_ctrl.arr_len, test_ctrl.ctrl); break;
        case 33: mem_e2prom_read(test_ctrl.arr, test_ctrl.arr_len, (uint16_t)test_ctrl.ctrl); break;
        case 34: mem_e2prom_write(test_ctrl.arr, test_ctrl.arr_len, (uint16_t)test_ctrl.ctrl); break;
        case 41: comm_cmd(CMD_SET_SN, test_ctrl.sn); break;
        case 71: M1_DIR(test_ctrl.ctrl);    break;
        case 72: M1_EN(test_ctrl.ctrl);     break;
        case 73: M1_SLEEP(test_ctrl.ctrl);  break;
        case 74: M2_DIR(test_ctrl.ctrl);    break;
        case 75: M2_EN(test_ctrl.ctrl);     break;
        case 76: M2_SLEEP(test_ctrl.ctrl);  break;
        case 77: M3_DIR(test_ctrl.ctrl);    break;
        case 78: M3_EN(test_ctrl.ctrl);     break;
        case 79: M3_SLEEP(test_ctrl.ctrl);  break;
        case 80: M4_DIR(test_ctrl.ctrl);    break;
        case 81: M4_EN(test_ctrl.ctrl);     break;
        case 82: M4_SLEEP(test_ctrl.ctrl);  break;
        case 83: M5_DIR(test_ctrl.ctrl);    break;
        case 84: M5_EN(test_ctrl.ctrl);     break;
        case 85: M5_SLEEP(test_ctrl.ctrl);  break;
        case 91: IO_LED_R(test_ctrl.ctrl);  break;
        case 92: IO_LED_G(test_ctrl.ctrl);  break;
        case 93: IO_LED_B(test_ctrl.ctrl);  break;
        case 94: IO_POWER(test_ctrl.ctrl);  break;
        case 95: IO_BREAK(test_ctrl.ctrl);  break;
        case 96: IO_BUZZER(test_ctrl.ctrl); break;
        case 97: IO_E2_WP(test_ctrl.ctrl);  break;
        default: break;
    }
    if(test_ctrl.state == 0)
    {
        test_ctrl.id = 0;
    }
#ifdef TEST_PRINT
    tim++;
    if(tim > 1000)
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
//    test_print_fill(motor_data.m4.cmd_speed);
    
//    test_print_fill(out_data.pot1);
//    test_print_fill(out_data.pot2);
//    test_print_fill(out_data.pot3);
//    test_print_fill(out_data.pot4);
    
//    test_print_fill(motor_data.m1.pos);
//    test_print_fill(motor_data.m2.pos);
//    test_print_fill(motor_data.m3.pos);
//    test_print_fill(motor_data.m4.pos);

//    test_print();

#endif
}
