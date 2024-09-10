/**@file   test.c
* @brief   测试
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "test.h"

#include "FreeRTOS.h"
#include "task.h"
#include "device.h"
#include "thb_cfg.h"
#include "data_algorithm.h"
#include "interp_table.h"

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
    .arr = {0x10, 0x00, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
    .arr_len = 2,
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
    uart1_send(print_buf, print_cnt);
    print_cnt = 0;
}

void getpre(void)
{
    int i = 0;
    
    for(; i<71; i++)
    {
        device_list.x_pre_t[i] = interp_1d(device_list.y_flow_t[i], device_list.y_flow, device_list.x_pre, SIZE_BASE, LIMIT_UEN);
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
        case 1: i2c1_read(test_ctrl.arr, test_ctrl.arr_len, ADDR_SDP31); break;
        case 2: i2c1_write(test_ctrl.arr, test_ctrl.arr_len, ADDR_SDP31); break;
        case 3: uart1_send(test_ctrl.arr, test_ctrl.arr_len); break;
        case 4: getpre(); break;
        case 5: i2c2_read(test_ctrl.arr, test_ctrl.arr_len, ADDR_SFM3300); break;
        case 6: i2c2_write(test_ctrl.arr, test_ctrl.arr_len, ADDR_SFM3300); break;
        case 7: i2cs_read(i2cs1, test_ctrl.arr, test_ctrl.arr_len, ADDR_SFM3300); break;
        case 8: i2cs_write(i2cs1, test_ctrl.arr, test_ctrl.arr_len, ADDR_SFM3300); break;
        
        case 11: bsp_valve_ctrl(test_ctrl.data1); break;
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
//        printf("press=%0.6f; temp=%0.6f\n", dev_data.press_dif, dev_data.temp_breath);
//        vTaskGetRunTimeStats((char *)&test_ctrl.cpu_state1);
//        printf("sta:\n%s\n", test_ctrl.cpu_state1);
//        vTaskList((char *)&test_ctrl.cpu_state2);
//        printf("list:\n%s\n", test_ctrl.cpu_state2);
        
//        printf("run times:%d; m1:%0.4f; m2:%0.4f; m3:%0.4f; m4:%0.4f; state:%d,%d,%d,%d; lock_r:%d,%d,%d,%d; time:%d\r\n",
//               test_ctrl.cnt, motor_data.m1.pos, motor_data.m2.pos, motor_data.m3.pos, motor_data.m4.pos,
//               motor_data.m1.state_l, motor_data.m2.state_l, motor_data.m3.state_l, motor_data.m4.state_l, 
//               motor_data.m1.cnt_recover, motor_data.m2.cnt_recover, motor_data.m3.cnt_recover, motor_data.m4.cnt_recover, sys_timer);
    }

//    test_print_fill(dev_data.flow_t1);
//    test_print_fill(dev_data.flow_t2_ff);
//    test_print_fill(dev_data.flow_t3);
//    test_print_fill(dev_data.flow_t4);
//    test_print_fill(dev_data.flow_t1_lff);
//    test_print_fill(dev_data.flow_t2_lff);
//    test_print_fill(dev_data.flow_t3_lff);
//    test_print_fill(dev_data.flow_t4_lff);
    test_print_fill(dev_data.flow_real);
    test_print_fill(dev_data.flow_real_ff);
    test_print_fill(dev_data.flow_real_lff);
    test_print_fill(dev_data.press_dif);
    test_print_fill(dev_data.press_dif_ff);
    test_print_fill(dev_data.press_lff);
//    test_print_fill(dev_data.temp);
//    test_print_fill(dev_data.temp_ff);
    test_print_fill(dev_data.flow);
    test_print_fill(dev_data.flow_ff);

    test_print();

#endif
}
