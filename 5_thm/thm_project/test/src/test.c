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

/**
* @brief  测试功能运行
* @attention 
*/
void test_run(void)
{
    static uint16_t tim = 0;

    switch(test_ctrl.id)
    {
        case 1:  IO_LED1(test_ctrl.ctrl); break;
        case 2:  IO_LED2(test_ctrl.ctrl); break;
        case 3:  IO_LED3(test_ctrl.ctrl); break;
        case 4:  IO_LED4(test_ctrl.ctrl); break;
        case 5:  IO_LED5(test_ctrl.ctrl); break;
        case 6:  IO_LED6(test_ctrl.ctrl); break;
        case 7:  IO_LED7(test_ctrl.ctrl); break;
        case 8:  IO_LED8(test_ctrl.ctrl); break;
        case 9:  IO_LED9(test_ctrl.ctrl); break;
        case 10: IO_LED10(test_ctrl.ctrl); break;
        case 11: IO_LED11(test_ctrl.ctrl); break;
        case 12: IO_LED12(test_ctrl.ctrl); break;
        case 13: IO_LED13(test_ctrl.ctrl); break;
        case 14: IO_LED14(test_ctrl.ctrl); break;
        case 15: IO_LED15(test_ctrl.ctrl); break;
        case 16: IO_LED16(test_ctrl.ctrl); break;
        case 17: IO_LED17(test_ctrl.ctrl); break;
        case 18: IO_LED18(test_ctrl.ctrl); break;
        
        case 21: test_ctrl.arr[0] = IO_KEY1; break;
        case 22: test_ctrl.arr[1] = IO_KEY2; break;
        case 23: test_ctrl.arr[2] = IO_KEY3; break;
        case 24: test_ctrl.arr[3] = IO_KEY4; break;
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
        vTaskGetRunTimeStats((char *)&test_ctrl.cpu_state1);
        printf("sta:\n%s\n", test_ctrl.cpu_state1);
        vTaskList((char *)&test_ctrl.cpu_state2);
        printf("list:\n%s\n", test_ctrl.cpu_state2);
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
