/**@file   test.h
* @brief   ����
* @author  ��׿��
* @date    2023/9/4
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef TEST_H_
#define TEST_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_cfg.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct test_ctrl_t 
* @brief  ����������
*/
typedef struct
{
    float ctrl;
    uint8_t id;
    uint8_t state;
    uint32_t data1;
    uint32_t data2;
    uint32_t data3;
    uint32_t data4;
    uint32_t cnt;
    uint8_t arr[50];
    uint16_t arr_len;
    char cpu_state1[400];
    char cpu_state2[400];
    char sn[20];
}test_ctrl_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern test_ctrl_t test_ctrl;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void test_run(void);
void test_print_fill(float fdata);
void test_print(void);

#endif /* TEST_H_ */
