/**@file   device.c
* @brief   设备
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

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

dev_data_t dev_data = 
{
    .ff = 0.999,
};

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  驱动设备
*/
static void dev_driver(void)
{
#ifndef USE_WPAS13
    bsp_sdp_read();
    bsp_sfm3300_read(SFM_S1);
    bsp_sfm3300_read(SFM_S2);
    bsp_sfm3300_read(SFM_S3);
    bsp_sfm3300_read(SFM_S4);
#else
    bsp_wpas31_read();
#endif
}
/**
* @brief  处理流量传感器数据
*/
static void dev_pro_data_flow(void)
{
    dev_data.flow_t1 = bsp_sfm3300_get_flow(SFM_S1);
    dev_data.flow_t1_ff = dev_data.flow_t1_ff * dev_data.ff + dev_data.flow_t1 * (1 - dev_data.ff);
    dev_data.flow_t2 = bsp_sfm3300_get_flow(SFM_S2);
    dev_data.flow_t2_ff = dev_data.flow_t2_ff * dev_data.ff + dev_data.flow_t2 * (1 - dev_data.ff);
    dev_data.flow_t3 = bsp_sfm3300_get_flow(SFM_S3);
    dev_data.flow_t3_ff = dev_data.flow_t3_ff * dev_data.ff + dev_data.flow_t3 * (1 - dev_data.ff);
    dev_data.flow_t4 = bsp_sfm3300_get_flow(SFM_S4);  //*1.02;//补偿2%
    dev_data.flow_t4_ff = dev_data.flow_t4_ff * dev_data.ff + dev_data.flow_t4 * (1 - dev_data.ff);
    
    dev_data.flow_real = dev_data.flow_t1 + dev_data.flow_t2 + dev_data.flow_t3 + dev_data.flow_t4;
    dev_data.flow_real_ff = dev_data.flow_t1_ff + dev_data.flow_t2_ff + dev_data.flow_t3_ff + dev_data.flow_t4_ff;
//    dev_data.flow_real = dev_data.flow_t4;
//    dev_data.flow_real_ff = dev_data.flow_t4_ff;
//    dev_data.flow_real = dev_data.flow_t2 + dev_data.flow_t2 + dev_data.flow_t3 + dev_data.flow_t4;
//    dev_data.flow_real_ff = dev_data.flow_t2_ff + dev_data.flow_t2_ff + dev_data.flow_t3_ff + dev_data.flow_t4_ff;
}
/**
* @brief  处理数据
*/
static void dev_pro_data(void)
{
#ifndef USE_WPAS13
    dev_data.press_dif = bsp_sdp_get_press();
    dev_data.temp      = bsp_sdp_get_temp();
    dev_data.flow      = interp_1d(dev_data.press_dif, flow_x_pre, flow_y_flow, SIZE_BASE, LIMIT_UEN);
#else
    dev_data.press_dif   = bsp_wpas31_get_press();
    dev_data.temp_breath = bsp_wpas31_get_temp();
#endif
    dev_data.press_dif_ff = dev_data.press_dif_ff * dev_data.ff + dev_data.press_dif * (1 - dev_data.ff);
    dev_data.temp_ff      = dev_data.temp_ff * dev_data.ff + dev_data.temp * (1 - dev_data.ff);
    dev_data.flow_ff      = interp_1d(dev_data.press_dif_ff, flow_x_pre, flow_y_flow, SIZE_BASE, LIMIT_UEN);
    
    dev_data.ff_add_press += dev_data.press_dif;
    dev_data.ff_add_flow += dev_data.flow_real;
    dev_data.ff_add_flow_t1 += dev_data.flow_t1;
    dev_data.ff_add_flow_t2 += dev_data.flow_t2;
    dev_data.ff_add_flow_t3 += dev_data.flow_t3;
    dev_data.ff_add_flow_t4 += dev_data.flow_t4;
    dev_data.ff_add_cnt++;
    dev_data.press_lff = dev_data.ff_add_press / dev_data.ff_add_cnt;
    dev_data.flow_real_lff = dev_data.ff_add_flow / dev_data.ff_add_cnt;
    dev_data.flow_t1_lff = dev_data.ff_add_flow_t1 / dev_data.ff_add_cnt;
    dev_data.flow_t2_lff = dev_data.ff_add_flow_t2 / dev_data.ff_add_cnt;
    dev_data.flow_t3_lff = dev_data.ff_add_flow_t3 / dev_data.ff_add_cnt;
    dev_data.flow_t4_lff = dev_data.ff_add_flow_t4 / dev_data.ff_add_cnt;
    if(dev_data.ff_flag)
    {
        dev_data.ff_flag = 0;
        dev_data.ff_add_press = dev_data.press_dif;
        dev_data.press_lff = dev_data.press_dif;
        dev_data.ff_add_flow = dev_data.flow_real;
        dev_data.flow_real_lff = dev_data.flow_real;
        dev_data.ff_add_flow_t1 = dev_data.flow_t1;
        dev_data.ff_add_flow_t2 = dev_data.flow_t2;
        dev_data.ff_add_flow_t3 = dev_data.flow_t3;
        dev_data.ff_add_flow_t4 = dev_data.flow_t4;
        dev_data.flow_t1_lff = dev_data.flow_t1;
        dev_data.flow_t2_lff = dev_data.flow_t2;
        dev_data.flow_t3_lff = dev_data.flow_t3;
        dev_data.flow_t4_lff = dev_data.flow_t4;
        dev_data.ff_add_cnt = 1;
    }
    
    dev_pro_data_flow();
}
/**
* @brief  设备模块运行
*/
void dev_run(void)
{
    dev_driver();
    dev_pro_data();
}
/**
* @brief  设备模块初始化
*/
void dev_init(void)
{
    bsp_sdp_start();
    bsp_sfm3300_start(SFM_S1);
    bsp_sfm3300_start(SFM_S2);
    bsp_sfm3300_start(SFM_S3);
    bsp_sfm3300_start(SFM_S4);
}
