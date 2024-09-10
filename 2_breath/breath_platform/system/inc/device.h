/**@file   device.h
* @brief   设备
* @author  陈卓哲
* @date    2024/4/7
* @version 1.00.0.0
**************************************************************************************************/

#ifndef DEVICE_H_
#define DEVICE_H_

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
* @struct  dev_data_t
* @brief   设备数据
*/
typedef struct
{
	float press_dif;    //压差(Pa)
    float press_dif_ff; //压差滤波(Pa)
    float press_lff;    //压差滤波测试(Pa)
    float temp;         //呼吸温度温度(℃)
    float temp_ff;      //呼吸温度温度滤波(℃)
    float flow;         //流量(L/min)
    float flow_ff;      //流量滤波(L/min)
    
    float flow_t1;      //流量测试1(L/min)
    float flow_t2;      //流量测试2(L/min)
    float flow_t3;      //流量测试3(L/min)
    float flow_t4;      //流量测试4(L/min)
    float flow_t1_ff;   //流量测试1滤波(L/min)
    float flow_t2_ff;   //流量测试2滤波(L/min)
    float flow_t3_ff;   //流量测试3滤波(L/min)
    float flow_t4_ff;   //流量测试4滤波(L/min)
    
    float flow_real;    //实际流量(L/min)
    float flow_real_ff; //实际流量滤波(L/min)
    float flow_real_lff; //实际滤波测试(L/min)
    float flow_t1_lff;
    float flow_t2_lff;
    float flow_t3_lff;
    float flow_t4_lff;
    
    float ff;           //滤波系数
    
    double ff_add_press;    //测试用累加值
    double ff_add_flow;     //测试用累加值
    double ff_add_flow_t1;
    double ff_add_flow_t2;
    double ff_add_flow_t3;
    double ff_add_flow_t4;
    uint32_t ff_add_cnt;    //测试用累加值次数
    uint8_t ff_flag;    //清空测试滤波
}dev_data_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern dev_data_t dev_data;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void dev_run(void);
void dev_init(void);

#endif /* DEVICE_H_ */
