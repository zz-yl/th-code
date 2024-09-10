/**@file   device.h
* @brief   设备
* @author  陈卓哲
* @date    2023/9/5
* @section
* -# 
*
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
* @struct  VOLTAGE_DATA 
* @brief   电压实时值结构体,数据接口
*/
typedef struct
{
    float adc_out0;   ///< 外部ADC通道0
    float adc_out1;   ///< 外部ADC通道1
    float adc_out2;   ///< 外部ADC通道2
    float adc_out3;   ///< 外部ADC通道3
    float adc_out4;   ///< 外部ADC通道4
    float adc_out5;   ///< 外部ADC通道5
    float adc_out6;   ///< 外部ADC通道6
    float adc_out7;   ///< 外部ADC通道7
}VOLTAGE_DATA;

/**
* @struct  DEVICE_IO
* @brief   设备IO数据
*/
typedef struct
{
    /* 电位器位置 */
    float m_pos1;  ///< 电机1
    float m_pos2;  ///< 电机2
    float m_pos3;  ///< 电机3
    float m_pos4;  ///< 电机4
    float e_pos1;  ///< 编码器1
    float e_pos2;  ///< 编码器2
    float e_pos3;  ///< 编码器3
    float e_pos4;  ///< 编码器4
    float pos1;   ///< 电位器1
    float pos2;   ///< 电位器2
    float pos3;   ///< 电位器3
    float pos4;   ///< 电位器4
    float init_pos1;  ///< 初始化电机位置
    float init_pos2;  ///< 初始化电机位置
    float init_pos3;  ///< 初始化电机位置
    float init_pos4;  ///< 初始化电机位置
    
    float struct_end;        ///< 结尾
}DEVICE_IO;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern VOLTAGE_DATA VolData;
extern DEVICE_IO InData;
extern DEVICE_IO OutData;
extern DEVICE_IO InFilterData;
extern DEVICE_IO OutFilterData;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void DevRun(void);

#endif /* DEVICE_H_ */
