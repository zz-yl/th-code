/**@file   driver.h
* @brief   驱动程序处理
* @author  陈卓哲
* @date    2023/11/10
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef DRIVER_H_
#define DRIVER_H_

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
* @struct  voltage_data_t 
* @brief   电压实时值结构体,数据接口
*/
typedef struct
{
    float ex0;   ///< 外部ADC通道0
    float ex1;   ///< 外部ADC通道1
    float ex2;   ///< 外部ADC通道2
    float ex3;   ///< 外部ADC通道3
    float ex4;   ///< 外部ADC通道4
    float ex5;   ///< 外部ADC通道5
    float ex6;   ///< 外部ADC通道6
    float ex7;   ///< 外部ADC通道7
}voltage_data_t;
/**
* @struct board_data_t
* @brief  板级数据
*/
typedef struct
{
    float u_ref;  ///< 电位计电源电压
}board_data_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern voltage_data_t vol_data;
extern board_data_t board_data;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void driver_init(void);
void driver_run(void);

#endif /* DRIVER_H_ */
