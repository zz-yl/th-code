/**@file   interp_table.h
* @brief   插值表
* @author  陈卓哲
* @date    2023/9/18
* @section 参考文档
* 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef INTERP_TABLE_H_
#define INTERP_TABLE_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_sys.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define SIZE_BASE  1110   ///< 插值表长度-基础
#define SIZE_BATTERY 32   ///< 电池电量插值表

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  device_interp_t 
* @brief   设备数据插值表
*/
typedef struct
{
    /* 压差-流量 */
    float x_pre[1];
    float y_flow[1];
    float x_pre_t[1];
    float y_flow_t[1];
}device_interp_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern device_interp_t device_list;
extern const float flow_x_pre[SIZE_BASE];
extern const float flow_y_flow[SIZE_BASE];
extern const float x_battery[SIZE_BATTERY];
extern const float y_battery[SIZE_BATTERY];

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

#endif /* INTERP_TABLE_H_ */

