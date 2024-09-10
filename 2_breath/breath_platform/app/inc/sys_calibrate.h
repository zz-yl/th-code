/**@file   sys_calibrate.h
* @brief   校准模块
* @author  陈卓哲
* @date    2023/11/14
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef SYS_CALIBRATE_H_
#define SYS_CALIBRATE_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "stdint.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define CAL_BUF_MAX  800

/**
* @enum  calibrate_cmd_t
* @brief 校准指令
*/
typedef enum
{
    CAL_CMD_CANCEL, //取消校准
    CAL_CMD_FLOW,   //流量传感器校准
    CAL_CMD_FLOW_B,   //流量传感器校准
    CAL_CMD_FLOW_C,
}calibrate_cmd_t;
/**
* @enum  calibrate_state_t
* @brief 校准状态
*/
typedef enum
{
    CAL_STATE_FREE,    //空闲
    CAL_STATE_START,   //校准开始
    CAL_STATE_STEP1,   //步骤1
    CAL_STATE_STEP2,   //步骤2
    CAL_STATE_STEP3,   //步骤3
    CAL_STATE_STEP4,   //步骤4
    CAL_STATE_STEP5,   //步骤5
    CAL_STATE_STEP6,   //步骤6
    CAL_STATE_STEP7,   //步骤7
    CAL_STATE_STEP8,   //步骤8
    CAL_STATE_STEP9,   //步骤9
    CAL_STATE_STEP10,  //步骤10
    CAL_STATE_STEP11,  //步骤11
    CAL_STATE_STEP12,  //步骤12
    CAL_STATE_RECORD, //数据存储
}calibrate_state_t;

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  calibrate_ctrl_t
* @brief   校准控制
*/
typedef struct
{
    calibrate_cmd_t cmd;      //校准指令
    calibrate_state_t state;  //校准状态
    uint32_t cur_tim;         //当前时间
    
    float data;     //数据
    float start;    //起始值
    float up;       //增量值
    float end;      //结束值
    uint32_t count; //次数
    uint32_t tim_c; //间隔时间ms
    float tim_pre;  //滤波开始时间占时间间隔百分比
    
    float buf1[CAL_BUF_MAX];
    float buf2[CAL_BUF_MAX];
    float buf3[CAL_BUF_MAX];
//    float buf3[CAL_BUF_MAX];
//    float buf4[CAL_BUF_MAX];
}calibrate_ctrl_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern calibrate_ctrl_t cal_ctrl;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void cal_run(void);

#endif /* SYS_CALIBRATE_H_ */
