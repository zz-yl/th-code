/**@file   sys_calibrate.h
* @brief   У׼ģ��
* @author  ��׿��
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
* @brief У׼ָ��
*/
typedef enum
{
    CAL_CMD_CANCEL, //ȡ��У׼
    CAL_CMD_FLOW,   //����������У׼
    CAL_CMD_FLOW_B,   //����������У׼
    CAL_CMD_FLOW_C,
}calibrate_cmd_t;
/**
* @enum  calibrate_state_t
* @brief У׼״̬
*/
typedef enum
{
    CAL_STATE_FREE,    //����
    CAL_STATE_START,   //У׼��ʼ
    CAL_STATE_STEP1,   //����1
    CAL_STATE_STEP2,   //����2
    CAL_STATE_STEP3,   //����3
    CAL_STATE_STEP4,   //����4
    CAL_STATE_STEP5,   //����5
    CAL_STATE_STEP6,   //����6
    CAL_STATE_STEP7,   //����7
    CAL_STATE_STEP8,   //����8
    CAL_STATE_STEP9,   //����9
    CAL_STATE_STEP10,  //����10
    CAL_STATE_STEP11,  //����11
    CAL_STATE_STEP12,  //����12
    CAL_STATE_RECORD, //���ݴ洢
}calibrate_state_t;

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  calibrate_ctrl_t
* @brief   У׼����
*/
typedef struct
{
    calibrate_cmd_t cmd;      //У׼ָ��
    calibrate_state_t state;  //У׼״̬
    uint32_t cur_tim;         //��ǰʱ��
    
    float data;     //����
    float start;    //��ʼֵ
    float up;       //����ֵ
    float end;      //����ֵ
    uint32_t count; //����
    uint32_t tim_c; //���ʱ��ms
    float tim_pre;  //�˲���ʼʱ��ռʱ�����ٷֱ�
    
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
