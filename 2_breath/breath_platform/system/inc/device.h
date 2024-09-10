/**@file   device.h
* @brief   �豸
* @author  ��׿��
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
* @brief   �豸����
*/
typedef struct
{
	float press_dif;    //ѹ��(Pa)
    float press_dif_ff; //ѹ���˲�(Pa)
    float press_lff;    //ѹ���˲�����(Pa)
    float temp;         //�����¶��¶�(��)
    float temp_ff;      //�����¶��¶��˲�(��)
    float flow;         //����(L/min)
    float flow_ff;      //�����˲�(L/min)
    
    float flow_t1;      //��������1(L/min)
    float flow_t2;      //��������2(L/min)
    float flow_t3;      //��������3(L/min)
    float flow_t4;      //��������4(L/min)
    float flow_t1_ff;   //��������1�˲�(L/min)
    float flow_t2_ff;   //��������2�˲�(L/min)
    float flow_t3_ff;   //��������3�˲�(L/min)
    float flow_t4_ff;   //��������4�˲�(L/min)
    
    float flow_real;    //ʵ������(L/min)
    float flow_real_ff; //ʵ�������˲�(L/min)
    float flow_real_lff; //ʵ���˲�����(L/min)
    float flow_t1_lff;
    float flow_t2_lff;
    float flow_t3_lff;
    float flow_t4_lff;
    
    float ff;           //�˲�ϵ��
    
    double ff_add_press;    //�������ۼ�ֵ
    double ff_add_flow;     //�������ۼ�ֵ
    double ff_add_flow_t1;
    double ff_add_flow_t2;
    double ff_add_flow_t3;
    double ff_add_flow_t4;
    uint32_t ff_add_cnt;    //�������ۼ�ֵ����
    uint8_t ff_flag;    //��ղ����˲�
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
