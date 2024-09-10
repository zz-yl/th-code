/**@file   motor_comm.h
* @brief   ���ڵ������(�������)
* @author  ��׿��
* @date    2024/2/20
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef MOTOR_COMM_H_
#define MOTOR_COMM_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_cfg.h"
#include "data_queue.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**
* @enum    motor_id_t
* @brief   ���id
*/
typedef enum
{
    MOTOR_1,
    MOTOR_2,
    MOTOR_3,
    MOTOR_4,
    MOTOR_5,
}motor_id_t;
/**
* @enum    motor_cmd_mode_t
* @brief   ���ָ��ģʽ
*/
typedef enum
{
    MOTOR_POS,   //λ��ģʽ
    MOTOR_SPEED, //�ٶ�ģʽ
}motor_cmd_mode_t;
/**
* @enum    motor_state_run_t
* @brief   �������״̬
*/
typedef enum
{
    MOTOR_STOP,
    MOTOR_RUN,
    MOTOR_FAULT,
}motor_state_run_t;

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  motor_t
* @brief   ���������
*/
typedef struct motor_node
{
    float cmd_pos;       //λ��ָ��(mm)
    float pos;           //���λ��(mm)
    float speed;         //����ٶ�(mm/s)
    uint32_t pos_all;    //������г�(mm)
    motor_state_run_t state_r; //�������״̬
    uint32_t tim_run;   //���������ʱ��

    struct motor_node *next;
}motor_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern motor_t motor_data[5];

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void motor_init(void);
void motor_run(void);
void motor_get_pos_all(void);
void motor_send_pos_all(void);
void motor_ctrl_pos(motor_id_t id, float pos);
void motor_monitor(void);
void motor_fill_msg(motor_id_t id, uint16_t cmd, uint8_t *data, uint8_t len);

#endif /* MOTOR_COMM_H_ */
