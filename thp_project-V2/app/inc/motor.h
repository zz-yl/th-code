/**@file   motor.h
* @brief   �����������
* @author  ��׿��
* @date    2023/11/15
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef MOTOR_H_
#define MOTOR_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_cfg.h"
#include "pid.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define MOTOR_MICROSTEP  256   //�������������ϸ��

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
/**
* @enum    motor_state_lock_t
* @brief   �����ת״̬
*/
typedef enum
{
    MOTOR_UNLOCKED,
    MOTOR_LOCKED,
}motor_state_lock_t;

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  motor_t
* @brief   ���������
*/
typedef struct motor_node
{
    motor_id_t id;       //���id
    float pos;           //���λ��(mm)
    float pos_init;      //����ϵ��ʼλ��(mm)
    float pos_last;      //����ϴ�λ��(mm)
    float speed;         //����ٶ�(mm/s)
    uint32_t pos_all;    //������г�(mm)
    motor_state_run_t state_r; //�������״̬
    motor_state_lock_t state_l; //�����ת״̬
    uint16_t cnt_lock;   //��ת����
    uint16_t cnt_stop;   //ֹͣ����
    uint32_t cnt_recover; //��ת�ָ���������
    motor_cmd_mode_t cmd_mode;  //���ָ��ģʽ
    float cmd_pos;       //λ��ָ��(mm)
    float cmd_speed;     //�ٶ�ָ��(mm/s)
    float cmd_pos_last;  //�ϴ�λ��ָ��(mm)
    float cmd_speed_last;//�ϴ��ٶ�ָ��(mm/s)
    float cmd_speed_limit;//�ٶ�ָ������(mm/s)
    float cmd_frq;       //Ƶ��ָ��(Hz)
    float cmd_speed_target;//�ٶ�Ŀ��
    pid_data_t *pid;     //pid����
    void (*init)(void);  //�����ʼ������
    void (*pid_pos)(struct motor_node *);  //���λ�ñջ�
    void (*set_frq)(float);  //����Ƶ�����
    
    struct motor_node *next;
}motor_t;

/**
* @struct  motor_data_t
* @brief   �������
*/
typedef struct
{
    motor_t m1;
    motor_t m2;
    motor_t m3;
    motor_t m4;
    motor_t m5;
    float tim_run;  //�������е�ʱ��
}motor_data_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern motor_data_t motor_data;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void motor_write_mode(motor_cmd_mode_t mode);
void motor_clear_lock(void);
void motor_stop_all(void);
void motor_init(void);
void motor_run(void);
void motor_monitor(void);
void motor_init_pos(void);
void motor_en(uint8_t cmd);
void motor_dir(motor_id_t id, uint8_t cmd);
uint8_t motor_read_lock(void);

void motor_pos_ctrl(motor_id_t id, float pos);
void motor_speed_ctrl(motor_id_t id, float speed);

#endif /* MOTOR_H_ */
