/**@file   state_machine.h
* @brief   ״̬������ģ��
* @author  ��׿��
* @date    2023/11/14
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "stdint.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define STATE_NO_TRIG  0
/**
* @enum    state_ret_t 
* @brief   ״̬������ֵ
*/
typedef enum
{
    STATE_OK,       //�ɹ�
    STATE_FAILED,   //����
}state_ret_t;

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

typedef void (*state_work_fcn)(void);
typedef uint16_t (*state_trigger_fcn)(void);
typedef void (*state_entry_fcn)(void);
typedef void (*state_exit_fcn)(void);

typedef struct
{
    uint16_t          index;  // ϵͳ״̬

    state_work_fcn	  work;   // ��״̬�ظ�ִ�еĺ���
    state_trigger_fcn trig;   // ״̬�л���������
    state_entry_fcn	  entry;  // �����״ִ̬�еĺ���
    state_exit_fcn	  exit;   // �˳���״ִ̬�еĺ���
}state_node_t;
/**
* @enum    state_machine_t
* @brief   ״̬��������
*/
typedef struct
{
	state_node_t  *current;
	state_node_t  *table;
	uint16_t state_num;
}state_machine_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

uint16_t state_register(state_machine_t *sm, state_node_t *table, uint16_t size);
uint16_t state_machine_run(state_machine_t *sm);

#endif /* STATE_MACHINE_H_ */
