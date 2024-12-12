/**@file   state_machine.h
* @brief   状态机核心模块
* @author  陈卓哲
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
* @brief   状态机返回值
*/
typedef enum
{
    STATE_OK,       //成功
    STATE_FAILED,   //错误
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
    uint16_t          index;  // 系统状态

    state_work_fcn	  work;   // 此状态重复执行的函数
    state_trigger_fcn trig;   // 状态切换触发函数
    state_entry_fcn	  entry;  // 进入此状态执行的函数
    state_exit_fcn	  exit;   // 退出此状态执行的函数
}state_node_t;
/**
* @enum    state_machine_t
* @brief   状态机描述符
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
