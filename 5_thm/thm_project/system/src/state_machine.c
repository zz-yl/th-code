/**@file   state_machine.c
* @brief   状态机核心模块
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "state_machine.h"
#include "stddef.h"
#include "string.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

static void work_default(void)
{

}
static void entry_default(void)
{

}

static void exit_default(void)
{

}
/**
* @brief  状态机运行
* @attention 
*/
uint16_t state_machine_run(state_machine_t *sm)
{
    uint16_t new_state = STATE_NO_TRIG;

    sm->current->work();
    if((new_state = sm->current->trig()) != STATE_NO_TRIG)
    {
        if(new_state > sm->state_num)
        {
            return STATE_NO_TRIG;
        }
        sm->current->exit();
        sm->current = &sm->table[new_state-1];
        sm->current->entry();
    }
    return new_state;
}
/**
* @brief  状态机注册
* @attention 
*/
uint16_t state_register(state_machine_t *sm, state_node_t *table, uint16_t size)
{
    uint16_t i = 0;

    if(sm == NULL)
    {
        return STATE_FAILED;
    }
    if(table == NULL)
    {
        return STATE_FAILED;
    }

    sm->current = table;
    sm->table = table;
    sm->state_num = size;

    for(i=0; i<size; i++)
    {
        if(table[i].work == NULL)
        {
            table[i].work = work_default;
        }
        if(table[i].entry == NULL)
        {
            table[i].entry = entry_default;
        }
        if(table[i].exit == NULL)
        {
            table[i].exit = exit_default;
        }
    }

    return STATE_OK;
}
