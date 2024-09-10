/**@file   task_cfg.c
* @brief
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "task_cfg.h"

#include "FreeRTOS.h"
#include "task.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/* 任务堆栈大小 */
#define TASK_STK_SIZE_START  128
#define TASK_STK_SIZE        256
#define TASK_STK_SIZE_COMM   1024

/**
* @enum    TASK_PRIO
* @brief   任务优先级
*/
typedef enum
{
    TASK_PRIO_START = 1,
    TASK_PRIO_2,
    TASK_PRIO_3,
    TASK_PRIO_4,
    TASK_PRIO_5,
    TASK_PRIO_6,
    TASK_PRIO_7,
    TASK_PRIO_8,
}TASK_PRIO;

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

/* 任务句柄 */
TaskHandle_t task_handle_start;
TaskHandle_t task_handle_dev;
TaskHandle_t task_handle_motor;
TaskHandle_t task_handle_sys;
TaskHandle_t task_handle_comm;
TaskHandle_t task_handle_test;
TaskHandle_t task_handle_led;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief    设备任务
* @attention 
*/
void task_dev(void *p_arg)
{
    (void)p_arg;
    
    vTaskDelay(500);
    dev_init();
    vTaskDelay(500);
    
    while(1)
    {
        dev_run();
        vTaskDelay(TASK_TIM);
    }
}
/**
* @brief    电机控制任务
* @attention 
*/
void task_motor(void *p_arg)
{
    (void)p_arg;

    while(1)
    {
        motor_run();
        valve_run();
        vTaskDelay(5);
    }
}
/**
* @brief    状态机任务
* @attention 
*/
void task_sys(void *p_arg)
{
    (void)p_arg;
    
    while(1)
    {
        cal_run();
        vTaskDelay(TASK_TIM);
    }
}
/**
* @brief    通信任务
* @attention 
*/
void task_comm(void *p_arg)
{
    (void)p_arg;

    while(1)
    {
        comm_run();
        vTaskDelay(TASK_TIM);
    }
}
/**
* @brief    测试任务
* @attention 
*/
void task_test(void *p_arg)
{
    (void)p_arg;
    
    while(1)
    {
        test_run();
        vTaskDelay(TASK_TIM);
    }
}
/**
* @brief    系统灯任务
* @attention 
*/
void task_led(void *p_arg)
{
    (void)p_arg;
    
    while(1)
    {
        IO_LED_RUN_TOGGLE;
        vTaskDelay(500);
    }
}

/**
* @brief  起始任务函数
* @attention 
*/
void task_start(void *p_arg)
{
    (void)p_arg;
    
    taskENTER_CRITICAL();           /* 进入临界区 */
    xTaskCreate((TaskFunction_t )task_dev,              /* 任务函数 */
                (const char*    )"task_dev",            /* 任务名称 */
                (uint16_t       )TASK_STK_SIZE,         /* 任务堆栈大小 */
                (void*          )NULL,                  /* 传入给任务函数的参数 */
                (UBaseType_t    )TASK_PRIO_2,           /* 任务优先级 */
                (TaskHandle_t*  )&task_handle_dev);     /* 任务句柄 */
    xTaskCreate((TaskFunction_t )task_motor,            /* 任务函数 */
                (const char*    )"task_motor",          /* 任务名称 */
                (uint16_t       )TASK_STK_SIZE,         /* 任务堆栈大小 */
                (void*          )NULL,                  /* 传入给任务函数的参数 */
                (UBaseType_t    )TASK_PRIO_3,           /* 任务优先级 */
                (TaskHandle_t*  )&task_handle_motor);   /* 任务句柄 */
    xTaskCreate((TaskFunction_t )task_sys,              /* 任务函数 */
                (const char*    )"task_sys_state",      /* 任务名称 */
                (uint16_t       )TASK_STK_SIZE,         /* 任务堆栈大小 */
                (void*          )NULL,                  /* 传入给任务函数的参数 */
                (UBaseType_t    )TASK_PRIO_4,           /* 任务优先级 */
                (TaskHandle_t*  )&task_handle_sys);     /* 任务句柄 */
    xTaskCreate((TaskFunction_t )task_comm,             /* 任务函数 */
                (const char*    )"task_comm",           /* 任务名称 */
                (uint16_t       )TASK_STK_SIZE_COMM,    /* 任务堆栈大小 */
                (void*          )NULL,                  /* 传入给任务函数的参数 */
                (UBaseType_t    )TASK_PRIO_5,           /* 任务优先级 */
                (TaskHandle_t*  )&task_handle_comm);    /* 任务句柄 */
    xTaskCreate((TaskFunction_t )task_test,             /* 任务函数 */
                (const char*    )"task_test",           /* 任务名称 */
                (uint16_t       )TASK_STK_SIZE,         /* 任务堆栈大小 */
                (void*          )NULL,                  /* 传入给任务函数的参数 */
                (UBaseType_t    )TASK_PRIO_6,           /* 任务优先级 */
                (TaskHandle_t*  )&task_handle_test);    /* 任务句柄 */
    xTaskCreate((TaskFunction_t )task_led,              /* 任务函数 */
                (const char*    )"task_led",            /* 任务名称 */
                (uint16_t       )TASK_STK_SIZE,         /* 任务堆栈大小 */
                (void*          )NULL,                  /* 传入给任务函数的参数 */
                (UBaseType_t    )TASK_PRIO_8,           /* 任务优先级 */
                (TaskHandle_t*  )&task_handle_led);     /* 任务句柄 */
    vTaskDelete(task_handle_start); /* 删除开始任务 */
    taskEXIT_CRITICAL();            /* 退出临界区 */
}
/**
* @brief  创建起始任务
* @attention 
*/
void task_create(void)
{
    xTaskCreate((TaskFunction_t )task_start,            /* 任务函数 */
                (const char*    )"task_start",          /* 任务名称 */
                (uint16_t       )TASK_STK_SIZE_START,   /* 任务堆栈大小 */
                (void*          )NULL,                  /* 传入给任务函数的参数 */
                (UBaseType_t    )TASK_PRIO_START,       /* 任务优先级 */
                (TaskHandle_t*  )&task_handle_start);   /* 任务句柄 */
    vTaskStartScheduler();
}
