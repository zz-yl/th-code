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

#include "test.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/* �����ջ��С */
#define TASK_STK_SIZE_START  128
#define TASK_STK_SIZE        256
#define TASK_STK_SIZE_COMM   1024
#define TASK_STK_SIZE_MOTOR  2048

/**
* @enum    TASK_PRIO
* @brief   �������ȼ�
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

/* ������ */
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
* @brief    �豸����
* @attention 
*/
void task_dev(void *p_arg)
{
    (void)p_arg;
    
    while(1)
    {
        motor_monitor();
        vTaskDelay(TASK_TIM);
    }
}
/**
* @brief    �����������
* @attention 
*/
void task_motor(void *p_arg)
{
    (void)p_arg;

    while(1)
    {
        sm_motor_run();
        
        vTaskDelay(TASK_TIM);
    }
}
/**
* @brief    ״̬������
* @attention 
*/
void task_sys(void *p_arg)
{
    (void)p_arg;
    
    while(1)
    {
        sm_sys_run();
        
        vTaskDelay(TASK_TIM);
    }
}
/**
* @brief    ͨ������
* @attention 
*/
void task_comm(void *p_arg)
{
    (void)p_arg;
    static uint16_t tim = 0;
    uint16_t tim_tar = 70;

    while(1)
    {
        comm_run();
        
        motor_run();

//        if(sm_motor_state() == MOTOR_STATE_RUN)
//        {
//            tim_tar = 60;
//        }
//        else
//        {
//            tim_tar = 250;
//        }
        tim++;
        if(tim > tim_tar)
        {
            tim = 0;
            motor_get_pos_all();
        }
        
        vTaskDelay(TASK_TIM);
    }
}
/**
* @brief    ��������
* @attention 
*/
void task_test(void *p_arg)
{
    (void)p_arg;
//    vTaskDelay(2000);
//    sm_sys_toggle(SYS_TEST);
//    test_ctrl.ctrl = 5;
//    test_ctrl.state = 1;
//    test_ctrl.id = 28;
    
//    vTaskDelay(2000);
//    
//    test_ctrl.state = 1;
//    test_ctrl.id = 18;
    
    while(1)
    {
        test_run();
        vTaskDelay(TASK_TIM);
    }
}
/**
* @brief    ϵͳ������
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
* @brief  ��ʼ������
* @attention 
*/
void task_start(void *p_arg)
{
    (void)p_arg;
    
    taskENTER_CRITICAL();           /* �����ٽ��� */
    xTaskCreate((TaskFunction_t )task_dev,              /* ������ */
                (const char*    )"task_dev",            /* �������� */
                (uint16_t       )TASK_STK_SIZE,         /* �����ջ��С */
                (void*          )NULL,                  /* ������������Ĳ��� */
                (UBaseType_t    )TASK_PRIO_2,           /* �������ȼ� */
                (TaskHandle_t*  )&task_handle_dev);     /* ������ */
    xTaskCreate((TaskFunction_t )task_motor,            /* ������ */
                (const char*    )"task_motor",          /* �������� */
                (uint16_t       )TASK_STK_SIZE_MOTOR,         /* �����ջ��С */
                (void*          )NULL,                  /* ������������Ĳ��� */
                (UBaseType_t    )TASK_PRIO_3,           /* �������ȼ� */
                (TaskHandle_t*  )&task_handle_motor);   /* ������ */
    xTaskCreate((TaskFunction_t )task_sys,              /* ������ */
                (const char*    )"task_sys_state",      /* �������� */
                (uint16_t       )TASK_STK_SIZE,         /* �����ջ��С */
                (void*          )NULL,                  /* ������������Ĳ��� */
                (UBaseType_t    )TASK_PRIO_4,           /* �������ȼ� */
                (TaskHandle_t*  )&task_handle_sys);     /* ������ */
    xTaskCreate((TaskFunction_t )task_comm,             /* ������ */
                (const char*    )"task_comm",           /* �������� */
                (uint16_t       )TASK_STK_SIZE_COMM,    /* �����ջ��С */
                (void*          )NULL,                  /* ������������Ĳ��� */
                (UBaseType_t    )TASK_PRIO_5,           /* �������ȼ� */
                (TaskHandle_t*  )&task_handle_comm);    /* ������ */
    xTaskCreate((TaskFunction_t )task_test,             /* ������ */
                (const char*    )"task_test",           /* �������� */
                (uint16_t       )TASK_STK_SIZE,         /* �����ջ��С */
                (void*          )NULL,                  /* ������������Ĳ��� */
                (UBaseType_t    )TASK_PRIO_6,           /* �������ȼ� */
                (TaskHandle_t*  )&task_handle_test);    /* ������ */
    xTaskCreate((TaskFunction_t )task_led,              /* ������ */
                (const char*    )"task_led",            /* �������� */
                (uint16_t       )TASK_STK_SIZE,         /* �����ջ��С */
                (void*          )NULL,                  /* ������������Ĳ��� */
                (UBaseType_t    )TASK_PRIO_8,           /* �������ȼ� */
                (TaskHandle_t*  )&task_handle_led);     /* ������ */
    vTaskDelete(task_handle_start); /* ɾ����ʼ���� */
    taskEXIT_CRITICAL();            /* �˳��ٽ��� */
}
/**
* @brief  ������ʼ����
* @attention 
*/
void task_create(void)
{
    xTaskCreate((TaskFunction_t )task_start,            /* ������ */
                (const char*    )"task_start",          /* �������� */
                (uint16_t       )TASK_STK_SIZE_START,   /* �����ջ��С */
                (void*          )NULL,                  /* ������������Ĳ��� */
                (UBaseType_t    )TASK_PRIO_START,       /* �������ȼ� */
                (TaskHandle_t*  )&task_handle_start);   /* ������ */
    vTaskStartScheduler();
}
