/**
 ****************************************************************************************************
 * @file        freertos_demo.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-08-01
 * @brief       lwIP SCOKET TCPServer ʵ��
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� ������ H743������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */
 
#include "freertos_demo.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "lwip_comm.h"
#include "lwip_demo.h"
#include "lwipopts.h"
#include "stdio.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "th_cfg.h"


/******************************************************************************************************/
/*FreeRTOS����*/

/* START_TASK ���� ����
 * ����: ������ �������ȼ� ��ջ��С ��������
 */
#define START_TASK_PRIO         5           /* �������ȼ� */
#define START_STK_SIZE          512         /* �����ջ��С */
TaskHandle_t StartTask_Handler;             /* ������ */
void start_task(void *pvParameters);        /* ������ */

/* LWIP_DEMO ���� ����
 * ����: ������ �������ȼ� ��ջ��С ��������
 */
#define LWIP_DMEO_TASK_PRIO     11          /* �������ȼ� */
#define LWIP_DMEO_STK_SIZE      1024        /* �����ջ��С */
TaskHandle_t LWIP_Task_Handler;             /* ������ */
void lwip_demo_task(void *pvParameters);    /* ������ */

/* LED_TASK ���� ����
 * ����: ������ �������ȼ� ��ջ��С ��������
 */
#define LED_TASK_PRIO           10          /* �������ȼ� */
#define LED_STK_SIZE            128         /* �����ջ��С */
TaskHandle_t LEDTask_Handler;               /* ������ */
void led_task(void *pvParameters);          /* ������ */

/* KEY_TASK ���� ����
 * ����: ������ �������ȼ� ��ջ��С ��������
 */
#define KEY_TASK_PRIO           13          /* �������ȼ� */
#define KEY_STK_SIZE            128         /* �����ջ��С */
TaskHandle_t KEYTask_Handler;               /* ������ */
void key_task(void *pvParameters);          /* ������ */

/* DISPLAY_TASK ���� ����
 * ����: ������ �������ȼ� ��ջ��С ��������
 */
#define DISPLAY_TASK_PRIO       7           /* �������ȼ� */
#define DISPLAY_STK_SIZE        512         /* �����ջ��С */
TaskHandle_t DISPLAYTask_Handler;           /* ������ */
void display_task(void *pvParameters);      /* ������ */

/* TH_TASK ���� ����
 * ����: ������ �������ȼ� ��ջ��С ��������
 */
#define TH_TASK_PRIO       7           /* �������ȼ� */
#define TH_STK_SIZE        512         /* �����ջ��С */
TaskHandle_t ThTask_Handler;           /* ������ */
void th_task(void *pvParameters);      /* ������ */

/* ��ʾ��Ϣ���е����� */
#define DISPLAYMSG_Q_NUM    20              /* ��ʾ��Ϣ���е����� */
QueueHandle_t g_display_queue;              /* ��ʾ��Ϣ���о�� */

/******************************************************************************************************/

void th_task(void *pvParameters)
{
    (void)pvParameters;

    while(1)
    {
        th_run();
        vTaskDelay(1);
    }
}

/**
 * @breif       ����UI
 * @param       mode :  bit0:0,������;1,����ǰ�벿��UI
 *                      bit1:0,������;1,���غ�벿��UI
 * @retval      ��
 */
void lwip_test_ui(uint8_t mode)
{
//        lcd_show_string(5, 110, 200, 16, 16, "lwIP Init Successed", MAGENTA);
//            sprintf((char*)buf,"DHCP IP:%d.%d.%d.%d",g_lwipdev.ip[0],g_lwipdev.ip[1],g_lwipdev.ip[2],g_lwipdev.ip[3]);     /* ��ʾ��̬IP��ַ */
//            sprintf((char*)buf,"Static IP:%d.%d.%d.%d",g_lwipdev.ip[0],g_lwipdev.ip[1],g_lwipdev.ip[2],g_lwipdev.ip[3]);    /* ��ӡ��̬IP��ַ */
//        speed = ethernet_chip_get_speed();      /* �õ����� */
}

/**
 * @breif       freertos_demo
 * @param       ��
 * @retval      ��
 */
void freertos_demo(void)
{
    /* start_task���� */
    xTaskCreate((TaskFunction_t )start_task,
                (const char *   )"start_task",
                (uint16_t       )START_STK_SIZE,
                (void *         )NULL,
                (UBaseType_t    )START_TASK_PRIO,
                (TaskHandle_t * )&StartTask_Handler);

    vTaskStartScheduler(); /* ����������� */
}

/**
 * @brief       start_task
 * @param       pvParameters : �������(δ�õ�)
 * @retval      ��
 */
void start_task(void *pvParameters)
{
    pvParameters = pvParameters;
    
    g_lwipdev.lwip_display_fn = lwip_test_ui;
    
    lwip_test_ui(1);    /* ���غ�ǰ����UI */
    
    while(lwip_comm_init() != 0)
    {
//        lcd_show_string(30, 110, 200, 16, 16, "lwIP Init failed!!", RED);
        delay_ms(500);
//        lcd_show_string(30, 110, 200, 16, 16, "Retrying...       ", RED);
        delay_ms(500);
        IO_LED_RUN_TOGGLE;
    }
    
    while (g_lwipdev.dhcpstatus != 2 && g_lwipdev.dhcpstatus != 0xff)/* �ȴ���̬�Ͷ�̬�������  */
    {
        vTaskDelay(500);
    }
    
    taskENTER_CRITICAL();           /* �����ٽ��� */
    
    g_display_queue = xQueueCreate(DISPLAYMSG_Q_NUM,200);      /* ������ϢMessage_Queue,���������200���� */
    
    /* ����lwIP���� */
    xTaskCreate((TaskFunction_t )lwip_demo_task,
                (const char*    )"lwip_demo_task",
                (uint16_t       )LWIP_DMEO_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )LWIP_DMEO_TASK_PRIO,
                (TaskHandle_t*  )&LWIP_Task_Handler);

    /* key���� */
    xTaskCreate((TaskFunction_t )key_task,
                (const char *   )"key_task",
                (uint16_t       )KEY_STK_SIZE,
                (void *         )NULL,
                (UBaseType_t    )KEY_TASK_PRIO,
                (TaskHandle_t * )&KEYTask_Handler);

    /* LED�������� */
    xTaskCreate((TaskFunction_t )led_task,
                (const char*    )"led_task",
                (uint16_t       )LED_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )LED_TASK_PRIO,
                (TaskHandle_t*  )&LEDTask_Handler);

    /* ��ʾ���� */
    xTaskCreate((TaskFunction_t )display_task,
                (const char*    )"display_task",
                (uint16_t       )DISPLAY_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )DISPLAY_TASK_PRIO,
                (TaskHandle_t*  )&DISPLAYTask_Handler);
    /* ���������� */
    xTaskCreate((TaskFunction_t )th_task,
                (const char*    )"th_task",
                (uint16_t       )TH_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )TH_TASK_PRIO,
                (TaskHandle_t*  )&ThTask_Handler);

    vTaskDelete(StartTask_Handler); /* ɾ����ʼ���� */
    taskEXIT_CRITICAL();            /* �˳��ٽ��� */
    
}

/**
 * @brief       lwIP��������
 * @param       pvParameters : �������(δ�õ�)
 * @retval      ��
 */
void lwip_demo_task(void *pvParameters)
{
    pvParameters = pvParameters;

    lwip_demo();            /* lwip���Դ��� */
    
    while (1)
    {
        vTaskDelay(5);
    }
}

/**
 * @brief       key_task
 * @param       pvParameters : �������(δ�õ�)
 * @retval      ��
 */
void key_task(void *pvParameters)
{
    pvParameters = pvParameters;

    while (1)
    {
//        switch(key)
//        {
//            case 1: comm_msg.footrest1 = !comm_msg.footrest1; break;
//            case 2: comm_msg.footrest2 = !comm_msg.footrest2; break;
//            default: break;
//        }
        

//        if (KEY0_PRES == key)
//        {
//            g_lwip_send_flag |= LWIP_SEND_DATA; /* ���LWIP������Ҫ���� */
//        }
        
        vTaskDelay(10);
    }
}

/**
 * @brief       ϵͳ������
 * @param       pvParameters : �������(δ�õ�)
 * @retval      ��
 */
void led_task(void *pvParameters)
{
    pvParameters = pvParameters;

    while(1)
    {
        IO_LED_RUN_TOGGLE;
        vTaskDelay(50);
        IO_LED_RUN_TOGGLE;
        vTaskDelay(950);
    }
}

/**
 * @brief       ��ʾ����
 * @param       pvParameters : �������(δ�õ�)
 * @retval      ��
 */
void display_task(void *pvParameters)
{
    pvParameters = pvParameters;
    
    while(1)
    {  
//        if (g_display_queue != NULL)
//        {
//            if (xQueueReceive(g_display_queue,buffer,portMAX_DELAY))
//            {
//                lcd_show_string(30, 230, lcddev.width - 30, lcddev.height - 230, 16, (char *)buffer, RED); 
//            }
//        }
        
        vTaskDelay(5);
    }
}
