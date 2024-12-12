/**@file   device_core.c
* @brief   设备模块
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "device_core.h"
#include "device_register.h"
#include "device_io.h"
#include "driver.h"

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


void call_sensor(device_descriptor_t *p_d)
{
    if(p_d->driver != NULL)
    {
        if(p_d->driver->work_fcn != NULL)
        {
            p_d->driver->device = p_d;
            p_d->driver->work_fcn(p_d->driver);
        }
    }
    if(p_d->filter_fcn != NULL)
    {
        p_d->filter_fcn(p_d);
    }
    if(p_d->work_fcn != NULL)
    {
        p_d->work_fcn(p_d);
    }
}
static void device_run_init(void)
{
    device_descriptor_t *head = &device_head;
    
    while(head->next != NULL)
    {
        if(head->next->init_fcn != NULL)
        {
            head->next->init_fcn(head->next);
        }
        head = head->next;
    }
}
static void device_run_work(void)
{
    device_descriptor_t *head = &device_head;
    
    while(head->next != NULL)
    {
        if(head->next->ch < CH_UNDEFINED)
        {
            call_sensor(head->next);
        }
        head = head->next;
    }
}
/**
* @brief     初始化设备
*/
void device_init(void)
{
    device_mount();
    device_run_init();
    driver_init();
}

/**
* @brief     运行设备
*/
void device_run(void)
{
    driver_run();
    device_run_work();
}
