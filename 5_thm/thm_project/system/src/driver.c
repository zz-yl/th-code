/**@file   driver.c
* @brief   驱动程序处理
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "driver.h"
#include "device_register.h"
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

driver_descriptor_t driver_head;
voltage_data_t vol_data;
board_data_t board_data;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

static void driver_init_adc(void)
{
    ads131_io_init();
    spi2_init();
    adcStartup();
}
adc_channel_data adc131_t = {0};
static void driver_work_adc(driver_descriptor_t *p_driver)
{
    if(p_driver->state != DRIVER_BUSY)
    {
        readData(&adc131_t);

        adc131_t.channel0 &= 0x7FFFFF;  //去负号
        adc131_t.channel1 &= 0x7FFFFF;  //去负号
        adc131_t.channel2 &= 0x7FFFFF;  //去负号
        adc131_t.channel3 &= 0x7FFFFF;  //去负号
        adc131_t.channel4 &= 0x7FFFFF;  //去负号
        adc131_t.channel5 &= 0x7FFFFF;  //去负号
        adc131_t.channel6 &= 0x7FFFFF;  //去负号
        adc131_t.channel7 &= 0x7FFFFF;  //去负号
        vol_data.ex0 = (float)adc131_t.channel0 * 1245.0f / 0x7FFFFF;
        vol_data.ex1 = (float)adc131_t.channel1 * 1245.0f / 0x7FFFFF;
        vol_data.ex2 = (float)adc131_t.channel2 * 1245.0f / 0x7FFFFF;
        vol_data.ex3 = (float)adc131_t.channel3 * 1245.0f / 0x7FFFFF;
        vol_data.ex4 = (float)adc131_t.channel4 * 1245.0f / 0x7FFFFF;
        vol_data.ex5 = (float)adc131_t.channel5 * 1245.0f / 0x7FFFFF;
        vol_data.ex6 = (float)adc131_t.channel6 * 1245.0f / 0x7FFFFF;
        vol_data.ex7 = (float)adc131_t.channel7 * 1245.0f / 0x7FFFFF;

        p_driver->state = DRIVER_BUSY;
    }

    *p_driver->device->data.in = (float)*((int32_t *)(&adc131_t.channel0) + p_driver->device->ch);
}
static void driver_init_spi1(void)
{

}
static void driver_work_spi1(driver_descriptor_t *p_driver)
{
    if(p_driver->state != DRIVER_BUSY)
    {
//        spi1_read_write(encoder_cmd, 6);
        
        p_driver->state = DRIVER_BUSY;
    }
    *p_driver->device->data.in = encoder_get(1);
}
static void driver_init_spi3(void)
{
    
}
static void driver_work_spi3(driver_descriptor_t *p_driver)
{
    if(p_driver->state != DRIVER_BUSY)
    {
//        spi3_read_write(encoder_cmd, 6);
        
        p_driver->state = DRIVER_BUSY;
    }
    *p_driver->device->data.in = encoder_get(3);
}
static void driver_init_spi4(void)
{

}
static void driver_work_spi4(driver_descriptor_t *p_driver)
{
    if(p_driver->state != DRIVER_BUSY)
    {
//        spi4_read_write(encoder_cmd, 6);
        
        p_driver->state = DRIVER_BUSY;
    }
    *p_driver->device->data.in = encoder_get(2);
}
static void driver_init_spi6(void)
{

}
static void driver_work_spi6(driver_descriptor_t *p_driver)
{
    if(p_driver->state != DRIVER_BUSY)
    {
//        spi6_read_write(encoder_cmd, 6);
        
        p_driver->state = DRIVER_BUSY;
    }
    *p_driver->device->data.in = encoder_get(4);
}
static void driver_init_qspi(void)
{
//    qspi_init();
}
static void driver_work_qspi(driver_descriptor_t *p_driver)
{
    
}
/**
* @struct    driver
* @brief     驱动节点
* @attention
*/
driver_descriptor_t driver_list[] = 
{
{
    .id       = DRIVER_ADC_EXT,
    .init_fcn = driver_init_adc,
    .work_fcn = driver_work_adc,
},
{
    .id       = DRIVER_SPI1,
    .init_fcn = driver_init_spi1,
    .work_fcn = driver_work_spi1,
},
{
    .id       = DRIVER_SPI3,
    .init_fcn = driver_init_spi3,
    .work_fcn = driver_work_spi3,
},
{
    .id       = DRIVER_SPI4,
    .init_fcn = driver_init_spi4,
    .work_fcn = driver_work_spi4,
},
{
    .id       = DRIVER_SPI6,
    .init_fcn = driver_init_spi6,
    .work_fcn = driver_work_spi6,
},
{
    .id       = DRIVER_QSPI,
    .init_fcn = driver_init_qspi,
    .work_fcn = driver_work_qspi,
},
};
static void driver_run_init(void)
{
    driver_descriptor_t *head = &driver_head;
    
    while(head->next != NULL)
    {
        if(head->next->init_fcn != NULL)
        {
            head->next->init_fcn();
        }
        head = head->next;
    }
}
static void driver_ready(void)
{
    driver_descriptor_t *head = &driver_head;
    
    while(head->next != NULL)
    {
        head->next->state = DRIVER_READY;
        head = head->next;
    }
}
static void mount_driver(void)
{
    device_descriptor_t *head = &device_head;
    
    while(head->next != NULL)
    {
        if(head->next->ch <= CH_ADC_EX7)
        {
            head->next->driver = &driver_list[0];
        }
        switch(head->next->ch)
        {
            case CH_SPI1: head->next->driver = &driver_list[1]; break;
            case CH_SPI3: head->next->driver = &driver_list[2]; break;
            case CH_SPI4: head->next->driver = &driver_list[3]; break;
            case CH_SPI6: head->next->driver = &driver_list[4]; break;
            case CH_QSPI: head->next->driver = &driver_list[5]; break;
            default: break;
        }
        head = head->next;
    }
}
static void regist_driver(driver_descriptor_t *p_new)
{
    p_new->next = driver_head.next;
    driver_head.next = p_new;
}
/**
* @brief 初始化驱动
*/
void driver_init(void)
{
    uint16_t index = 0;

    for(index=0; index<sizeof(driver_list) / sizeof(driver_list[0]); index++)
    {
        regist_driver(&driver_list[index]);
    }
    
    mount_driver();
    driver_run_init();
}
/**
* @brief 运行驱动
*/
void driver_run(void)
{
    driver_ready();
}
