/**@file   device_register.c
* @brief   设备注册
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "device_register.h"
#include "device_core.h"
#include "device_io.h"
#include "interp_table.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

/**
* @struct    device_sensor 
* @brief     传感器设备节点
* @attention
*/
device_descriptor_t device_sensor[] = 
{
/* 电位计1
 * 物理值范围:0mm~90mm */
{
    .id             = DEV_POT1,
    .ch             = CH_ADC_EX2,
    .data.list_x    = device_list.x_pos1,
    .data.list_y    = device_list.y_pos,
    .data.list_size = sizeof(device_list.x_pos1) / sizeof(device_list.x_pos1[0]),
    .data.out_min   = 0,
    .data.out_max   = 160,
},
/* 电位计2
 * 物理值范围:0mm~90mm */
{
    .id             = DEV_POT2,
    .ch             = CH_ADC_EX1,
    .data.list_x    = device_list.x_pos2,
    .data.list_y    = device_list.y_pos,
    .data.list_size = sizeof(device_list.x_pos2) / sizeof(device_list.x_pos2[0]),
    .data.out_min   = 0,
    .data.out_max   = 160,
},
/* 电位计3
 * 物理值范围:0mm~90mm */
{
    .id             = DEV_POT3,
    .ch             = CH_ADC_EX3,
    .data.list_x    = device_list.x_pos3,
    .data.list_y    = device_list.y_pos,
    .data.list_size = sizeof(device_list.x_pos3) / sizeof(device_list.x_pos3[0]),
    .data.out_min   = 0,
    .data.out_max   = 160,
},
/* 电位计4
 * 物理值范围:0mm~90mm */
{
    .id             = DEV_POT4,
    .ch             = CH_ADC_EX0,
    .data.list_x    = device_list.x_pos4,
    .data.list_y    = device_list.y_pos,
    .data.list_size = sizeof(device_list.x_pos4) / sizeof(device_list.x_pos4[0]),
    .data.out_min   = 0,
    .data.out_max   = 160,
},
/* 电位计5
 * 物理值范围:0mm~90mm */
{
    .id             = DEV_POT5,
    .ch             = CH_ADC_EX4,
    .data.list_x    = device_list.x_pos5,
    .data.list_y    = device_list.y_pos,
    .data.list_size = sizeof(device_list.x_pos5) / sizeof(device_list.x_pos5[0]),
    .data.out_min   = 0,
    .data.out_max   = 160,
},
/* 编码器1
 * 物理值范围: */
{
    .id             = DEV_ENCODER1,
    .ch             = CH_SPI1,
    .data.out_min   = 0,
    .data.out_max   = 160,
    .work_fcn       = dev_conversion_encoder,
},
/* 编码器2
 * 物理值范围: */
{
    .id             = DEV_ENCODER2,
    .ch             = CH_SPI4,
    .data.out_min   = 0,
    .data.out_max   = 160,
    .work_fcn       = dev_conversion_encoder,
},
/* 编码器3
 * 物理值范围: */
{
    .id             = DEV_ENCODER3,
    .ch             = CH_SPI3,
    .data.out_min   = 0,
    .data.out_max   = 160,
    .work_fcn       = dev_conversion_encoder,
},
/* 编码器4
 * 物理值范围: */
{
    .id             = DEV_ENCODER4,
    .ch             = CH_SPI6,
    .data.out_min   = 0,
    .data.out_max   = 160,
    .work_fcn       = dev_conversion_encoder,
},
/* 编码器5
 * 物理值范围: */
{
    .id             = DEV_ENCODER5,
    .ch             = CH_QSPI,
    .data.out_min   = 0,
    .data.out_max   = 160,
    .work_fcn       = dev_conversion_encoder,
},
};

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief     设备注册
* @param[io] p_new:设备描述符
*/
static void device_regist(device_descriptor_t *p_new)
{
    p_new->next = device_head.next;
    device_head.next = p_new;

    if((p_new->init_fcn == NULL) && (p_new->data.list_x != NULL))
    {
        p_new->init_fcn = dev_list_size_init;
    }
    if((p_new->work_fcn == NULL) && (p_new->data.list_x != NULL))
    {
        p_new->work_fcn = dev_conversion;
    }
    if(p_new->filter_fcn == NULL)
    {
        p_new->filter_fcn = dev_filter_low_pass;
    }
}
/**
* @brief 设备挂载
* @attention 
*/
void device_mount(void)
{
    uint16_t index = 0;

    for(index=0; index<sizeof(device_sensor) / sizeof(device_sensor[0]); index++)
    {
        device_regist(&device_sensor[index]);
    }
    /* 数据地址填充 */
    device_init_data();
}
