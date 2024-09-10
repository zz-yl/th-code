/**@file   device_register.c
* @brief   �豸ע��
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
* @brief     �������豸�ڵ�
* @attention
*/
device_descriptor_t device_sensor[] = 
{
/* ��λ��1
 * ����ֵ��Χ:0mm~90mm */
{
    .id             = DEV_POT1,
    .ch             = CH_ADC_EX2,
    .data.list_x    = device_list.x_pos1,
    .data.list_y    = device_list.y_pos,
    .data.list_size = sizeof(device_list.x_pos1) / sizeof(device_list.x_pos1[0]),
    .data.out_min   = 0,
    .data.out_max   = 160,
},
/* ��λ��2
 * ����ֵ��Χ:0mm~90mm */
{
    .id             = DEV_POT2,
    .ch             = CH_ADC_EX1,
    .data.list_x    = device_list.x_pos2,
    .data.list_y    = device_list.y_pos,
    .data.list_size = sizeof(device_list.x_pos2) / sizeof(device_list.x_pos2[0]),
    .data.out_min   = 0,
    .data.out_max   = 160,
},
/* ��λ��3
 * ����ֵ��Χ:0mm~90mm */
{
    .id             = DEV_POT3,
    .ch             = CH_ADC_EX3,
    .data.list_x    = device_list.x_pos3,
    .data.list_y    = device_list.y_pos,
    .data.list_size = sizeof(device_list.x_pos3) / sizeof(device_list.x_pos3[0]),
    .data.out_min   = 0,
    .data.out_max   = 160,
},
/* ��λ��4
 * ����ֵ��Χ:0mm~90mm */
{
    .id             = DEV_POT4,
    .ch             = CH_ADC_EX0,
    .data.list_x    = device_list.x_pos4,
    .data.list_y    = device_list.y_pos,
    .data.list_size = sizeof(device_list.x_pos4) / sizeof(device_list.x_pos4[0]),
    .data.out_min   = 0,
    .data.out_max   = 160,
},
/* ��λ��5
 * ����ֵ��Χ:0mm~90mm */
{
    .id             = DEV_POT5,
    .ch             = CH_ADC_EX4,
    .data.list_x    = device_list.x_pos5,
    .data.list_y    = device_list.y_pos,
    .data.list_size = sizeof(device_list.x_pos5) / sizeof(device_list.x_pos5[0]),
    .data.out_min   = 0,
    .data.out_max   = 160,
},
/* ������1
 * ����ֵ��Χ: */
{
    .id             = DEV_ENCODER1,
    .ch             = CH_SPI1,
    .data.out_min   = 0,
    .data.out_max   = 160,
    .work_fcn       = dev_conversion_encoder,
},
/* ������2
 * ����ֵ��Χ: */
{
    .id             = DEV_ENCODER2,
    .ch             = CH_SPI4,
    .data.out_min   = 0,
    .data.out_max   = 160,
    .work_fcn       = dev_conversion_encoder,
},
/* ������3
 * ����ֵ��Χ: */
{
    .id             = DEV_ENCODER3,
    .ch             = CH_SPI3,
    .data.out_min   = 0,
    .data.out_max   = 160,
    .work_fcn       = dev_conversion_encoder,
},
/* ������4
 * ����ֵ��Χ: */
{
    .id             = DEV_ENCODER4,
    .ch             = CH_SPI6,
    .data.out_min   = 0,
    .data.out_max   = 160,
    .work_fcn       = dev_conversion_encoder,
},
/* ������5
 * ����ֵ��Χ: */
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
* @brief     �豸ע��
* @param[io] p_new:�豸������
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
* @brief �豸����
* @attention 
*/
void device_mount(void)
{
    uint16_t index = 0;

    for(index=0; index<sizeof(device_sensor) / sizeof(device_sensor[0]); index++)
    {
        device_regist(&device_sensor[index]);
    }
    /* ���ݵ�ַ��� */
    device_init_data();
}
