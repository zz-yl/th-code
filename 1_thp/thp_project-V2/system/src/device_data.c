/**@file   device_data.c
* @brief   �豸ģ�����ݽṹ
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "device_data.h"
#include "thp_cfg.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

device_descriptor_t device_head;  //�豸����ͷ
float filter_now = FILTER_NOW;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief     ͨ������ת��
* @param[io] p_d:�豸������
*/
void dev_conversion(device_descriptor_t *p_d)
{
    float data = 0;
    float fdata = 0;
    
    if(p_d->data.in == NULL)
    {
        return;
    }
    data = interp_1d((*p_d->data.in - p_d->data.in_offset), p_d->data.list_x, p_d->data.list_y, p_d->data.list_size, LIMIT_UEN);
    data = limit(data, p_d->data.out_max, p_d->data.out_min);  //������
    //�˲�
    fdata = interp_1d((*p_d->data.in_filter - p_d->data.in_offset), p_d->data.list_x, p_d->data.list_y, p_d->data.list_size, LIMIT_UEN);
    fdata = limit(fdata, p_d->data.out_max, p_d->data.out_min);  //������
    
    *p_d->data.out = data;
    *p_d->data.out_filter = fdata;
}
/**
* @brief     ����������ת��
* @param[io] p_d:�豸������
*/
void dev_conversion_encoder(device_descriptor_t *p_d)
{
    if(p_d->data.in == NULL)
    {
        return;
    }

    *p_d->data.out = *p_d->data.in;
    *p_d->data.out_filter = *p_d->data.in_filter;
}
/**
* @brief ��ͨ�˲�
* @attention 
*/
void dev_filter_low_pass(device_descriptor_t *device_p)
{
    *device_p->data.in_filter = filter_low_pass(*device_p->data.in_filter, *device_p->data.in, filter_now);
}
/**
* @brief ��ֵ���С��ʼ��
* @attention 
*/
void dev_list_size_init(device_descriptor_t *device_p)
{
    device_p->data.list_size = table_max[device_type];
}
