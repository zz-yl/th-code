/**@file   device_io.c
* @brief   设备IO模块
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "device_io.h"
#include "device_register.h"
#include "device_core.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/


/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

device_io_t in_data;
device_io_t out_data;
device_io_t in_filter_data;
device_io_t out_filter_data;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  设备数据接口初始化
* @attention 
*/
void device_init_data(void)
{
    int16_t i = 0;
    device_descriptor_t *head = &device_head;
    float *in = &in_data.struct_end;
    float *out = &out_data.struct_end;
    float *in_f = &in_filter_data.struct_end;
    float *out_f = &out_filter_data.struct_end;
    
    while((head->next != NULL) && (&in[i] != (float*)&in_data))
    {
        i--;
        head->next->data.in         = &in[i];
        head->next->data.out        = &out[i];
        head->next->data.in_filter  = &in_f[i];
        head->next->data.out_filter = &out_f[i];
        head = head->next;
    }
}

const device_io_t *dev_read_in(void)
{
    return &in_data;
}
const device_io_t *dev_read_out(void)
{
    return &out_data;
}
const device_io_t *dev_read_in_filter(void)
{
    return &in_filter_data;
}
const device_io_t *dev_read_out_filter(void)
{
    return &out_filter_data;
}
