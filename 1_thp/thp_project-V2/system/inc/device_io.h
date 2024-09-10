/**@file   device_io.h
* @brief   设备IO模块
* @author  陈卓哲
* @date    2023/11/10
* @section
*
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef DEVICE_IO_H_
#define DEVICE_IO_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "device_data.h"
#include "driver.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  device_io_t
* @brief   设备IO数据
*/
typedef struct
{
    float pot1;       ///< 电位器1
    float pot2;       ///< 电位器2
    float pot3;       ///< 电位器3
    float pot4;       ///< 电位器4
    float pot5;       ///< 电位器5
    float encoder1;   ///< 编码器1
    float encoder2;   ///< 编码器2
    float encoder3;   ///< 编码器3
    float encoder4;   ///< 编码器4
    float encoder5;   ///< 编码器5
    
    float struct_end; ///< 结尾
}device_io_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern device_io_t in_data;
extern device_io_t out_data;
extern device_io_t in_filter_data;
extern device_io_t out_filter_data; 

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

const device_io_t *dev_read_in(void);
const device_io_t *dev_read_out(void);
const device_io_t *dev_read_in_filter(void);
const device_io_t *dev_read_out_filter(void);

void device_init_data(void);

#endif /* DEVICE_IO_H_ */
