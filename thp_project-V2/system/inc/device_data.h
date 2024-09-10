/**@file   device_data.h
* @brief   设备模块数据结构
* @author  陈卓哲
* @date    2023/11/10
* @section 参考文档
* 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef DEVICE_DATA_H_
#define DEVICE_DATA_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "stdint.h"
#include "stdlib.h"
#include "data_algorithm.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/* 一阶滤波配置 */
#define TAST_TIMS_DEV  1  ///< 任务执行周期
#define PI  3.1415926535897932384626433832795
#define FILTER_CUT_OFF_FREQ  5  ///< 截止频率,Hz
#define FILTER_NOW           FILTER_CUT_OFF_FREQ * 2.0 * PI * (TAST_TIMS_DEV / 1000.0)  ///< 滤波实时值权重

/**
* @enum    driver_state_t
* @brief   驱动状态
*/
typedef enum
{
    DRIVER_READY,      ///< 就绪
    DRIVER_BUSY,       ///< 忙
    DRIVER_ERR,        ///< 错误
}driver_state_t;
/**
* @enum    driver_id_t
* @brief   驱动ID
*/
typedef enum
{
    DRIVER_NULL,      ///< 无
    
    DRIVER_ADC_EXT,   ///< 外部AD
    DRIVER_SPI1,      ///< SPI1
    DRIVER_SPI3,      ///< SPI3
    DRIVER_SPI4,      ///< SPI4
    DRIVER_SPI6,      ///< SPI6
    DRIVER_QSPI,      ///< QSPI
    DRIVER_PULSE1,    ///< PULSE1
    DRIVER_PULSE2,    ///< PULSE2
    DRIVER_PULSE3,    ///< PULSE3
    DRIVER_PULSE4,    ///< PULSE4
    DRIVER_PULSE5,    ///< PULSE5
    
    DRIVER_UNDEFINED, ///< 无
}driver_id_t;
/**
* @enum  dev_ret_t
* @brief 函数返回值
*/
typedef enum
{
    DEV_OK,
    DEV_ERR,
    DEV_BUSY,
}dev_ret_t;
/**
* @enum  dev_channel_id_t
* @brief 设备数据通道编号
*/
typedef enum
{
    CH_ADC_EX0,
    CH_ADC_EX1,
    CH_ADC_EX2,
    CH_ADC_EX3,
    CH_ADC_EX4,
    CH_ADC_EX5,
    CH_ADC_EX6,
    CH_ADC_EX7,
    CH_SPI1,
    CH_SPI3,
    CH_SPI4,
    CH_SPI6,
    CH_QSPI,
    
    CH_UNDEFINED, ///< 无
}dev_channel_id_t;
/**
* @enum    device_id_t
* @brief   设备ID
*/
typedef enum
{
    DEV_NULL,   ///< 无
    /* 电位器mm */
    DEV_POT1,   ///< 电位器1
    DEV_POT2,   ///< 电位器2
    DEV_POT3,   ///< 电位器3
    DEV_POT4,   ///< 电位器4
    DEV_POT5,   ///< 电位器5
    /* 编码器mm */
    DEV_ENCODER1,  ///< 编码器1
    DEV_ENCODER2,  ///< 编码器2
    DEV_ENCODER3,  ///< 编码器3
    DEV_ENCODER4,  ///< 编码器4
    DEV_ENCODER5,  ///< 编码器5
    
    DEV_UNDEFINED, ///< 无
}device_id_t;

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  driver_descriptor_t
* @brief   驱动描述符
*/
typedef struct driver_node
{
    driver_id_t    id;     ///< 驱动ID
    void           *data;  ///< 数据表
    uint16_t       len;    ///< 数据长度
    driver_state_t state;  ///< 驱动运行状态
    void (*init_fcn)(void);  ///< 初始化函数
    void (*work_fcn)(struct driver_node *);  ///< 工作函数
    struct device_node *device; ///< 调用设备

    struct driver_node *next;   ///< 链表下一个节点
}driver_descriptor_t;

/**
* @struct  device_data_t
* @brief   设备数据
*/
typedef struct
{
    float        *in;          ///< 输入值,如:电压值v
    float        *out;         ///< 输出值,如:传感器实际数值
    float        *in_filter;   ///< 输入值滤波
    float        *out_filter;  ///< 输出值滤波
    const float  out_min;      ///< 输出值下限
    const float  out_max;      ///< 输出值上限
    float        in_offset;    ///< 输入值偏移量
    float        *list_x;      ///< 输入参数表
    float        *list_y;      ///< 输出参数表
    uint16_t     list_size;    ///< 参数表长度
    queue_list_t *data_f;      ///< 滤波数据
}device_data_t;
/**
* @struct  device_descriptor_t
* @brief   设备描述符
*/
typedef struct device_node
{
    device_id_t      id;    ///< 设备ID
    dev_channel_id_t ch;    ///< 通道编号
    device_data_t    data;  ///< 设备数据
    void (*init_fcn)(struct device_node *);    ///< 初始化函数
    void (*work_fcn)(struct device_node *);    ///< 工作函数
    void (*filter_fcn)(struct device_node *);  ///< 滤波函数
    void (*comp_fcn)(struct device_node * , float , float);  ///< 补偿函数
    driver_descriptor_t *driver; ///< 设备对应驱动

    struct device_node *next;  ///< 链表下一个节点
}device_descriptor_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern device_descriptor_t device_head;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void dev_conversion(device_descriptor_t *p_d);
void dev_filter_low_pass(device_descriptor_t *device_p);
void dev_conversion_encoder(device_descriptor_t *p_d);
void dev_list_size_init(device_descriptor_t *device_p);

#endif /* DEVICE_DATA_H_ */
