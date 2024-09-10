/**@file   device_data.h
* @brief   �豸ģ�����ݽṹ
* @author  ��׿��
* @date    2023/11/10
* @section �ο��ĵ�
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

/* һ���˲����� */
#define TAST_TIMS_DEV  1  ///< ����ִ������
#define PI  3.1415926535897932384626433832795
#define FILTER_CUT_OFF_FREQ  5  ///< ��ֹƵ��,Hz
#define FILTER_NOW           FILTER_CUT_OFF_FREQ * 2.0 * PI * (TAST_TIMS_DEV / 1000.0)  ///< �˲�ʵʱֵȨ��

/**
* @enum    driver_state_t
* @brief   ����״̬
*/
typedef enum
{
    DRIVER_READY,      ///< ����
    DRIVER_BUSY,       ///< æ
    DRIVER_ERR,        ///< ����
}driver_state_t;
/**
* @enum    driver_id_t
* @brief   ����ID
*/
typedef enum
{
    DRIVER_NULL,      ///< ��
    
    DRIVER_ADC_EXT,   ///< �ⲿAD
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
    
    DRIVER_UNDEFINED, ///< ��
}driver_id_t;
/**
* @enum  dev_ret_t
* @brief ��������ֵ
*/
typedef enum
{
    DEV_OK,
    DEV_ERR,
    DEV_BUSY,
}dev_ret_t;
/**
* @enum  dev_channel_id_t
* @brief �豸����ͨ�����
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
    
    CH_UNDEFINED, ///< ��
}dev_channel_id_t;
/**
* @enum    device_id_t
* @brief   �豸ID
*/
typedef enum
{
    DEV_NULL,   ///< ��
    /* ��λ��mm */
    DEV_POT1,   ///< ��λ��1
    DEV_POT2,   ///< ��λ��2
    DEV_POT3,   ///< ��λ��3
    DEV_POT4,   ///< ��λ��4
    DEV_POT5,   ///< ��λ��5
    /* ������mm */
    DEV_ENCODER1,  ///< ������1
    DEV_ENCODER2,  ///< ������2
    DEV_ENCODER3,  ///< ������3
    DEV_ENCODER4,  ///< ������4
    DEV_ENCODER5,  ///< ������5
    
    DEV_UNDEFINED, ///< ��
}device_id_t;

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  driver_descriptor_t
* @brief   ����������
*/
typedef struct driver_node
{
    driver_id_t    id;     ///< ����ID
    void           *data;  ///< ���ݱ�
    uint16_t       len;    ///< ���ݳ���
    driver_state_t state;  ///< ��������״̬
    void (*init_fcn)(void);  ///< ��ʼ������
    void (*work_fcn)(struct driver_node *);  ///< ��������
    struct device_node *device; ///< �����豸

    struct driver_node *next;   ///< ������һ���ڵ�
}driver_descriptor_t;

/**
* @struct  device_data_t
* @brief   �豸����
*/
typedef struct
{
    float        *in;          ///< ����ֵ,��:��ѹֵv
    float        *out;         ///< ���ֵ,��:������ʵ����ֵ
    float        *in_filter;   ///< ����ֵ�˲�
    float        *out_filter;  ///< ���ֵ�˲�
    const float  out_min;      ///< ���ֵ����
    const float  out_max;      ///< ���ֵ����
    float        in_offset;    ///< ����ֵƫ����
    float        *list_x;      ///< ���������
    float        *list_y;      ///< ���������
    uint16_t     list_size;    ///< ��������
    queue_list_t *data_f;      ///< �˲�����
}device_data_t;
/**
* @struct  device_descriptor_t
* @brief   �豸������
*/
typedef struct device_node
{
    device_id_t      id;    ///< �豸ID
    dev_channel_id_t ch;    ///< ͨ�����
    device_data_t    data;  ///< �豸����
    void (*init_fcn)(struct device_node *);    ///< ��ʼ������
    void (*work_fcn)(struct device_node *);    ///< ��������
    void (*filter_fcn)(struct device_node *);  ///< �˲�����
    void (*comp_fcn)(struct device_node * , float , float);  ///< ��������
    driver_descriptor_t *driver; ///< �豸��Ӧ����

    struct device_node *next;  ///< ������һ���ڵ�
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
