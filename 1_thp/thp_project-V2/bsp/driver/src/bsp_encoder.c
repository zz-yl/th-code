/**@file   bsp_encoder.c
* @brief   编码器驱动
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_encoder.h"
#include "bsp_cfg.h"

#include "test.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/


/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @union   encoder_rx_t
* @brief   编码器接收数据
*/
typedef union
{
    uint8_t data[4];
    struct
    {
        uint32_t null           : 8;   //无
        
        uint32_t na1            : 3;   //保留
        uint32_t over_speed     : 1;   //1:超速
        uint32_t na2            : 4;   //保留
        uint32_t pc             : 1;   //奇偶校验
        uint32_t no_mag_warning : 1;   //1:磁场太弱
        uint32_t angle          : 14;  //转动角度数据(14位)
    }bits;
}encoder_rx_t;
/**
* @struct  encoder_data_t
* @brief   编码器数据结构
*/
typedef struct
{
    encoder_rx_t rx;
    float pos;       //实际位置(mm)
    float pos_last;  //上次读到的位置(mm)
    float pos_init;  //初始位置(mm)
    
    float pos_sum[2]; //实际位置缓存
    uint16_t sum_cnt[2]; //缓存次数
    uint8_t sum_flag; //两组缓存切换,0:第一组;1:第二组
}encoder_data_t;
/**
* @struct  encoder_t
* @brief   编码器数据
*/
typedef struct
{
    encoder_data_t e1;
    encoder_data_t e2;
    encoder_data_t e3;
    encoder_data_t e4;
    encoder_data_t e5;
}encoder_t;

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

encoder_t encoder_data;

//uint8_t encoder_cmd[] = {0x83, 0x00, 0x84, 0x00, 0x85, 0x00};
uint8_t encoder_cmd[] = {0x83, 0x00, 0x00, 0x00};

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  编码器清零
* @attention 
*/
void encoder_clear(void)
{
    encoder_data.e1.pos = 0;
    encoder_data.e2.pos = 0;
    encoder_data.e3.pos = 0;
    encoder_data.e4.pos = 0;
    encoder_data.e5.pos = 0;
}
/**
* @brief  发送编码器查询指令
* @attention 
*/
void encoder_call(void)
{
    spi1_read_write(encoder_cmd, 4);
    spi4_read_write(encoder_cmd, 4);
    spi3_read_write(encoder_cmd, 4);
    spi6_read_write(encoder_cmd, 4);
}
/**
* @brief  编码器获取近期采集的平均值
* @attention 
*/
float encoder_get(uint8_t id)
{
    uint8_t flag = 0;
    encoder_data_t *enc = NULL;
    float data = 0;
    
    switch(id)
    {
        case 1: enc = &encoder_data.e1; break;
        case 2: enc = &encoder_data.e2; break;
        case 3: enc = &encoder_data.e3; break;
        case 4: enc = &encoder_data.e4; break;
        default: return 0;
    }
    
    flag = enc->sum_flag;
    if(enc->sum_cnt[flag] == 0)
    {
        return 0;
    }
    enc->sum_flag = !enc->sum_flag;

    data = enc->pos_sum[flag] / enc->sum_cnt[flag];
    enc->pos_sum[flag] = 0;
    enc->sum_cnt[flag] = 0;
    
    return data;
}
static void encoder_rx(encoder_data_t *enc, uint8_t *data)
{
    float pos_now;
    float pos_err;
    
    enc->rx.data[3] = data[1];
    enc->rx.data[2] = data[2];
    enc->rx.data[1] = data[3];
    
    pos_now = -((float)enc->rx.bits.angle) / 0x3FFF;
    pos_err = pos_now - enc->pos_last;
    if(pos_err > 0.5f)  //反转套圈
    {
        enc->pos += pos_err - 1;
    }
    else if(pos_err < -0.5f)  //正转套圈
    {
        enc->pos += pos_err + 1;
    }
    else
    {
        enc->pos += pos_err;
    }
    enc->pos_last = pos_now;
    
    enc->pos_sum[enc->sum_flag] += enc->pos;
    enc->sum_cnt[enc->sum_flag]++;
}
void spi3_receive(uint8_t *data, uint16_t len)
{
    if(len != 4)
    {
        return;
    }
    encoder_rx(&encoder_data.e1, data);
}
void spi4_receive(uint8_t *data, uint16_t len)
{
    if(len != 4)
    {
        return;
    }
    encoder_rx(&encoder_data.e2, data);
}
void spi6_receive(uint8_t *data, uint16_t len)
{
    if(len != 4)
    {
        return;
    }
    encoder_rx(&encoder_data.e3, data);
}
void spi1_receive(uint8_t *data, uint16_t len)
{
    if(len != 4)
    {
        return;
    }
    encoder_rx(&encoder_data.e4, data);
    
//    test_print_fill(encoder_data.e1.pos_last);
//    test_print_fill(((uint16_t)data[1] << 6) + ((uint16_t)data[3] >> 2));
//    test_print();
}
/**
* @brief  编码器初始化
* @attention 
*/
void enc_init(void)
{
    
}

