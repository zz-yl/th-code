/**@file   bsp_sfm3300.c
* @brief   sfm3300传感器驱动
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_sfm3300.h"
#include "bsp_i2c.h"
#include "bsp_i2c_soft.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

struct
{
    uint8_t cmd[2];
    uint8_t s1_data[SFM3300_DATA_LEN];
    uint8_t s2_data[SFM3300_DATA_LEN];
    uint8_t s3_data[SFM3300_DATA_LEN];
    uint8_t s4_data[SFM3300_DATA_LEN];
}sfm3300_data = 
{
    .cmd = {0x10, 0x00},  //质量流量
};

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  sfm3300传感器启动
*/
void bsp_sfm3300_start(sfm3300_id_t id)
{
    switch(id)
    {
//        case SFM_S1: i2c3_write(sfm3300_data.cmd, sizeof(sfm3300_data.cmd), ADDR_SFM3300); break;
//        case SFM_S2: i2c2_write(sfm3300_data.cmd, sizeof(sfm3300_data.cmd), ADDR_SFM3300); break;
        case SFM_S1: i2cs_write(i2cs1, sfm3300_data.cmd, sizeof(sfm3300_data.cmd), ADDR_SFM3300); break;
        case SFM_S2: i2cs_write(i2cs2, sfm3300_data.cmd, sizeof(sfm3300_data.cmd), ADDR_SFM3300); break;
        case SFM_S3: i2cs_write(i2cs3, sfm3300_data.cmd, sizeof(sfm3300_data.cmd), ADDR_SFM3300); break;
        case SFM_S4: i2cs_write(i2cs4, sfm3300_data.cmd, sizeof(sfm3300_data.cmd), ADDR_SFM3300); break;
        default: break;
    }
}
/**
* @brief  sfm3300传感器读数据
*/
void bsp_sfm3300_read(sfm3300_id_t id)
{
    switch(id)
    {
//        case SFM_S1: i2c3_read(sfm3300_data.s1_data, SFM3300_DATA_LEN, ADDR_SFM3300); break;
//        case SFM_S2: i2c2_read(sfm3300_data.s2_data, SFM3300_DATA_LEN, ADDR_SFM3300); break;
        case SFM_S1: i2cs_read(i2cs1, sfm3300_data.s1_data, SFM3300_DATA_LEN, ADDR_SFM3300); break;
        case SFM_S2: i2cs_read(i2cs2, sfm3300_data.s2_data, SFM3300_DATA_LEN, ADDR_SFM3300); break;
        case SFM_S3: i2cs_read(i2cs3, sfm3300_data.s3_data, SFM3300_DATA_LEN, ADDR_SFM3300); break;
        case SFM_S4: i2cs_read(i2cs4, sfm3300_data.s4_data, SFM3300_DATA_LEN, ADDR_SFM3300); break;
        default: break;
    }
}
/**
* @brief  sfm3300传感器获取压差
*/
float bsp_sfm3300_get_flow(sfm3300_id_t id)
{
    float press = 0;
    int16_t buf = 0;
    
    switch(id)
    {
        case SFM_S1: buf = (int16_t)(((uint16_t)sfm3300_data.s1_data[0] << 8) + sfm3300_data.s1_data[1]); break;
        case SFM_S2: buf = (int16_t)(((uint16_t)sfm3300_data.s2_data[0] << 8) + sfm3300_data.s2_data[1]); break;
        case SFM_S3: buf = (int16_t)(((uint16_t)sfm3300_data.s3_data[0] << 8) + sfm3300_data.s3_data[1]); break;
        case SFM_S4: buf = (int16_t)(((uint16_t)sfm3300_data.s4_data[0] << 8) + sfm3300_data.s4_data[1]); break;
        default: break;
    }
    
    buf -= SFM3300_OFFSET;
    press = (float)buf / SFM3300_SCALE;
    
    return press;
}
/**
* @brief  sfm3300传感器获取温度
*/
float bsp_sfm3300_get_temp(sfm3300_id_t id)
{
    float temp = 0;
    
    switch(id)
    {
        case SFM_S1: temp = (int16_t)(((uint16_t)sfm3300_data.s1_data[3] << 8) + sfm3300_data.s1_data[4]); break;
        case SFM_S2: temp = (int16_t)(((uint16_t)sfm3300_data.s2_data[3] << 8) + sfm3300_data.s2_data[4]); break;
        case SFM_S3: temp = (int16_t)(((uint16_t)sfm3300_data.s3_data[3] << 8) + sfm3300_data.s3_data[4]); break;
        case SFM_S4: temp = (int16_t)(((uint16_t)sfm3300_data.s4_data[3] << 8) + sfm3300_data.s4_data[4]); break;
        default: break;
    }
    
    temp /= SFM3300_TEMP_SCALE;
    
    return temp;
}
