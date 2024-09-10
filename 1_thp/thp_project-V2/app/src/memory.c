/**@file   memory.c
* @brief   存储模块
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "memory.h"

#include "FreeRTOS.h"
#include "task.h"
#include "thp_cfg.h"
#include "interp_table.h"
#include "crc_check.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

uint8_t mem_buf[20];
uint32_t mem_buf_pot[601];

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  e2写数据
* @attention 
*/
static void mem_e2prom_write_page(uint8_t *p_tx, uint16_t len, uint16_t addr_data)
{
    while(len > MEM_E2_BYTE_MAX)
    {
        i2c3_write(p_tx, MEM_E2_BYTE_MAX, addr_data);
        len -= MEM_E2_BYTE_MAX;
        p_tx += MEM_E2_BYTE_MAX;
        addr_data += MEM_E2_BYTE_MAX;
        vTaskDelay(9);
    }
    i2c3_write(p_tx, len, addr_data);
    vTaskDelay(9);
}
/**
* @brief  e2写数据
* @attention 
*/
void mem_e2prom_write(uint8_t *buf, uint16_t len, uint16_t addr_data)
{
    uint8_t crc8 = 0;
    
    crc8 = crc_get_crc8(buf, len, 0xFF);
    buf[len++] = crc8;
    mem_e2prom_write_page(buf, len, addr_data);
}
/**
* @brief  e2读数据
* @attention 
*/
uint16_t mem_e2prom_read(uint8_t *buf, uint16_t len, uint16_t addr_data)
{
    uint8_t crc8 = 0;
    
    i2c3_read(buf, len+1, addr_data);
    i2c3_wait();
    crc8 = crc_get_crc8(buf, len, 0xFF);

    if(crc8 != buf[len])
    {
        return 0;
    }
    
    return len;
}
/**
* @brief  写sn
* @attention 
*/
void mem_write_sn(uint8_t *data, uint8_t len)
{
    uint8_t i = 0;
    uint8_t buf[32] = {0};
    
    for(i=0; i<15; i++)
    {
        buf[i] = thp_sn[i];
    }
    mem_e2prom_write(buf, 15, MEM_ADDR_SN);
}
/**
* @brief  存储初始化
* @attention 
*/
void mem_init(void)
{
    uint16_t i = 0;
    
    /* 装置类型 */
    if(mem_e2prom_read(mem_buf, 1, MEM_ADDR_TYPE))
    {
        device_type = (thp_device_type_t)mem_buf[0];
    }
    if(device_type > SYS_LENGTH_150)  //避免无效数据
    {
        device_type = SYS_LENGTH_60;
    }
    /* 电位器校准数据 */
    if(mem_e2prom_read((uint8_t *)mem_buf_pot, 2400, MEM_ADDR_POT))
    {
        for(i=0; i<150; i++)
        {
            device_list.x_pos1[i] = mem_buf_pot[i];
            device_list.x_pos2[i] = mem_buf_pot[150+i];
            device_list.x_pos3[i] = mem_buf_pot[300+i];
            device_list.x_pos4[i] = mem_buf_pot[450+i];
        }
    }
    switch(device_type)
    {
        case SYS_LENGTH_60:
            for(i=0; i<table_max[device_type]; i++)
            {
                device_list.y_pos[i] = device_list.y_pos_60[i];
            }
            break;
        case SYS_LENGTH_100:
            for(i=0; i<table_max[device_type]; i++)
            {
                device_list.y_pos[i] = device_list.y_pos_100[i];
            }
            break;
        default: break;
    }
    /* sn码 */
    if(mem_e2prom_read(mem_buf, 15, MEM_ADDR_SN))
    {
        for(i=0; i<15; i++)
        {
            thp_sn[i] = mem_buf[i];
        }
    }
}
