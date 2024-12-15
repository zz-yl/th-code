/**@file   memory.h
* @brief   存储模块
* @author  陈卓哲
* @date    2023/10/8
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef MEMORY_H_
#define MEMORY_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_cfg.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define MEM_E2_PAGE_SIZE  128  //每页字节数

#define MEM_LEN_DATA_ADDR (MEM_ADDR_DATA_LEN - MEM_ADDR_DATA_ADDR - 2)  //地址缓存大小
#define MEM_LEN_DATA_LEN (MEM_ADDR_DATA - MEM_ADDR_DATA_LEN - 2)  //长度缓存大小
#define MEM_LEN_DATA   5000              //数据缓存大小
#define MEM_SIZE_DATA (0x7FFF - 0x4000)  //总空间大小

/**
* @enum    mem_addr_t
* @brief   数据存储地址,E2地址范围:0x0000~0xFFFF,共64kByte,共512页,每页128Byte
*/
typedef enum
{
    MEM_ADDR_TEST       = 0x00,  //E2测试地址
    MEM_ADDR_TYPE       = 0x02,  //装置类型
    MEM_ADDR_SN         = 0x05,  //sn号
    MEM_ADDR_DATA_SIZE  = 0x20,  //上位机数据长度
    MEM_ADDR_POT        = 0x80,  //电位器校准数据,600*4字节
    
    MEM_ADDR_DATA_ADDR  = 0x3000,  //上位机预留存储数据地址,0x3000~0x37FF,共2048字节,长度数据为2字节,预留crc校验位置则能存储1023组数据,ID即为地址数据排序位置
    MEM_ADDR_DATA_LEN   = 0x3800,  //上位机预留存储数据长度,0x3800~0x3FFF,共2048字节,长度数据为2字节,预留crc校验位置则能存储1023组数据,ID即为长度数据排序位置
    MEM_ADDR_DATA       = 0x4000,  //上位机预留存储数据空间,0x4000~0x7FFF,共16384字节
}mem_addr_t;

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct mem_data_t
* @brief  上位机存储数据
*/
typedef struct
{
    uint16_t addr[(MEM_LEN_DATA_ADDR+2)/2];  //地址表
    uint16_t len[(MEM_LEN_DATA_LEN+2)/2];    //长度表
    uint8_t  buf[MEM_LEN_DATA];              //数据缓存
    uint32_t size;                           //剩余空间大小
}mem_data_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern uint8_t mem_buf[20];
extern uint32_t mem_buf_pot[601];
extern mem_data_t mem_data;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void mem_write_sn(uint8_t *data, uint8_t len);
void mem_init(void);
void mem_e2prom_write(uint8_t *buf, uint16_t len, uint16_t addr_data);
uint16_t mem_e2prom_read(uint8_t *buf, uint16_t len, uint16_t addr_data);

#endif /* MEMORY_H_ */
