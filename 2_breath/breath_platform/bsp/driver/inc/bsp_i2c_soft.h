/**@file   bsp_i2c_soft.h
* @brief   软件I2C
* @author  陈卓哲
* @date    2024/3/28
* @section 参考文档
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef BSP_I2C_SOFT_H_
#define BSP_I2C_SOFT_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_sys.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define IIC_ACK     0
#define IIC_NACK    1
#define IIC_WRITE   0x00
#define IIC_READ    0x01

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct io_para_t
* @brief  IO口参数
*/
typedef struct
{
    GPIO_TypeDef* io_x;
    uint32_t io_clk;
    uint32_t pin;
}io_para_t;
/**
* @struct i2cs_t
* @brief  软件I2C句柄
*/
typedef struct
{
    io_para_t scl;
    io_para_t sda;
}i2cs_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern i2cs_t i2cs1;
extern i2cs_t i2cs2;
extern i2cs_t i2cs3;
extern i2cs_t i2cs4;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void i2c_start(i2cs_t i2c);
void i2c_stop(i2cs_t i2c);
uint8_t i2c_write_byte(uint8_t tx_byte, i2cs_t i2c);
uint8_t i2c_read_byte(uint8_t ack, i2cs_t i2c);
void i2cs_write(i2cs_t i2c, uint8_t *buf, uint16_t len, uint8_t addr_dev);
void i2cs_read(i2cs_t i2c, uint8_t *buf, uint16_t len, uint8_t addr_dev);

void bsp_i2cs_init(void);

#endif /* BSP_I2C_SOFT_H_ */
