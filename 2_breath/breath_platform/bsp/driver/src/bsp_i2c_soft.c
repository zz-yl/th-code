/**@file   bsp_i2c_soft.c
* @brief   软件I2C
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

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

i2cs_t i2cs1 =
{
    .scl.io_x   = GPIOC,
    .scl.pin    = GPIO_PIN_0,
    .sda.io_x   = GPIOC,
    .sda.pin    = GPIO_PIN_1,
};
i2cs_t i2cs2 =
{
    .scl.io_x   = GPIOB,
    .scl.pin    = GPIO_PIN_10,
    .sda.io_x   = GPIOB,
    .sda.pin    = GPIO_PIN_11,
};
i2cs_t i2cs3 =
{
    .scl.io_x   = GPIOB,
    .scl.pin    = GPIO_PIN_12,
    .sda.io_x   = GPIOB,
    .sda.pin    = GPIO_PIN_13,
};
i2cs_t i2cs4 =
{
    .scl.io_x   = GPIOB,
    .scl.pin    = GPIO_PIN_14,
    .sda.io_x   = GPIOB,
    .sda.pin    = GPIO_PIN_15,
};


/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

static void scl_h(i2cs_t i2c)
{
    HAL_GPIO_WritePin(i2c.scl.io_x, i2c.scl.pin, GPIO_PIN_SET);
}
static void scl_l(i2cs_t i2c)
{
    HAL_GPIO_WritePin(i2c.scl.io_x, i2c.scl.pin, GPIO_PIN_RESET);
}
static void sda_h(i2cs_t i2c)
{
    HAL_GPIO_WritePin(i2c.sda.io_x, i2c.sda.pin, GPIO_PIN_SET);
}
static void sda_l(i2cs_t i2c)
{
    HAL_GPIO_WritePin(i2c.sda.io_x, i2c.sda.pin, GPIO_PIN_RESET);
}
static uint8_t sda_read(i2cs_t i2c)
{
    return HAL_GPIO_ReadPin(i2c.sda.io_x, i2c.sda.pin);
}
static void delay_us(uint16_t us)
{
    if(us == 0)
    {
        return;
    }
    us *= 30;
    while(us--)
    {
        
    }
}
/**
* @brief  软件I2C IO初始化
* @attention 
*/
static void i2cs_init(i2cs_t i2c)
{
    if(i2c.scl.io_x == 0)
    {
        return;
    }
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};

    /* IO配置 */
    gpio_init_struct.Pin   = i2c.scl.pin;            //引脚号
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_OD;    //模式
    gpio_init_struct.Pull  = GPIO_PULLUP;//GPIO_PULLUP;GPIO_NOPULL  //上下拉电阻
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;   //IO口驱动电路响应速度
    HAL_GPIO_Init(i2c.scl.io_x, &gpio_init_struct);  //初始化
    
    gpio_init_struct.Pin   = i2c.sda.pin;            //引脚号
    HAL_GPIO_Init(i2c.sda.io_x, &gpio_init_struct);  //初始化
    /* 启动 */
    sda_h(i2c);
    scl_h(i2c);
}
static void sda_out(i2cs_t i2c)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};

    /* IO配置 */
    gpio_init_struct.Pin   = i2c.sda.pin;            //引脚号
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_OD;    //模式
    gpio_init_struct.Pull  = GPIO_PULLUP;            //上下拉电阻
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;   //IO口驱动电路响应速度
    HAL_GPIO_Init(i2c.sda.io_x, &gpio_init_struct);  //初始化
}
static void sda_in(i2cs_t i2c)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};

    /* IO配置 */
    gpio_init_struct.Pin   = i2c.sda.pin;             //引脚号
    gpio_init_struct.Mode  = GPIO_MODE_INPUT;         //模式
    gpio_init_struct.Pull  = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    HAL_GPIO_Init(i2c.sda.io_x, &gpio_init_struct);   //初始化
}
/**
* @brief  I2C start信号
* @attention 
*/
void i2cs_start(i2cs_t i2c)
{
    sda_out(i2c);    //sda输出

    sda_h(i2c);
    scl_h(i2c);
    delay_us(1);
    sda_l(i2c);
    delay_us(1);
    scl_l(i2c);
}
/**
* @brief  I2C stop信号
* @attention 
*/
void i2cs_stop(i2cs_t i2c)
{
    sda_out(i2c);    //sda输出

    scl_l(i2c);
    sda_l(i2c);
    delay_us(1);
    scl_h(i2c);
    delay_us(1);
    sda_h(i2c);
    delay_us(1);
}
/**
* @brief  I2C 写一个字节
* @attention 
*/
uint8_t i2cs_write_byte(uint8_t tx_byte, i2cs_t i2c)
{	
    uint8_t mask = 0;
    uint8_t ack = IIC_ACK;

    sda_out(i2c);
    scl_l(i2c);
    delay_us(1);
    for(mask=0x80; mask>0; mask>>=1)   //shift bit for masking (8 times)
    { 
        if ((mask & tx_byte) == 0) 
        {
            sda_l(i2c);
        }
        else
        {
            sda_h(i2c);
        }
        delay_us(1);
        scl_h(i2c);
        delay_us(1);
        scl_l(i2c);
        delay_us(2);
    }

//    sda_l(i2c);
    sda_in(i2c);
    scl_h(i2c);  //clk #9 for ack
    
    if(sda_read(i2c) == IIC_NACK)
        ack = IIC_NACK;
    sda_out(i2c);
    delay_us(1);
    scl_l(i2c);

    return ack;
}
/**
* @brief  I2C 读一个字节
* @attention 
*/
uint8_t i2cs_read_byte(uint8_t ack, i2cs_t i2c)
{
    uint8_t mask = 0;
    uint8_t rx_byte = 0;

    sda_in(i2c);
    scl_l(i2c);
    delay_us(1);
    for(mask=0x80; mask>0; mask>>=1)
    { 
        scl_h(i2c);
        if(sda_read(i2c)==1) rx_byte=(rx_byte | mask); //read bit
        delay_us(1);
        scl_l(i2c);
        delay_us(1);
    }
    //ACK
    sda_out(i2c);
    if(ack == IIC_NACK)
    {
        sda_h(i2c);
    }
    else
    {
        sda_l(i2c);
    }

    delay_us(1);
    scl_h(i2c);
    delay_us(1);
    scl_l(i2c);

    return rx_byte;
}
/**
* @brief  软件i2c写
*/
void i2cs_write(i2cs_t i2c, uint8_t *buf, uint16_t len, uint8_t addr_dev)
{
    uint16_t index = 0;
    
    i2cs_start(i2c);
    if(i2cs_write_byte(addr_dev, i2c) != IIC_ACK)
    {
        i2cs_stop(i2c);
        return;
    }
    
    for(; index<len; index++)
    {
        if(i2cs_write_byte(buf[index], i2c))
        {
            i2cs_stop(i2c);
            return;
        }
    }
    
    i2cs_stop(i2c);
}
/**
* @brief  软件i2c读
*/
void i2cs_read(i2cs_t i2c, uint8_t *buf, uint16_t len, uint8_t addr_dev)
{
    uint16_t index = 0;
    
    i2cs_start(i2c);
    if(i2cs_write_byte((addr_dev | IIC_READ), i2c) != IIC_ACK)
    {
        i2cs_stop(i2c);
        return;
    }
    
    for(; index<len-1; index++)
    {
        buf[index] = i2cs_read_byte(IIC_ACK, i2c);
    }
    buf[index] = i2cs_read_byte(IIC_NACK, i2c);
    
    i2cs_stop(i2c);
}
/**
* @brief  软件I2C初始化
* @attention 
*/
void bsp_i2cs_init(void)
{
    i2cs_init(i2cs1);
    i2cs_init(i2cs2);
    i2cs_init(i2cs3);
    i2cs_init(i2cs4);
}
