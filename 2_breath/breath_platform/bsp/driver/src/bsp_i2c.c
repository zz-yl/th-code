/**@file   bsp_i2c.c
* @brief   硬件I2C驱动,中断
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_i2c.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

I2C_HandleTypeDef i2c1_handle;  //i2c句柄
I2C_HandleTypeDef i2c2_handle;  //i2c句柄
I2C_HandleTypeDef i2c3_handle;  //i2c句柄

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/*I2C1================================================================================================*/
/**
* @brief  i2c等待空闲
*/
void i2c1_wait(void)
{
    while(i2c1_handle.State != HAL_I2C_STATE_READY);
}
/**
* @brief  i2c写
*/
void i2c1_write(uint8_t *buf, uint16_t len, uint8_t addr_dev)
{
    HAL_I2C_Master_Transmit_IT(&i2c1_handle, addr_dev, buf, len);
}
/**
* @brief  i2c读
*/
void i2c1_read(uint8_t *buf, uint16_t len, uint8_t addr_dev)
{
    HAL_I2C_Master_Receive_IT(&i2c1_handle, addr_dev, buf, len);
}
/**
* @brief  i2c初始化
*/
void i2c1_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_I2C_DeInit(&i2c1_handle);
    /* 时钟使能 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    /* IO配置 */
    gpio_init_struct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;           //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_OD;      //模式,!!!此处必须开漏才能正常通信!!!
    gpio_init_struct.Pull      = GPIO_PULLUP;//GPIO_PULLUP;GPIO_NOPULL          //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;  //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF4_I2C1;        //复用
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);  //初始化
    /* i2c配置 */
    i2c1_handle.Instance              = I2C1;                     //寄存器地址
    i2c1_handle.Init.Timing           = 0x00702991;//0x00300EAC;//0x00702991;//0x00B03FDB;               //波特率
    i2c1_handle.Init.OwnAddress1      = 0;                        //自己的第一个设备地址
    i2c1_handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;  //选择7位或10位寻址模式
    i2c1_handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;  //是否选择双寻址模式
    i2c1_handle.Init.OwnAddress2      = 0;                        //自己的第二个设备地址(双寻址模式)
    i2c1_handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;           //当选择双寻址方式时，确认掩码地址为自身第二设备地址
    i2c1_handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;  //是否使用广播
    i2c1_handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;    //指定是否选择nostretch模式
    HAL_I2C_Init(&i2c1_handle);  //初始化
    
    HAL_I2CEx_ConfigAnalogFilter(&i2c1_handle, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&i2c1_handle, 0);
    
    /* i2c中断配置 */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);          //使能中断
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);          //使能中断
}
/* I2C事件中断 */
void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2c1_handle);
}
/* I2C错误中断 */
void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2c1_handle);
}
/*I2C1*************************************************************************************************/
/*I2C2================================================================================================*/
/**
* @brief  i2c等待空闲
*/
void i2c2_wait(void)
{
    while(i2c2_handle.State != HAL_I2C_STATE_READY);
}
/**
* @brief  i2c写
*/
void i2c2_write(uint8_t *buf, uint16_t len, uint8_t addr_dev)
{
    HAL_I2C_Master_Transmit_IT(&i2c2_handle, addr_dev, buf, len);
}
/**
* @brief  i2c读
*/
void i2c2_read(uint8_t *buf, uint16_t len, uint8_t addr_dev)
{
    HAL_I2C_Master_Receive_IT(&i2c2_handle, addr_dev, buf, len);
}
/**
* @brief  i2c初始化
*/
void i2c2_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_I2C_DeInit(&i2c2_handle);
    /* 时钟使能 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    /* IO配置 */
    gpio_init_struct.Pin       = GPIO_PIN_10 | GPIO_PIN_11; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_OD;      //模式,!!!此处必须开漏才能正常通信!!!
    gpio_init_struct.Pull      = GPIO_PULLUP;          //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;  //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF4_I2C2;        //复用
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);  //初始化
    /* i2c配置 */
    i2c2_handle.Instance              = I2C2;                     //寄存器地址
    i2c2_handle.Init.Timing           = 0x00702991;               //波特率
    i2c2_handle.Init.OwnAddress1      = 0;                        //自己的第一个设备地址
    i2c2_handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;  //选择7位或10位寻址模式
    i2c2_handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;  //是否选择双寻址模式
    i2c2_handle.Init.OwnAddress2      = 0;                        //自己的第二个设备地址(双寻址模式)
    i2c2_handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;           //当选择双寻址方式时，确认掩码地址为自身第二设备地址
    i2c2_handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;  //是否使用广播
    i2c2_handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;    //指定是否选择nostretch模式
    HAL_I2C_Init(&i2c2_handle);  //初始化
    /* i2c中断配置 */
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 1, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);          //使能中断
    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 1, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);          //使能中断
}
/* I2C事件中断 */
void I2C2_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2c2_handle);
}
/* I2C错误中断 */
void I2C2_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2c2_handle);
}
/*I2C2*************************************************************************************************/
/*I2C3================================================================================================*/
/**
* @brief  i2c写
*/
void i2c3_write(uint8_t *buf, uint16_t len, uint8_t addr_dev)
{
    HAL_I2C_Master_Transmit_IT(&i2c3_handle, addr_dev, buf, len);
}
/**
* @brief  i2c读
*/
void i2c3_read(uint8_t *buf, uint16_t len, uint8_t addr_dev)
{
    HAL_I2C_Master_Receive_IT(&i2c3_handle, addr_dev, buf, len);
}
/**
* @brief  i2c初始化
*/
void i2c3_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_I2C_DeInit(&i2c3_handle);
    /* 时钟使能 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_I2C3_CLK_ENABLE();
    /* IO配置 */
    gpio_init_struct.Pin       = GPIO_PIN_0 | GPIO_PIN_1; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_OD;      //模式,!!!此处必须开漏才能正常通信!!!
    gpio_init_struct.Pull      = GPIO_PULLUP;          //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;  //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF4_I2C3;        //复用
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);  //初始化
    /* i2c配置 */
    i2c3_handle.Instance              = I2C3;                     //寄存器地址
    i2c3_handle.Init.Timing           = 0x00702991;               //波特率
    i2c3_handle.Init.OwnAddress1      = 0;                        //自己的第一个设备地址
    i2c3_handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;  //选择7位或10位寻址模式
    i2c3_handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;  //是否选择双寻址模式
    i2c3_handle.Init.OwnAddress2      = 0;                        //自己的第二个设备地址(双寻址模式)
    i2c3_handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;           //当选择双寻址方式时，确认掩码地址为自身第二设备地址
    i2c3_handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;  //是否使用广播
    i2c3_handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;    //指定是否选择nostretch模式
    HAL_I2C_Init(&i2c3_handle);  //初始化
    /* i2c中断配置 */
    HAL_NVIC_SetPriority(I2C3_EV_IRQn, 1, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);          //使能中断
    HAL_NVIC_SetPriority(I2C3_ER_IRQn, 1, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);          //使能中断
}
/* I2C事件中断 */
void I2C3_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2c3_handle);
}
/* I2C错误中断 */
void I2C3_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2c3_handle);
}
/*I2C3*************************************************************************************************/

