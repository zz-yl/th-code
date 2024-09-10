/**@file   bsp_io.c
* @brief   IO口驱动
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_io.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/


/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/


/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  LED小灯
* @attention 
*/
void bsp_led_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};

    /* IO配置 */
//    gpio_init_struct.Pin   = GPIO_PIN_5;        //引脚号
//    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;     //模式
//    gpio_init_struct.Pull  = GPIO_PULLUP;             //上下拉电阻
//    gpio_init_struct.Speed = GPIO_SPEED_FREQ_MEDIUM;  //IO口驱动电路响应速度
//    HAL_GPIO_Init(GPIOD, &gpio_init_struct);  //初始化
    gpio_init_struct.Pin   = GPIO_PIN_3;        //引脚号
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;     //模式
    gpio_init_struct.Pull  = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);  //初始化
    /* 启动 */
    IO_LED_RUN(GPIO_PIN_RESET);
}
/**
* @brief  输出IO口
* @attention 
*/
void bsp_output_io_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};

    /* IO配置 */
    gpio_init_struct.Pin   = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8; //引脚号
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;     //模式
    gpio_init_struct.Pull  = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    HAL_GPIO_Init(GPIOF, &gpio_init_struct);  //初始化
    /* 启动 */
}
/**
* @brief  输入IO口
* @attention 
*/
void bsp_input_io_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};

    /* IO配置 */
    gpio_init_struct.Pin   = GPIO_PIN_6;              //引脚号
    gpio_init_struct.Mode  = GPIO_MODE_INPUT;         //模式
    gpio_init_struct.Pull  = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);  //初始化
    gpio_init_struct.Pin   = GPIO_PIN_9;              //引脚号
    HAL_GPIO_Init(GPIOF, &gpio_init_struct);  //初始化
}
/**
* @brief  IO口控制初始化
* @attention 
*/
void bsp_io_init(void)
{
    /* 时钟使能 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    /* IO初始化 */
//    bsp_led_init();
    bsp_output_io_init();
    bsp_input_io_init();
}
