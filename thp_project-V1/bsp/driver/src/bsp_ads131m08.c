/**@file   bsp_ads131m08.c
* @brief   ads131m08驱动,spi频率25MHz
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_ads131m08.h"
#include "bsp_cfg.h"

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
* @brief  ads131m08 GPIO初始化
* @attention 
*/
void Ads131IOInit(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_StructInit(&GPIO_InitStruct);
    /* 使能时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);  //使能GPIO的时钟
    /* IO配置 */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0;  //引脚号
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;      //模式
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //输出模式
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //上下拉电阻
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO口驱动电路响应速度
    GPIO_Init(GPIOB, &GPIO_InitStruct);  //初始化
    
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_5;  //引脚号
    GPIO_Init(GPIOC, &GPIO_InitStruct);  //初始化
    
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_4;   //引脚号
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;  //模式
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;  //上拉
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO口驱动电路响应速度
    GPIO_Init(GPIOC, &GPIO_InitStruct);  //初始化
    
    /* 启动 */
    AD_SYNC_UP;
    AD_CS_UP;
}
/**
* @brief  ads131m08初始化
* @attention 
*/
void Ads131Init(void)
{
    Ads131IOInit();
}
