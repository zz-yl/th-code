/**@file   bsp_spi.c
* @brief   硬件SPI驱动,DMA,中断,
*          ADC芯片的SPI,磁编码器SPI
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_spi.h"
#include "bsp_cfg.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define SPI_DMA_SIZE  10
#define SPI2_DMA_SIZE  100

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  spi_handle_t
* @brief   SPI句柄
*/
typedef struct
{
    SPI_HandleTypeDef spi;
    DMA_HandleTypeDef dma_tx;
    DMA_HandleTypeDef dma_rx;
}spi_handle_t;
/**
* @struct  spi_dma_buf_t
* @brief   spi dma缓存
*/
typedef struct
{
    uint8_t s2_tx[SPI2_DMA_SIZE];
    uint8_t s2_rx[SPI2_DMA_SIZE];
}spi_dma_buf_t;

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

spi_handle_t  spi2_handle = {0};  //SPI句柄
spi_dma_buf_t spi_dma_buf;  //SPI,DMA缓存

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/*spi2*************************************************************************************************/
/* spi2初始化 */
void spi2_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_SPI_DeInit(&spi2_handle.spi);
    /* 时钟使能 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();
    /* IO配置 */
    /**SPI2 GPIO Configuration
    PB12     ------> SPI2_NSS
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    gpio_init_struct.Pin       = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //模式
    gpio_init_struct.Pull      = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF5_SPI2;           //复用
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);  //初始化
    /* DMA配置 */
    /* Tx DMA配置 */
    spi2_handle.dma_tx.Instance                 = DMA1_Channel3;            //数据流选择
    spi2_handle.dma_tx.Init.Request             = DMA_REQUEST_1;            //请求设置，设置是哪个外设请求的
    spi2_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //传输方向
    spi2_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //外设（非）增量模式
    spi2_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //存储器（非）增量模式
    spi2_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //外设数据大小： 8/16/32 位
    spi2_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //存储器数据大小： 8/16/32 位
    spi2_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //模式：外设流控模式/循环模式/普通模式
    spi2_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA 优先级：低/中/高/非常高
    HAL_DMA_DeInit(&spi2_handle.dma_tx);  //取消初始化
    HAL_DMA_Init(&spi2_handle.dma_tx);    //初始化
    __HAL_LINKDMA(&spi2_handle.spi, hdmatx, spi2_handle.dma_tx);  //将DMA与SPI联系起来
    /* Rx DMA配置 */
    spi2_handle.dma_rx = spi2_handle.dma_tx;  //其它设置相同
    spi2_handle.dma_rx.Instance                 = DMA1_Channel2;            //数据流选择
    spi2_handle.dma_rx.Init.Request             = DMA_REQUEST_1;            //请求设置，设置是哪个外设请求的
    spi2_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //传输方向
    HAL_DMA_DeInit(&spi2_handle.dma_rx);  //取消初始化
    HAL_DMA_Init(&spi2_handle.dma_rx);    //初始化
    __HAL_LINKDMA(&spi2_handle.spi, hdmarx, spi2_handle.dma_rx);  //将DMA与SPI联系起来
    /* SPI配置 */
    spi2_handle.spi.Instance                        = SPI2;                                     //寄存器基地址
    spi2_handle.spi.Init.Mode                       = SPI_MODE_MASTER;                          //模式
    spi2_handle.spi.Init.Direction                  = SPI_DIRECTION_2LINES;                     //SPI双向模式状态
    spi2_handle.spi.Init.DataSize                   = SPI_DATASIZE_8BIT;                        //数据大小
    spi2_handle.spi.Init.CLKPolarity                = SPI_POLARITY_LOW;                         //时钟空闲状态
    spi2_handle.spi.Init.CLKPhase                   = SPI_PHASE_2EDGE;                          //第几个上升沿
    spi2_handle.spi.Init.NSS                        = SPI_NSS_HARD_OUTPUT;                      //硬件(NSS引脚)或软件使用SSI位
    spi2_handle.spi.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_16;                 //波特率预算器值分频
    spi2_handle.spi.Init.FirstBit                   = SPI_FIRSTBIT_MSB;                         //数据传输是从MSB位还是LSB位开始
    spi2_handle.spi.Init.TIMode                     = SPI_TIMODE_DISABLE;                       //是否启用TI模式
    spi2_handle.spi.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;               //是否启用CRC计算
    spi2_handle.spi.Init.CRCPolynomial              = 0x0UL;                                    //CRC计算的多项式
    spi2_handle.spi.Init.CRCLength                  = SPI_CRC_LENGTH_DATASIZE;                  //CRC长度
    spi2_handle.spi.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;                     //是否启用NSSP信号
    HAL_SPI_Init(&spi2_handle.spi);  //初始化
    /* DMA中断配置 */
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);          //使能中断
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);          //使能中断
}
/**
* @brief SPI DMA发送中断
*/
void DMA1_Stream3_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = spi2_handle.spi.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //调用HAL库中断处理公用函数
}
/**
* @brief SPI DMA接收中断
*/
void DMA1_Stream2_IRQHandler(void)
{
    uint32_t len = 0;
    SPI_HandleTypeDef *handle = &spi2_handle.spi;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
        
        HAL_SPI_Abort(handle);  //停止DMA传送
        len = handle->RxXferSize - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        spi2_receive(handle->pRxBuffPtr, len);  //数据输出
        
//        //读DR避免频繁进中断
//        len = SPI2->RXDR;
//        len = SPI2->SR;
    }
    else if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx)) != RESET)  //半传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
    }
    else if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输错误中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
    }
    else  /* 半传输完成中断处理中可能会产生传输完成中断,所以先不处理其它中断 */
    {
//        HAL_DMA_IRQHandler(handle->hdmarx);  //调用HAL库中断处理公用函数
    }
}
/**
* @brief  SPI 等待空闲
* @attention
*/
void spi2_wait(void)
{
    while(spi2_handle.spi.State != HAL_SPI_STATE_READY);
}
/**
* @brief  SPI 读写数据
* @attention
*/
uint16_t spi2_read_write(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_SPI_GET_FLAG(&spi2_handle.spi, SPI_FLAG_TXE) != SET)  //上次发送未完成
    {
        return 0;
    }

    if(len > sizeof(spi_dma_buf.s2_tx))  //长度超出DMA缓存区
    {
        len = sizeof(spi_dma_buf.s2_tx);
    }
    i = len;
    while(i--)
    {
        if(data != NULL)
        {
            spi_dma_buf.s2_tx[i] = data[i];
        }
        else
        {
            spi_dma_buf.s2_tx[i] = 0;
        }
    }

    HAL_SPI_TransmitReceive_DMA(&spi2_handle.spi, spi_dma_buf.s2_tx, spi_dma_buf.s2_rx, len);  //txdma启动，开启各dma中断
    
    return len;
}
/**
* @brief  SPI 读写数据
* @attention
*/
uint8_t spi2_read_write_byte(uint8_t data)
{
    if(__HAL_SPI_GET_FLAG(&spi2_handle.spi, SPI_FLAG_TXE) != SET)  //上次发送未完成
    {
        return 0;
    }
    
    spi_dma_buf.s2_tx[0] = data;

    HAL_SPI_TransmitReceive_DMA(&spi2_handle.spi, spi_dma_buf.s2_tx, spi_dma_buf.s2_rx, 1);  //txdma启动，开启各dma中断
    spi2_wait();
    
    return spi_dma_buf.s2_rx[0];
}
/*spi2================================================================================================*/
