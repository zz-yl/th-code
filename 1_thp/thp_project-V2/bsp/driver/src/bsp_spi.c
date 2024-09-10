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
    uint8_t s1_tx[SPI_DMA_SIZE];
    uint8_t s1_rx[SPI_DMA_SIZE];
    uint8_t s2_tx[SPI2_DMA_SIZE];
    uint8_t s2_rx[SPI2_DMA_SIZE];
    uint8_t s3_tx[SPI_DMA_SIZE];
    uint8_t s3_rx[SPI_DMA_SIZE];
    uint8_t s4_tx[SPI_DMA_SIZE];
    uint8_t s4_rx[SPI_DMA_SIZE];
    uint8_t s5_tx[SPI_DMA_SIZE];
    uint8_t s5_rx[SPI_DMA_SIZE];
    uint8_t s6_tx[SPI_DMA_SIZE];
    uint8_t s6_rx[SPI_DMA_SIZE];
}spi_dma_buf_t;

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

spi_handle_t  spi1_handle = {0};  //SPI句柄
spi_handle_t  spi2_handle = {0};  //SPI句柄
spi_handle_t  spi3_handle = {0};  //SPI句柄
spi_handle_t  spi4_handle = {0};  //SPI句柄
spi_handle_t  spi5_handle = {0};  //SPI句柄
spi_handle_t  spi6_handle = {0};  //SPI句柄
spi_dma_buf_t spi_dma_buf __attribute__((at(0x38000000))) = {0} ;  //SPI,DMA缓存

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/*spi1*************************************************************************************************/
/* spi1初始化 */
void spi1_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_SPI_DeInit(&spi1_handle.spi);
    /* 时钟使能 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    /* IO配置 */
    /**SPI1 GPIO Configuration
    PA4     ------> SPI1_NSS
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    gpio_init_struct.Pin       = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //模式
    gpio_init_struct.Pull      = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF5_SPI1;           //复用
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);  //初始化
    /* DMA配置 */
    /* Tx DMA配置 */
    spi1_handle.dma_tx.Instance                 = DMA1_Stream0;             //数据流选择
    spi1_handle.dma_tx.Init.Request             = DMA_REQUEST_SPI1_TX;      //请求设置，设置是哪个外设请求的
    spi1_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //传输方向
    spi1_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //外设（非）增量模式
    spi1_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //存储器（非）增量模式
    spi1_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //外设数据大小： 8/16/32 位
    spi1_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //存储器数据大小： 8/16/32 位
    spi1_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //模式：外设流控模式/循环模式/普通模式
    spi1_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA 优先级：低/中/高/非常高
    spi1_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO 模式开启或者禁止
    spi1_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO 阈值选择
    spi1_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //存储器突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    spi1_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //外设突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    HAL_DMA_DeInit(&spi1_handle.dma_tx);  //取消初始化
    HAL_DMA_Init(&spi1_handle.dma_tx);    //初始化
    __HAL_LINKDMA(&spi1_handle.spi, hdmatx, spi1_handle.dma_tx);  //将DMA与SPI联系起来
    /* Rx DMA配置 */
    spi1_handle.dma_rx = spi1_handle.dma_tx;  //其它设置相同
    spi1_handle.dma_rx.Instance                 = DMA1_Stream1;             //数据流选择
    spi1_handle.dma_rx.Init.Request             = DMA_REQUEST_SPI1_RX;      //请求设置，设置是哪个外设请求的
    spi1_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //传输方向
    HAL_DMA_DeInit(&spi1_handle.dma_rx);  //取消初始化
    HAL_DMA_Init(&spi1_handle.dma_rx);    //初始化
    __HAL_LINKDMA(&spi1_handle.spi, hdmarx, spi1_handle.dma_rx);  //将DMA与SPI联系起来
    /* SPI配置 */
    spi1_handle.spi.Instance                        = SPI1;                                     //寄存器基地址
    spi1_handle.spi.Init.Mode                       = SPI_MODE_MASTER;                          //模式
    spi1_handle.spi.Init.Direction                  = SPI_DIRECTION_2LINES;                     //SPI双向模式状态
    spi1_handle.spi.Init.DataSize                   = SPI_DATASIZE_8BIT;                        //数据大小
    spi1_handle.spi.Init.CLKPolarity                = SPI_POLARITY_HIGH;                        //时钟空闲状态
    spi1_handle.spi.Init.CLKPhase                   = SPI_PHASE_2EDGE;                          //第几个上升沿
    spi1_handle.spi.Init.NSS                        = SPI_NSS_HARD_OUTPUT;                      //硬件(NSS引脚)或软件使用SSI位
    spi1_handle.spi.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_32;                 //波特率预算器值分频
    spi1_handle.spi.Init.FirstBit                   = SPI_FIRSTBIT_MSB;                         //数据传输是从MSB位还是LSB位开始
    spi1_handle.spi.Init.TIMode                     = SPI_TIMODE_DISABLE;                       //是否启用TI模式
    spi1_handle.spi.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;               //是否启用CRC计算
    spi1_handle.spi.Init.CRCPolynomial              = 0x0UL;                                    //CRC计算的多项式
    spi1_handle.spi.Init.CRCLength                  = SPI_CRC_LENGTH_DATASIZE;                  //CRC长度
    spi1_handle.spi.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;                     //是否启用NSSP信号
    spi1_handle.spi.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;                     //指定SS输入/输出外部信号有效电平
    spi1_handle.spi.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;                //FIFO阈值级别
    spi1_handle.spi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC初始化模式
    spi1_handle.spi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC初始化模式
    spi1_handle.spi.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;           //指定额外延迟
    spi1_handle.spi.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;    //指定最小时间延迟
    spi1_handle.spi.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;           //在主接收模式下控制连续SPI传输并自动管理，以避免超限的情况
    spi1_handle.spi.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;         //备用功能gpio状态控制
    spi1_handle.spi.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;                      //反转MISO/MOSI
    HAL_SPI_Init(&spi1_handle.spi);  //初始化
    /* DMA中断配置 */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);          //使能中断
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);          //使能中断
}
/**
* @brief SPI DMA发送中断
*/
void DMA1_Stream0_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = spi1_handle.spi.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //调用HAL库中断处理公用函数
}
/**
* @brief SPI DMA接收中断
*/
void DMA1_Stream1_IRQHandler(void)
{
    uint32_t len = 0;
    SPI_HandleTypeDef *handle = &spi1_handle.spi;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
        
        HAL_SPI_Abort(handle);  //停止DMA传送
        len = handle->RxXferSize - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        spi1_receive(handle->pRxBuffPtr, len);  //数据输出
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx)) != RESET)  //半传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输错误中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
    }
}
/**
* @brief  SPI 读写数据
* @attention
*/
uint16_t spi1_read_write(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_SPI_GET_FLAG(&spi1_handle.spi, SPI_SR_TXP) != SET)  //上次发送未完成
    {
        return 0;
    }

    if(len > sizeof(spi_dma_buf.s1_tx))  //长度超出DMA缓存区
    {
        len = sizeof(spi_dma_buf.s1_tx);
    }
    i = len;
    while(i--)
    {
        spi_dma_buf.s1_tx[i] = data[i];
    }

    HAL_SPI_TransmitReceive_DMA(&spi1_handle.spi, spi_dma_buf.s1_tx, spi_dma_buf.s1_rx, len);  //txdma启动，开启各dma中断
    
    return len;
}
/*spi1================================================================================================*/
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
    spi2_handle.dma_tx.Instance                 = DMA1_Stream2;             //数据流选择
    spi2_handle.dma_tx.Init.Request             = DMA_REQUEST_SPI2_TX;      //请求设置，设置是哪个外设请求的
    spi2_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //传输方向
    spi2_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //外设（非）增量模式
    spi2_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //存储器（非）增量模式
    spi2_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //外设数据大小： 8/16/32 位
    spi2_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //存储器数据大小： 8/16/32 位
    spi2_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //模式：外设流控模式/循环模式/普通模式
    spi2_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA 优先级：低/中/高/非常高
    spi2_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO 模式开启或者禁止
    spi2_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO 阈值选择
    spi2_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //存储器突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    spi2_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //外设突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    HAL_DMA_DeInit(&spi2_handle.dma_tx);  //取消初始化
    HAL_DMA_Init(&spi2_handle.dma_tx);    //初始化
    __HAL_LINKDMA(&spi2_handle.spi, hdmatx, spi2_handle.dma_tx);  //将DMA与SPI联系起来
    /* Rx DMA配置 */
    spi2_handle.dma_rx = spi2_handle.dma_tx;  //其它设置相同
    spi2_handle.dma_rx.Instance                 = DMA1_Stream3;             //数据流选择
    spi2_handle.dma_rx.Init.Request             = DMA_REQUEST_SPI2_RX;      //请求设置，设置是哪个外设请求的
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
    spi2_handle.spi.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;                     //指定SS输入/输出外部信号有效电平
    spi2_handle.spi.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;                //FIFO阈值级别
    spi2_handle.spi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC初始化模式
    spi2_handle.spi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC初始化模式
    spi2_handle.spi.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;           //指定额外延迟
    spi2_handle.spi.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;    //指定最小时间延迟
    spi2_handle.spi.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;           //在主接收模式下控制连续SPI传输并自动管理，以避免超限的情况
    spi2_handle.spi.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;         //备用功能gpio状态控制
    spi2_handle.spi.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;                      //反转MISO/MOSI
    HAL_SPI_Init(&spi2_handle.spi);  //初始化
    /* DMA中断配置 */
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);          //使能中断
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);          //使能中断
}
/**
* @brief SPI DMA发送中断
*/
void DMA1_Stream2_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = spi2_handle.spi.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //调用HAL库中断处理公用函数
}
/**
* @brief SPI DMA接收中断
*/
void DMA1_Stream3_IRQHandler(void)
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
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx)) != RESET)  //半传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输错误中断
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
    if(__HAL_SPI_GET_FLAG(&spi2_handle.spi, SPI_SR_TXP) != SET)  //上次发送未完成
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
    if(__HAL_SPI_GET_FLAG(&spi2_handle.spi, SPI_SR_TXP) != SET)  //上次发送未完成
    {
        return 0;
    }
    
    spi_dma_buf.s2_tx[0] = data;

    HAL_SPI_TransmitReceive_DMA(&spi2_handle.spi, spi_dma_buf.s2_tx, spi_dma_buf.s2_rx, 1);  //txdma启动，开启各dma中断
    spi2_wait();
    
    return spi_dma_buf.s2_rx[0];
}
/*spi2================================================================================================*/
/*spi3*************************************************************************************************/
/* spi3初始化 */
void spi3_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_SPI_DeInit(&spi3_handle.spi);
    /* 时钟使能 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_SPI3_CLK_ENABLE();
    /* IO配置 */
    /**SPI3 GPIO Configuration
    PA15 (JTDI)     ------> SPI3_NSS
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    gpio_init_struct.Pin       = GPIO_PIN_15; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //模式
    gpio_init_struct.Pull      = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF6_SPI3;           //复用
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);  //初始化
    gpio_init_struct.Pin       = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12; //引脚号
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);  //初始化
    /* DMA配置 */
    /* Tx DMA配置 */
    spi3_handle.dma_tx.Instance                 = DMA1_Stream4;             //数据流选择
    spi3_handle.dma_tx.Init.Request             = DMA_REQUEST_SPI3_TX;      //请求设置，设置是哪个外设请求的
    spi3_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //传输方向
    spi3_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //外设（非）增量模式
    spi3_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //存储器（非）增量模式
    spi3_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //外设数据大小： 8/16/32 位
    spi3_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //存储器数据大小： 8/16/32 位
    spi3_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //模式：外设流控模式/循环模式/普通模式
    spi3_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA 优先级：低/中/高/非常高
    spi3_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO 模式开启或者禁止
    spi3_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO 阈值选择
    spi3_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //存储器突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    spi3_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //外设突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    HAL_DMA_DeInit(&spi3_handle.dma_tx);  //取消初始化
    HAL_DMA_Init(&spi3_handle.dma_tx);    //初始化
    __HAL_LINKDMA(&spi3_handle.spi, hdmatx, spi3_handle.dma_tx);  //将DMA与SPI联系起来
    /* Rx DMA配置 */
    spi3_handle.dma_rx = spi3_handle.dma_tx;  //其它设置相同
    spi3_handle.dma_rx.Instance                 = DMA1_Stream5;             //数据流选择
    spi3_handle.dma_rx.Init.Request             = DMA_REQUEST_SPI3_RX;      //请求设置，设置是哪个外设请求的
    spi3_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //传输方向
    HAL_DMA_DeInit(&spi3_handle.dma_rx);  //取消初始化
    HAL_DMA_Init(&spi3_handle.dma_rx);    //初始化
    __HAL_LINKDMA(&spi3_handle.spi, hdmarx, spi3_handle.dma_rx);  //将DMA与SPI联系起来
    /* SPI配置 */
    spi3_handle.spi.Instance                        = SPI3;                                     //寄存器基地址
    spi3_handle.spi.Init.Mode                       = SPI_MODE_MASTER;                          //模式
    spi3_handle.spi.Init.Direction                  = SPI_DIRECTION_2LINES;                     //SPI双向模式状态
    spi3_handle.spi.Init.DataSize                   = SPI_DATASIZE_8BIT;                        //数据大小
    spi3_handle.spi.Init.CLKPolarity                = SPI_POLARITY_HIGH;                        //时钟空闲状态
    spi3_handle.spi.Init.CLKPhase                   = SPI_PHASE_2EDGE;                          //第几个上升沿
    spi3_handle.spi.Init.NSS                        = SPI_NSS_HARD_OUTPUT;                      //硬件(NSS引脚)或软件使用SSI位
    spi3_handle.spi.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_32;                 //波特率预算器值分频
    spi3_handle.spi.Init.FirstBit                   = SPI_FIRSTBIT_MSB;                         //数据传输是从MSB位还是LSB位开始
    spi3_handle.spi.Init.TIMode                     = SPI_TIMODE_DISABLE;                       //是否启用TI模式
    spi3_handle.spi.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;               //是否启用CRC计算
    spi3_handle.spi.Init.CRCPolynomial              = 0x0UL;                                    //CRC计算的多项式
    spi3_handle.spi.Init.CRCLength                  = SPI_CRC_LENGTH_DATASIZE;                  //CRC长度
    spi3_handle.spi.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;                     //是否启用NSSP信号
    spi3_handle.spi.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;                     //指定SS输入/输出外部信号有效电平
    spi3_handle.spi.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;                //FIFO阈值级别
    spi3_handle.spi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC初始化模式
    spi3_handle.spi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC初始化模式
    spi3_handle.spi.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;           //指定额外延迟
    spi3_handle.spi.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;    //指定最小时间延迟
    spi3_handle.spi.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;           //在主接收模式下控制连续SPI传输并自动管理，以避免超限的情况
    spi3_handle.spi.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;         //备用功能gpio状态控制
    spi3_handle.spi.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;                      //反转MISO/MOSI
    HAL_SPI_Init(&spi3_handle.spi);  //初始化
    /* DMA中断配置 */
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);          //使能中断
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);          //使能中断
}
/**
* @brief SPI DMA发送中断
*/
void DMA1_Stream4_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = spi3_handle.spi.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //调用HAL库中断处理公用函数
}
/**
* @brief SPI DMA接收中断
*/
void DMA1_Stream5_IRQHandler(void)
{
    uint32_t len = 0;
    SPI_HandleTypeDef *handle = &spi3_handle.spi;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
        
        HAL_SPI_Abort(handle);  //停止DMA传送
        len = handle->RxXferSize - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        spi3_receive(handle->pRxBuffPtr, len);  //数据输出
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx)) != RESET)  //半传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输错误中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
    }
}
/**
* @brief  SPI 读写数据
* @attention
*/
uint16_t spi3_read_write(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_SPI_GET_FLAG(&spi3_handle.spi, SPI_SR_TXP) != SET)  //上次发送未完成
    {
        return 0;
    }

    if(len > sizeof(spi_dma_buf.s3_tx))  //长度超出DMA缓存区
    {
        len = sizeof(spi_dma_buf.s3_tx);
    }
    i = len;
    while(i--)
    {
        spi_dma_buf.s3_tx[i] = data[i];
    }

    HAL_SPI_TransmitReceive_DMA(&spi3_handle.spi, spi_dma_buf.s3_tx, spi_dma_buf.s3_rx, len);  //txdma启动，开启各dma中断
    
    return len;
}
/*spi3================================================================================================*/
/*spi4*************************************************************************************************/
/* spi4初始化 */
void spi4_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_SPI_DeInit(&spi4_handle.spi);
    /* 时钟使能 */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_SPI4_CLK_ENABLE();
    /* IO配置 */
    /**SPI4 GPIO Configuration
    PE11     ------> SPI4_NSS
    PE12     ------> SPI4_SCK
    PE13     ------> SPI4_MISO
    PE14     ------> SPI4_MOSI
    */
    gpio_init_struct.Pin       = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //模式
    gpio_init_struct.Pull      = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF5_SPI4;           //复用
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);  //初始化
    /* DMA配置 */
    /* Tx DMA配置 */
    spi4_handle.dma_tx.Instance                 = DMA1_Stream6;             //数据流选择
    spi4_handle.dma_tx.Init.Request             = DMA_REQUEST_SPI4_TX;      //请求设置，设置是哪个外设请求的
    spi4_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //传输方向
    spi4_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //外设（非）增量模式
    spi4_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //存储器（非）增量模式
    spi4_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //外设数据大小： 8/16/32 位
    spi4_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //存储器数据大小： 8/16/32 位
    spi4_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //模式：外设流控模式/循环模式/普通模式
    spi4_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA 优先级：低/中/高/非常高
    spi4_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO 模式开启或者禁止
    spi4_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO 阈值选择
    spi4_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //存储器突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    spi4_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //外设突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    HAL_DMA_DeInit(&spi4_handle.dma_tx);  //取消初始化
    HAL_DMA_Init(&spi4_handle.dma_tx);    //初始化
    __HAL_LINKDMA(&spi4_handle.spi, hdmatx, spi4_handle.dma_tx);  //将DMA与SPI联系起来
    /* Rx DMA配置 */
    spi4_handle.dma_rx = spi4_handle.dma_tx;  //其它设置相同
    spi4_handle.dma_rx.Instance                 = DMA1_Stream7;             //数据流选择
    spi4_handle.dma_rx.Init.Request             = DMA_REQUEST_SPI4_RX;      //请求设置，设置是哪个外设请求的
    spi4_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //传输方向
    HAL_DMA_DeInit(&spi4_handle.dma_rx);  //取消初始化
    HAL_DMA_Init(&spi4_handle.dma_rx);    //初始化
    __HAL_LINKDMA(&spi4_handle.spi, hdmarx, spi4_handle.dma_rx);  //将DMA与SPI联系起来
    /* SPI配置 */
    spi4_handle.spi.Instance                        = SPI4;                                     //寄存器基地址
    spi4_handle.spi.Init.Mode                       = SPI_MODE_MASTER;                          //模式
    spi4_handle.spi.Init.Direction                  = SPI_DIRECTION_2LINES;                     //SPI双向模式状态
    spi4_handle.spi.Init.DataSize                   = SPI_DATASIZE_8BIT;                        //数据大小
    spi4_handle.spi.Init.CLKPolarity                = SPI_POLARITY_HIGH;                        //时钟空闲状态
    spi4_handle.spi.Init.CLKPhase                   = SPI_PHASE_2EDGE;                          //第几个上升沿
    spi4_handle.spi.Init.NSS                        = SPI_NSS_HARD_OUTPUT;                      //硬件(NSS引脚)或软件使用SSI位
    spi4_handle.spi.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_32;                 //波特率预算器值分频
    spi4_handle.spi.Init.FirstBit                   = SPI_FIRSTBIT_MSB;                         //数据传输是从MSB位还是LSB位开始
    spi4_handle.spi.Init.TIMode                     = SPI_TIMODE_DISABLE;                       //是否启用TI模式
    spi4_handle.spi.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;               //是否启用CRC计算
    spi4_handle.spi.Init.CRCPolynomial              = 0x0UL;                                    //CRC计算的多项式
    spi4_handle.spi.Init.CRCLength                  = SPI_CRC_LENGTH_DATASIZE;                  //CRC长度
    spi4_handle.spi.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;                     //是否启用NSSP信号
    spi4_handle.spi.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;                     //指定SS输入/输出外部信号有效电平
    spi4_handle.spi.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;                //FIFO阈值级别
    spi4_handle.spi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC初始化模式
    spi4_handle.spi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC初始化模式
    spi4_handle.spi.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;           //指定额外延迟
    spi4_handle.spi.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;    //指定最小时间延迟
    spi4_handle.spi.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;           //在主接收模式下控制连续SPI传输并自动管理，以避免超限的情况
    spi4_handle.spi.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;         //备用功能gpio状态控制
    spi4_handle.spi.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;                      //反转MISO/MOSI
    HAL_SPI_Init(&spi4_handle.spi);  //初始化
    /* DMA中断配置 */
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);          //使能中断
    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);          //使能中断
}
/**
* @brief SPI DMA发送中断
*/
void DMA1_Stream6_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = spi4_handle.spi.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //调用HAL库中断处理公用函数
}
/**
* @brief SPI DMA接收中断
*/
void DMA1_Stream7_IRQHandler(void)
{
    uint32_t len = 0;
    SPI_HandleTypeDef *handle = &spi4_handle.spi;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
        
        HAL_SPI_Abort(handle);  //停止DMA传送
        len = handle->RxXferSize - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        spi4_receive(handle->pRxBuffPtr, len);  //数据输出
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx)) != RESET)  //半传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输错误中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
    }
}
/**
* @brief  SPI 读写数据
* @attention
*/
uint16_t spi4_read_write(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_SPI_GET_FLAG(&spi4_handle.spi, SPI_SR_TXP) != SET)  //上次发送未完成
    {
        return 0;
    }

    if(len > sizeof(spi_dma_buf.s4_tx))  //长度超出DMA缓存区
    {
        len = sizeof(spi_dma_buf.s4_tx);
    }
    i = len;
    while(i--)
    {
        spi_dma_buf.s4_tx[i] = data[i];
    }

    HAL_SPI_TransmitReceive_DMA(&spi4_handle.spi, spi_dma_buf.s4_tx, spi_dma_buf.s4_rx, len);  //txdma启动，开启各dma中断
    
    return len;
}
/*spi4================================================================================================*/
/*spi6*************************************************************************************************/
/* spi6初始化 */
void spi6_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_SPI_DeInit(&spi6_handle.spi);
    /* 时钟使能 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_BDMA_CLK_ENABLE();
    __HAL_RCC_SPI6_CLK_ENABLE();
    /* IO配置 */
    /**SPI6 GPIO Configuration
    PB3 (JTDO/TRACESWO)     ------> SPI6_SCK
    PB4 (NJTRST)     ------> SPI6_MISO
    PB5     ------> SPI6_MOSI
    PD7     ------> SPI6_NSS
    */
    gpio_init_struct.Pin       = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //模式
    gpio_init_struct.Pull      = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF8_SPI6;           //复用
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);  //初始化
    gpio_init_struct.Pin       = GPIO_PIN_7; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_OUTPUT_PP;     //模式
    gpio_init_struct.Pull      = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);  //初始化
    /* DMA配置 */
    /* Tx DMA配置 */
    spi6_handle.dma_tx.Instance                 = BDMA_Channel0;            //数据流选择
    spi6_handle.dma_tx.Init.Request             = BDMA_REQUEST_SPI6_TX;     //请求设置，设置是哪个外设请求的
    spi6_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //传输方向
    spi6_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //外设（非）增量模式
    spi6_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //存储器（非）增量模式
    spi6_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //外设数据大小： 8/16/32 位
    spi6_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //存储器数据大小： 8/16/32 位
    spi6_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //模式：外设流控模式/循环模式/普通模式
    spi6_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA 优先级：低/中/高/非常高
    spi6_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO 模式开启或者禁止
    spi6_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO 阈值选择
    spi6_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //存储器突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    spi6_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //外设突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    HAL_DMA_DeInit(&spi6_handle.dma_tx);  //取消初始化
    HAL_DMA_Init(&spi6_handle.dma_tx);    //初始化
    __HAL_LINKDMA(&spi6_handle.spi, hdmatx, spi6_handle.dma_tx);  //将DMA与SPI联系起来
    /* Rx DMA配置 */
    spi6_handle.dma_rx = spi6_handle.dma_tx;  //其它设置相同
    spi6_handle.dma_rx.Instance                 = BDMA_Channel1;             //数据流选择
    spi6_handle.dma_rx.Init.Request             = BDMA_REQUEST_SPI6_RX;     //请求设置，设置是哪个外设请求的
    spi6_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //传输方向
    HAL_DMA_DeInit(&spi6_handle.dma_rx);  //取消初始化
    HAL_DMA_Init(&spi6_handle.dma_rx);    //初始化
    __HAL_LINKDMA(&spi6_handle.spi, hdmarx, spi6_handle.dma_rx);  //将DMA与SPI联系起来
    /* SPI配置 */
    spi6_handle.spi.Instance                        = SPI6;                                     //寄存器基地址
    spi6_handle.spi.Init.Mode                       = SPI_MODE_MASTER;                          //模式
    spi6_handle.spi.Init.Direction                  = SPI_DIRECTION_2LINES;                     //SPI双向模式状态
    spi6_handle.spi.Init.DataSize                   = SPI_DATASIZE_8BIT;                        //数据大小
    spi6_handle.spi.Init.CLKPolarity                = SPI_POLARITY_HIGH;                        //时钟空闲状态
    spi6_handle.spi.Init.CLKPhase                   = SPI_PHASE_2EDGE;                          //第几个上升沿
    spi6_handle.spi.Init.NSS                        = SPI_NSS_SOFT;                             //硬件(NSS引脚)或软件使用SSI位
    spi6_handle.spi.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_32;                 //波特率预算器值分频
    spi6_handle.spi.Init.FirstBit                   = SPI_FIRSTBIT_MSB;                         //数据传输是从MSB位还是LSB位开始
    spi6_handle.spi.Init.TIMode                     = SPI_TIMODE_DISABLE;                       //是否启用TI模式
    spi6_handle.spi.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;               //是否启用CRC计算
    spi6_handle.spi.Init.CRCPolynomial              = 0x0UL;                                    //CRC计算的多项式
    spi6_handle.spi.Init.CRCLength                  = SPI_CRC_LENGTH_DATASIZE;                  //CRC长度
    spi6_handle.spi.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;                     //是否启用NSSP信号
    spi6_handle.spi.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;                     //指定SS输入/输出外部信号有效电平
    spi6_handle.spi.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;                //FIFO阈值级别
    spi6_handle.spi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC初始化模式
    spi6_handle.spi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC初始化模式
    spi6_handle.spi.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;           //指定额外延迟
    spi6_handle.spi.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;    //指定最小时间延迟
    spi6_handle.spi.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;           //在主接收模式下控制连续SPI传输并自动管理，以避免超限的情况
    spi6_handle.spi.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;         //备用功能gpio状态控制
    spi6_handle.spi.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;                      //反转MISO/MOSI
    HAL_SPI_Init(&spi6_handle.spi);  //初始化
    /* DMA中断配置 */
    HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);          //使能中断
    HAL_NVIC_SetPriority(BDMA_Channel1_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(BDMA_Channel1_IRQn);          //使能中断
    /* 启动 */
    SPI6_NSS(1);
}
/**
* @brief SPI DMA发送中断
*/
void BDMA_Channel0_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = spi6_handle.spi.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //调用HAL库中断处理公用函数
}
/**
* @brief SPI DMA接收中断
*/
void BDMA_Channel1_IRQHandler(void)
{
    uint32_t len = 0;
    SPI_HandleTypeDef *handle = &spi6_handle.spi;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
        
        HAL_SPI_Abort(handle);  //停止DMA传送
        len = handle->RxXferSize - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        spi6_receive(handle->pRxBuffPtr, len);  //数据输出
        
        SPI6_NSS(1);
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx)) != RESET)  //半传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输错误中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
    }
}
/**
* @brief  SPI 读写数据
* @attention
*/
uint16_t spi6_read_write(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_SPI_GET_FLAG(&spi6_handle.spi, SPI_SR_TXP) != SET)  //上次发送未完成
    {
        return 0;
    }

    if(len > sizeof(spi_dma_buf.s6_tx))  //长度超出DMA缓存区
    {
        len = sizeof(spi_dma_buf.s6_tx);
    }
    i = len;
    while(i--)
    {
        spi_dma_buf.s6_tx[i] = data[i];
    }
    SPI6_NSS(0);
    HAL_SPI_TransmitReceive_DMA(&spi6_handle.spi, spi_dma_buf.s6_tx, spi_dma_buf.s6_rx, len);  //txdma启动，开启各dma中断
    
    return len;
}
/*spi6================================================================================================*/
