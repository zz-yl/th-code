/**@file   bsp_spi.c
* @brief   硬件SPI驱动,DMA,中断,
*          针对ADC芯片的SPI驱动
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

#define SPI_DMA_RX_SIZE  10
#define SPI_DMA_TX_SIZE  10

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  SPI_DMA_DATA
* @brief   SPI的DMA数据结构
*/
typedef struct
{
    uint16_t Rx[SPI_DMA_RX_SIZE];
    uint16_t Tx[SPI_DMA_TX_SIZE];
}SPI_DMA_DATA;

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

SPI_DMA_DATA Spi1Dma = {0};  //SPI收发DMA缓存

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/*spi1*************************************************************************************************/
/* SPI1初始化 */
void Spi1Init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_InitTypeDef  SPI_InitStruct;
    DMA_InitTypeDef  DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    GPIO_StructInit(&GPIO_InitStruct);
    SPI_StructInit(&SPI_InitStruct);
    DMA_StructInit(&DMA_InitStruct);
    
    DMA_DeInit(DMA2_Stream0);
    DMA_DeInit(DMA2_Stream5);
    /* 时钟配置 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);  //使能SPI时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  //使能DMA时钟
    /* GPIO配置 */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;  //通道
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       //IO模式,复用功能
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO口驱动电路响应速度100MHz
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //推挽输出
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //上拉
    GPIO_Init(GPIOA, &GPIO_InitStruct);              //初始化
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);  //复用
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);  //复用
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);  //复用
    /* SPI配置 */
    SPI_InitStruct.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;  //SPI设置为双线双向全双工
    SPI_InitStruct.SPI_Mode              = SPI_Mode_Master;                  //SPI工作模式为主SPI
    SPI_InitStruct.SPI_DataSize          = SPI_DataSize_8b;                  //SPI收发数据宽度x位
    SPI_InitStruct.SPI_CPOL              = SPI_CPOL_Low;                     //串行同步时钟的空闲状态为低电平
    SPI_InitStruct.SPI_CPHA              = SPI_CPHA_2Edge;                   //串行同步时钟的第x个跳变沿（上升或下降）数据被采样
    SPI_InitStruct.SPI_NSS               = SPI_NSS_Soft;                     //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;         //定义波特率预分频的值
    SPI_InitStruct.SPI_FirstBit          = SPI_FirstBit_MSB;                 //数据传输从MSB位开始
    SPI_InitStruct.SPI_CRCPolynomial     = 7;                                //CRC值计算的多项式
    SPI_Init(SPI1, &SPI_InitStruct);                                         //初始化

    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);     //使能SPI的DMA收发
    /* DMA配置 */
    /* RxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_3;                   //通道
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(SPI1->DR);           //外设寄存器(多重模式)基地址
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Spi1Dma.Rx;            //存储器基地址
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralToMemory;      //DMA传输方向,外设到存储器
    DMA_InitStruct.DMA_BufferSize         = 0;                               //传输的字节数
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //外设地址递增,关闭
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //存储器地址递增,使能
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //外设数据宽度
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;         //存储器数据宽度
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;                 //非循环模式
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;               //优先级
    DMA_InitStruct.DMA_FIFOMode           = DMA_FIFOMode_Disable;            //FIFO,关闭
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;  //FIFO阈值级别,1/4
    DMA_InitStruct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;          //存储器突发传输配置,单次传输
    DMA_InitStruct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;      //外设突发传输配置,单次传输
    DMA_Init(DMA2_Stream0, &DMA_InitStruct);                                 //初始化
    /* TxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_3;                   //通道
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Spi1Dma.Tx;            //存储器基地址
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;      //DMA传输方向,存储器到外设
    DMA_InitStruct.DMA_BufferSize         = 0;                               //传输的字节数,初始化时不传输
    DMA_Init(DMA2_Stream5, &DMA_InitStruct);                                 //初始化
    /* DMA中断配置 */
    /* RxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream0_IRQn;  //中断名
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;               //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x03;               //子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;             //使能
    NVIC_Init(&NVIC_InitStruct);                                            //初始化
    /* TxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream5_IRQn;  //中断名
    NVIC_Init(&NVIC_InitStruct);                                            //初始化
    
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);  //DMA中断使能,传输完成中断
    DMA_ITConfig(DMA2_Stream5, DMA_IT_TC | DMA_IT_FE, ENABLE);  //DMA中断使能,传输完成中断,上溢下溢中断
    /* 启动传输 */
    SPI_Cmd(SPI1, ENABLE);           //使能SPI外设
    DMA_Cmd(DMA2_Stream0, DISABLE);  //禁止DMA传输,Rx
    DMA_Cmd(DMA2_Stream5, DISABLE);  //禁止DMA传输,Tx
}
/**
* @brief DMA2数据流0中断,Spi1Rx
*/
void DMA2_Stream0_IRQHandler(void)
{
    uint16_t clean = 0;
    
    if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
        
//        Max1168GetBuf(Spi1Dma.Rx);
        
        //清状态
        clean = SPI1->DR;
        clean = SPI1->SR;
    }
}
/**
* @brief DMA2数据流5中断,Spi1Tx
*/
void DMA2_Stream5_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
        
        DMA_Cmd(DMA2_Stream5, DISABLE);  //禁止DMA
    }
    if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_FEIF5) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_FEIF5);
        
    }
}
/**
* @brief  SPI1读数据
* @attention 参数:len:长度;设定Rx的DMA长度,并启动
*/
void Spi1Read(uint16_t len)
{
    DMA_Cmd(DMA2_Stream0, DISABLE);             //禁止接收DMA
    DMA_SetCurrDataCounter(DMA2_Stream0, len);  //设置DMA接收长度
    DMA_Cmd(DMA2_Stream0, ENABLE);              //使能接收DMA
}
/**
* @brief  SPI1写数据
* @attention 参数:data:数据,len:长度
*/
void Spi1Write(uint16_t *data, uint16_t len)
{
    uint16_t cnt = 0;

    if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) != RESET)  //总线忙
    {
        return;
    }
    
    if(len > SPI_DMA_TX_SIZE)
    {
        return;
    }
    while(len--)
    {
        Spi1Dma.Tx[cnt] = data[cnt];
        cnt++;
    }

    DMA_Cmd(DMA2_Stream5, DISABLE);  //禁止DMA
    DMA_SetCurrDataCounter(DMA2_Stream5, cnt);  //设置长度
    DMA_Cmd(DMA2_Stream5, ENABLE);  //使能DMA
}
/**
* @brief  SPI1 读写一个字节
* @attention 参数:tx_data:要写入的字节;返回值:读取到的字节
*/
uint8_t Spi1ReadWriteByte(uint8_t tx_data)
{
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空
    SPI_I2S_SendData(SPI1, tx_data); //通过外设SPIx发送一个byte数据
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一个byte

    return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据
}

