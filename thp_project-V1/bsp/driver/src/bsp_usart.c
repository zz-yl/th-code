/**@file   bsp_usart.c
* @brief   串口驱动
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_usart.h"
#include "bsp_cfg.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define USART1_BAUD  115200  //串口波特率
#define USART2_BAUD  115200  //串口波特率
#define USART3_BAUD  115200  //串口波特率
#define UART4_BAUD   115200  //串口波特率
#define UART5_BAUD   115200  //串口波特率
#define USART6_BAUD  115200  //串口波特率

/**
* @struct  UART_DMA_DATA
* @brief   串口DMA数据
*/
typedef struct
{
    uint8_t Rx[UART_DMA_RX_SIZE];
    uint8_t Tx[UART_DMA_TX_SIZE];
}UART_DMA_DATA;


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

UART_DMA_DATA Usart1DMA = {0};  //串口1 DMA数据
UART_DMA_DATA Usart2DMA = {0};  //串口2 DMA数据
UART_DMA_DATA Usart3DMA = {0};  //串口3 DMA数据
UART_DMA_DATA Uart4DMA  = {0};  //串口4 DMA数据
UART_DMA_DATA Uart5DMA  = {0};  //串口5 DMA数据
UART_DMA_DATA Usart6DMA = {0};  //串口6 DMA数据

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/*串口1*************************************************************************************************/
/**
* @brief  串口初始化
* @attention 使用了DMA功能,共开启三个中断,开启Tx和Rx的DMA传输完成中断,判断DMA传输完成
*            开启串口总线空闲中断用于判断一帧数据接收完成
*/
void Usart1Init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef   DMA_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;

    GPIO_StructInit(&GPIO_InitStruct);
    USART_StructInit(&USART_InitStruct);
    DMA_StructInit(&DMA_InitStruct);
    DMA_DeInit(DMA2_Stream2);
    DMA_DeInit(DMA2_Stream7);
    /* 使能时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    /* GPIO配置 */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;  //通道
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       //模式
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO口驱动电路响应速度
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //输出模式
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //上下拉
    GPIO_Init(GPIOA, &GPIO_InitStruct);              //初始化
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    /* 串口配置 */
    USART_InitStruct.USART_BaudRate            = USART1_BAUD;                     //波特率
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;             //8个数据位
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;                //1个停止位
    USART_InitStruct.USART_Parity              = USART_Parity_No;                 //无奇偶校验
    USART_InitStruct.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;   //收发模式
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件数据流控制
    USART_Init(USART1, &USART_InitStruct);                                        //初始化

    USART_Cmd(USART1, ENABLE);                                                    //使能
    USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);              //使能DMA串口收发
    /* 串口中断初始化 */
    NVIC_InitStruct.NVIC_IRQChannel                   = USART1_IRQn;  //中断名
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;         //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;         //子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;       //使能
    NVIC_Init(&NVIC_InitStruct);                                      //初始化
    
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);  //使能总线空闲中断,判断一帧数据接受完成
    /* DMA配置 */
    /* RxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //通道
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);         //外设寄存器(多重模式)基地址
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Usart1DMA.Rx;          //存储器基地址
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralToMemory;      //DMA传输方向,外设到存储器
    DMA_InitStruct.DMA_BufferSize         = UART_DMA_RX_SIZE;                //传输的字节数
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //外设地址递增,关闭
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //存储器地址递增,使能
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //外设数据宽度,按字节对齐,8位
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;         //存储器数据宽度,按字节对齐,8位
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;                 //非循环模式
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;               //优先级
    DMA_InitStruct.DMA_FIFOMode           = DMA_FIFOMode_Disable;            //FIFO,关闭
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;  //FIFO阈值级别,1/4
    DMA_InitStruct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;          //存储器突发传输配置,单次传输
    DMA_InitStruct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;      //外设突发传输配置,单次传输
    DMA_Init(DMA2_Stream2, &DMA_InitStruct);                                 //初始化
    /* TxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //通道
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Usart1DMA.Tx;          //存储器基地址
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;      //DMA传输方向,存储器到外设
    DMA_InitStruct.DMA_BufferSize         = 0;                               //传输的字节数,初始化时不传输
    DMA_Init(DMA2_Stream7, &DMA_InitStruct);                                 //初始化
    /* DMA中断配置 */
    /* RxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream2_IRQn;  //中断名
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;               //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;               //子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;             //使能
    NVIC_Init(&NVIC_InitStruct);                                            //初始化
    /* TxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream7_IRQn;  //中断名
    NVIC_Init(&NVIC_InitStruct);                                            //初始化
    
    DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);  //DMA中断使能,传输完成中断
    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);  //DMA中断使能,传输完成中断
    /* 启动传输 */
    DMA_Cmd(DMA2_Stream2, ENABLE);   //启动DMA传输,Rx
    DMA_Cmd(DMA2_Stream7, DISABLE);  //禁止DMA传输,Tx
}
/**
* @brief  串口中断
* @attention 
*/
void USART1_IRQHandler(void)
{
    uint16_t len = 0;  //数据长度
    
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_IDLE);
        
        DMA_Cmd(DMA2_Stream2, DISABLE);  //关闭接收DMA
        len = sizeof(Usart1DMA.Rx) - DMA_GetCurrDataCounter(DMA2_Stream2);  //获取接收数据长度
        
        Usart1Receive(Usart1DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA2_Stream2, UART_DMA_RX_SIZE);  //设置DMA接收长度
        DMA_Cmd(DMA2_Stream2, ENABLE);     //使能接收DMA
        
        //读DR避免频繁进中断
        len = USART1->DR;
        len = USART1->SR;
    }
}
/**
* @brief 串口 DMA中断,RX
*/
void DMA2_Stream2_IRQHandler(void)
{
    uint16_t len = 0;  //数据长度
    
    if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);

        DMA_Cmd(DMA2_Stream2, DISABLE);     //关闭接收DMA
        len = sizeof(Usart1DMA.Rx) - DMA_GetCurrDataCounter(DMA2_Stream2);  //获取接收数据长度
        
        Usart1Receive(Usart1DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA2_Stream2, UART_DMA_RX_SIZE);  //设置DMA接收长度
        DMA_Cmd(DMA2_Stream2, ENABLE);     //使能接收DMA
        USART_ClearITPendingBit(USART1, USART_IT_IDLE);  //必须在此位置清接收一帧数据中断
        
        //读DR避免频繁进中断
        len = USART1->DR;
        len = USART1->SR;
    }
}
/**
* @brief 串口 DMA中断,TX
*/
void DMA2_Stream7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);

        DMA_Cmd(DMA2_Stream7, DISABLE);     //关闭接收DMA
        DMA_SetCurrDataCounter(DMA2_Stream7, 0);  //清除数据长度
    }
}
/**
* @brief 串口发送数据
*/
uint16_t Usart1Send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET)  //上次发送未完成
    {
        return 0;
    }

    if(len > UART_DMA_TX_SIZE)  //长度超出DMA缓存区
    {
        len = UART_DMA_TX_SIZE;
    }
    i = len;
    while(i--)
    {
        Usart1DMA.Tx[i] = data[i];
    }

    DMA_Cmd(DMA2_Stream7, DISABLE);  //关闭DMA传输,DMA传输数据需先关闭,写入长度,再启动
    DMA_SetCurrDataCounter(DMA2_Stream7, len);  //重新写入传输数据长度
    DMA_Cmd(DMA2_Stream7, ENABLE);  //启动DMA传输

    return len;
}
/*串口2*************************************************************************************************/
/**
* @brief  串口初始化
* @attention 使用了DMA功能,共开启三个中断,开启Tx和Rx的DMA传输完成中断,判断DMA传输完成
*            开启串口总线空闲中断用于判断一帧数据接收完成
*/
void Usart2Init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef   DMA_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;

    GPIO_StructInit(&GPIO_InitStruct);
    USART_StructInit(&USART_InitStruct);
    DMA_StructInit(&DMA_InitStruct);
    DMA_DeInit(DMA1_Stream5);
    DMA_DeInit(DMA1_Stream6);
    /* 使能时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    /* GPIO配置 */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;  //通道
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       //模式
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO口驱动电路响应速度
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //输出模式
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //上下拉
    GPIO_Init(GPIOA, &GPIO_InitStruct);              //初始化
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
    /* 串口配置 */
    USART_InitStruct.USART_BaudRate            = USART2_BAUD;                     //波特率
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;             //8个数据位
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;                //1个停止位
    USART_InitStruct.USART_Parity              = USART_Parity_No;                 //无奇偶校验
    USART_InitStruct.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;   //收发模式
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件数据流控制
    USART_Init(USART2, &USART_InitStruct);                                        //初始化

    USART_Cmd(USART2, ENABLE);                                                    //使能
    USART_DMACmd(USART2, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);              //使能DMA串口收发
    /* 串口中断初始化 */
    NVIC_InitStruct.NVIC_IRQChannel                   = USART2_IRQn;  //中断名
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;         //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;         //子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;       //使能
    NVIC_Init(&NVIC_InitStruct);                                      //初始化
    
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);  //使能总线空闲中断,判断一帧数据接受完成
    /* DMA配置 */
    /* RxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //通道
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);         //外设寄存器(多重模式)基地址
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Usart2DMA.Rx;          //存储器基地址
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralToMemory;      //DMA传输方向,外设到存储器
    DMA_InitStruct.DMA_BufferSize         = UART_DMA_RX_SIZE;                //传输的字节数
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //外设地址递增,关闭
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //存储器地址递增,使能
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //外设数据宽度,按字节对齐,8位
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;         //存储器数据宽度,按字节对齐,8位
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;                 //非循环模式
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;               //优先级
    DMA_InitStruct.DMA_FIFOMode           = DMA_FIFOMode_Disable;            //FIFO,关闭
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;  //FIFO阈值级别,1/4
    DMA_InitStruct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;          //存储器突发传输配置,单次传输
    DMA_InitStruct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;      //外设突发传输配置,单次传输
    DMA_Init(DMA1_Stream5, &DMA_InitStruct);                                 //初始化
    /* TxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //通道
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Usart2DMA.Tx;          //存储器基地址
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;      //DMA传输方向,存储器到外设
    DMA_InitStruct.DMA_BufferSize         = 0;                               //传输的字节数,初始化时不传输
    DMA_Init(DMA1_Stream6, &DMA_InitStruct);                                 //初始化
    /* DMA中断配置 */
    /* RxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Stream5_IRQn;  //中断名
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;               //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;               //子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;             //使能
    NVIC_Init(&NVIC_InitStruct);                                            //初始化
    /* TxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Stream6_IRQn;  //中断名
    NVIC_Init(&NVIC_InitStruct);                                            //初始化
    
    DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);  //DMA中断使能,传输完成中断
    DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);  //DMA中断使能,传输完成中断
    /* 启动传输 */
    DMA_Cmd(DMA1_Stream5, ENABLE);   //启动DMA传输,Rx
    DMA_Cmd(DMA1_Stream6, DISABLE);  //禁止DMA传输,Tx
}
/**
* @brief  串口中断
* @attention 
*/
void USART2_IRQHandler(void)
{
    uint16_t len = 0;  //数据长度
    
    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        USART_ClearITPendingBit(USART2, USART_IT_IDLE);
        
        DMA_Cmd(DMA1_Stream5, DISABLE);  //关闭接收DMA
        len = sizeof(Usart2DMA.Rx) - DMA_GetCurrDataCounter(DMA1_Stream5);  //获取接收数据长度
        
        Usart2Receive(Usart2DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA1_Stream5, UART_DMA_RX_SIZE);  //设置DMA接收长度
        DMA_Cmd(DMA1_Stream5, ENABLE);     //使能接收DMA
        
        //读DR避免频繁进中断
        len = USART2->DR;
        len = USART2->SR;
    }
}
/**
* @brief 串口 DMA中断,RX
*/
void DMA1_Stream5_IRQHandler(void)
{
    uint16_t len = 0;  //数据长度
    
    if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);

        DMA_Cmd(DMA1_Stream5, DISABLE);     //关闭接收DMA
        len = sizeof(Usart2DMA.Rx) - DMA_GetCurrDataCounter(DMA1_Stream5);  //获取接收数据长度
        
        Usart2Receive(Usart2DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA1_Stream5, UART_DMA_RX_SIZE);  //设置DMA接收长度
        DMA_Cmd(DMA1_Stream5, ENABLE);     //使能接收DMA
        USART_ClearITPendingBit(USART2, USART_IT_IDLE);  //必须在此位置清接收一帧数据中断
        
        //读DR避免频繁进中断
        len = USART2->DR;
        len = USART2->SR;
    }
}
/**
* @brief 串口 DMA中断,TX
*/
void DMA1_Stream6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);

        DMA_Cmd(DMA1_Stream6, DISABLE);     //关闭接收DMA
        DMA_SetCurrDataCounter(DMA1_Stream6, 0);  //清除数据长度
    }
}
/**
* @brief 串口发送数据
*/
uint16_t Usart2Send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(USART_GetFlagStatus(USART2, USART_FLAG_TC) != SET)  //上次发送未完成
    {
        return 0;
    }

    if(len > UART_DMA_TX_SIZE)  //长度超出DMA缓存区
    {
        len = UART_DMA_TX_SIZE;
    }
    i = len;
    while(i--)
    {
        Usart2DMA.Tx[i] = data[i];
    }

    DMA_Cmd(DMA1_Stream6, DISABLE);  //关闭DMA传输,DMA传输数据需先关闭,写入长度,再启动
    DMA_SetCurrDataCounter(DMA1_Stream6, len);  //重新写入传输数据长度
    DMA_Cmd(DMA1_Stream6, ENABLE);  //启动DMA传输

    return len;
}
/*串口3*************************************************************************************************/
/**
* @brief  串口初始化
* @attention 使用了DMA功能,共开启三个中断,开启Tx和Rx的DMA传输完成中断,判断DMA传输完成
*            开启串口总线空闲中断用于判断一帧数据接收完成
*/
void Usart3Init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef   DMA_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;

    GPIO_StructInit(&GPIO_InitStruct);
    USART_StructInit(&USART_InitStruct);
    DMA_StructInit(&DMA_InitStruct);
    DMA_DeInit(DMA1_Stream1);
    DMA_DeInit(DMA1_Stream3);
    /* 使能时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    /* GPIO配置 */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;  //通道
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       //模式
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO口驱动电路响应速度
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //输出模式
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //上下拉
    GPIO_Init(GPIOC, &GPIO_InitStruct);              //初始化
    
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
    /* 串口配置 */
    USART_InitStruct.USART_BaudRate            = USART3_BAUD;                     //波特率
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;             //8个数据位
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;                //1个停止位
    USART_InitStruct.USART_Parity              = USART_Parity_No;                 //无奇偶校验
    USART_InitStruct.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;   //收发模式
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件数据流控制
    USART_Init(USART3, &USART_InitStruct);                                        //初始化

    USART_Cmd(USART3, ENABLE);                                                    //使能
    USART_DMACmd(USART3, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);              //使能DMA串口收发
    /* 串口中断初始化 */
    NVIC_InitStruct.NVIC_IRQChannel                   = USART3_IRQn;  //中断名
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;         //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;         //子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;       //使能
    NVIC_Init(&NVIC_InitStruct);                                      //初始化
    
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);  //使能总线空闲中断,判断一帧数据接受完成
    /* DMA配置 */
    /* RxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //通道
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);         //外设寄存器(多重模式)基地址
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Usart3DMA.Rx;          //存储器基地址
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralToMemory;      //DMA传输方向,外设到存储器
    DMA_InitStruct.DMA_BufferSize         = UART_DMA_RX_SIZE;                //传输的字节数
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //外设地址递增,关闭
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //存储器地址递增,使能
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //外设数据宽度,按字节对齐,8位
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;         //存储器数据宽度,按字节对齐,8位
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;                 //非循环模式
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;               //优先级
    DMA_InitStruct.DMA_FIFOMode           = DMA_FIFOMode_Disable;            //FIFO,关闭
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;  //FIFO阈值级别,1/4
    DMA_InitStruct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;          //存储器突发传输配置,单次传输
    DMA_InitStruct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;      //外设突发传输配置,单次传输
    DMA_Init(DMA1_Stream1, &DMA_InitStruct);                                 //初始化
    /* TxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //通道
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Usart3DMA.Tx;          //存储器基地址
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;      //DMA传输方向,存储器到外设
    DMA_InitStruct.DMA_BufferSize         = 0;                               //传输的字节数,初始化时不传输
    DMA_Init(DMA1_Stream3, &DMA_InitStruct);                                 //初始化
    /* DMA中断配置 */
    /* RxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Stream1_IRQn;  //中断名
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;               //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;               //子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;             //使能
    NVIC_Init(&NVIC_InitStruct);                                            //初始化
    /* TxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Stream3_IRQn;  //中断名
    NVIC_Init(&NVIC_InitStruct);                                            //初始化
    
    DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);  //DMA中断使能,传输完成中断
    DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);  //DMA中断使能,传输完成中断
    /* 启动传输 */
    DMA_Cmd(DMA1_Stream1, ENABLE);   //启动DMA传输,Rx
    DMA_Cmd(DMA1_Stream3, DISABLE);  //禁止DMA传输,Tx
}
/**
* @brief  串口中断
* @attention 
*/
void USART3_IRQHandler(void)
{
    uint16_t len = 0;  //数据长度
    
    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);
        
        DMA_Cmd(DMA1_Stream1, DISABLE);  //关闭接收DMA
        len = sizeof(Usart3DMA.Rx) - DMA_GetCurrDataCounter(DMA1_Stream1);  //获取接收数据长度
        
        Usart3Receive(Usart3DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA1_Stream1, UART_DMA_RX_SIZE);  //设置DMA接收长度
        DMA_Cmd(DMA1_Stream1, ENABLE);     //使能接收DMA
        
        //读DR避免频繁进中断
        len = USART3->DR;
        len = USART3->SR;
    }
}
/**
* @brief 串口 DMA中断,RX
*/
void DMA1_Stream1_IRQHandler(void)
{
    uint16_t len = 0;  //数据长度
    
    if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);

        DMA_Cmd(DMA1_Stream1, DISABLE);     //关闭接收DMA
        len = sizeof(Usart3DMA.Rx) - DMA_GetCurrDataCounter(DMA1_Stream1);  //获取接收数据长度
        
        Usart3Receive(Usart3DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA1_Stream1, UART_DMA_RX_SIZE);  //设置DMA接收长度
        DMA_Cmd(DMA1_Stream1, ENABLE);     //使能接收DMA
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);  //必须在此位置清接收一帧数据中断
        
        //读DR避免频繁进中断
        len = USART3->DR;
        len = USART3->SR;
    }
}
/**
* @brief 串口 DMA中断,TX
*/
void DMA1_Stream3_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);

        DMA_Cmd(DMA1_Stream3, DISABLE);     //关闭接收DMA
        DMA_SetCurrDataCounter(DMA1_Stream3, 0);  //清除数据长度
    }
}
/**
* @brief 串口发送数据
*/
uint16_t Usart3Send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(USART_GetFlagStatus(USART3, USART_FLAG_TC) != SET)  //上次发送未完成
    {
        return 0;
    }

    if(len > UART_DMA_TX_SIZE)  //长度超出DMA缓存区
    {
        len = UART_DMA_TX_SIZE;
    }
    i = len;
    while(i--)
    {
        Usart3DMA.Tx[i] = data[i];
    }

    DMA_Cmd(DMA1_Stream3, DISABLE);  //关闭DMA传输,DMA传输数据需先关闭,写入长度,再启动
    DMA_SetCurrDataCounter(DMA1_Stream3, len);  //重新写入传输数据长度
    DMA_Cmd(DMA1_Stream3, ENABLE);  //启动DMA传输

    return len;
}
/*串口4*************************************************************************************************/
/**
* @brief  串口初始化
* @attention 使用了DMA功能,共开启三个中断,开启Tx和Rx的DMA传输完成中断,判断DMA传输完成
*            开启串口总线空闲中断用于判断一帧数据接收完成
*/
void Uart4Init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    GPIO_StructInit(&GPIO_InitStruct);
    USART_StructInit(&USART_InitStruct);
    DMA_StructInit(&DMA_InitStruct);
    DMA_DeInit(DMA1_Stream2);
    DMA_DeInit(DMA1_Stream4);
    /* 使能时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    /* GPIO配置 */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1; //通道
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       //模式
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO口驱动电路响应速度
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //输出模式
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //上下拉
    GPIO_Init(GPIOA, &GPIO_InitStruct);              //初始化
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
    /* 串口配置 */
    USART_InitStruct.USART_BaudRate            = UART4_BAUD;                      //波特率
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;             //8个数据位
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;                //1个停止位
    USART_InitStruct.USART_Parity              = USART_Parity_No;                 //无奇偶校验
    USART_InitStruct.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;   //收发模式
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件数据流控制
    USART_Init(UART4, &USART_InitStruct);                                         //初始化

    USART_Cmd(UART4, ENABLE);                                                    //使能
    USART_DMACmd(UART4, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);              //使能DMA串口收发
    /* 串口中断初始化 */
    NVIC_InitStruct.NVIC_IRQChannel                   = UART4_IRQn;   //中断名
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;         //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;         //子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;       //使能
    NVIC_Init(&NVIC_InitStruct);                                      //初始化
    
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);  //使能总线空闲中断,判断一帧数据接受完成
    /* DMA配置 */
    /* RxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //通道
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR);          //外设寄存器(多重模式)基地址
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Uart4DMA.Rx;           //存储器基地址
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralToMemory;      //DMA传输方向,外设到存储器
    DMA_InitStruct.DMA_BufferSize         = UART_DMA_RX_SIZE;                //传输的字节数
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //外设地址递增,关闭
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //存储器地址递增,使能
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //外设数据宽度,按字节对齐,8位
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;         //存储器数据宽度,按字节对齐,8位
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;                 //非循环模式
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;               //优先级
    DMA_InitStruct.DMA_FIFOMode           = DMA_FIFOMode_Disable;            //FIFO,关闭
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;  //FIFO阈值级别,1/4
    DMA_InitStruct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;          //存储器突发传输配置,单次传输
    DMA_InitStruct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;      //外设突发传输配置,单次传输
    DMA_Init(DMA1_Stream2, &DMA_InitStruct);                                 //初始化
    /* TxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //通道
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Uart4DMA.Tx;           //存储器基地址
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;      //DMA传输方向,存储器到外设
    DMA_InitStruct.DMA_BufferSize         = 0;                               //传输的字节数,初始化时不传输
    DMA_Init(DMA1_Stream4, &DMA_InitStruct);                                 //初始化
    /* DMA中断配置 */
    /* RxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Stream2_IRQn;  //中断名
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;               //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;               //子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;             //使能
    NVIC_Init(&NVIC_InitStruct);                                            //初始化
    /* TxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Stream4_IRQn;  //中断名
    NVIC_Init(&NVIC_InitStruct);                                            //初始化
    
    DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);  //DMA中断使能,传输完成中断
    DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);  //DMA中断使能,传输完成中断
    /* 启动传输 */
    DMA_Cmd(DMA1_Stream2, DISABLE);  //禁止DMA传输,Rx
    DMA_Cmd(DMA1_Stream4, DISABLE);  //禁止DMA传输,Tx
}
/**
* @brief  串口中断
* @attention 
*/
void UART4_IRQHandler(void)
{
    uint16_t len = 0;  //数据长度
    
    if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
    {
        USART_ClearITPendingBit(UART4, USART_IT_IDLE);

        DMA_Cmd(DMA1_Stream2, DISABLE);     //关闭接收DMA
        len = sizeof(Uart4DMA.Rx) - DMA_GetCurrDataCounter(DMA1_Stream2);  //获取接收数据长度
        
        Uart4Receive(Uart4DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA1_Stream2, UART_DMA_RX_SIZE);  //设置DMA接收长度
        DMA_Cmd(DMA1_Stream2, ENABLE);     //使能接收DMA
        
        //读DR避免频繁进中断
        len = UART4->DR;
        len = UART4->SR;
    }
}
/**
* @brief 串口 DMA接收中断
*/
void DMA1_Stream2_IRQHandler(void)
{
    uint16_t len = 0;  //数据长度
    
    if(DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
        
        DMA_Cmd(DMA1_Stream2, DISABLE);     //关闭接收DMA
        len = sizeof(Uart4DMA.Rx) - DMA_GetCurrDataCounter(DMA1_Stream2);  //获取接收数据长度
        
        Uart4Receive(Uart4DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA1_Stream2, UART_DMA_RX_SIZE);  //设置DMA接收长度
        DMA_Cmd(DMA1_Stream2, ENABLE);     //使能接收DMA
        
        //读DR避免频繁进中断
        len = UART4->DR;
        len = UART4->SR;
    }
}
/**
* @brief 串口 DMA发送中断
*/
void DMA1_Stream4_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);

        DMA_Cmd(DMA1_Stream4, DISABLE);     //关闭接收DMA
        DMA_SetCurrDataCounter(DMA1_Stream4, 0);  //清除数据长度
    }
}
/**
* @brief 串口发送数据
*/
uint16_t Uart4Send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(USART_GetFlagStatus(UART4, USART_FLAG_TC) != SET)  //上次发送未完成
    {
        return 0;
    }

    if(len > UART_DMA_TX_SIZE)  //长度超出DMA缓存区
    {
        len = UART_DMA_TX_SIZE;
    }
    i = len;
    while(i--)
    {
        Uart4DMA.Tx[i] = data[i];
    }

    DMA_Cmd(DMA1_Stream4, DISABLE);  //关闭DMA传输,DMA传输数据需先关闭,写入长度,再启动
    DMA_SetCurrDataCounter(DMA1_Stream4, len);  //重新写入传输数据长度
    DMA_Cmd(DMA1_Stream4, ENABLE);  //启动DMA传输
    
    return len;
}
/*串口5*************************************************************************************************/
/**
* @brief  串口初始化
* @attention 使用了DMA功能,共开启三个中断,开启Tx和Rx的DMA传输完成中断,判断DMA传输完成
*            开启串口总线空闲中断用于判断一帧数据接收完成
*/
void Uart5Init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    GPIO_StructInit(&GPIO_InitStruct);
    USART_StructInit(&USART_InitStruct);
    DMA_StructInit(&DMA_InitStruct);
    DMA_DeInit(DMA1_Stream0);
    DMA_DeInit(DMA1_Stream7);
    /* 使能时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    /* GPIO配置 */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_12;        //通道
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       //模式
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO口驱动电路响应速度
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //输出模式
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //上下拉
    GPIO_Init(GPIOC, &GPIO_InitStruct);              //初始化

    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2;        //通道
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;      //模式
    GPIO_Init(GPIOD, &GPIO_InitStruct);             //初始化
    
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
    /* 串口配置 */
    USART_InitStruct.USART_BaudRate            = UART5_BAUD;                      //波特率
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;             //8个数据位
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;                //1个停止位
    USART_InitStruct.USART_Parity              = USART_Parity_No;                 //无奇偶校验
    USART_InitStruct.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;   //收发模式
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件数据流控制
    USART_Init(UART5, &USART_InitStruct);                                         //初始化

    USART_Cmd(UART5, ENABLE);                                                    //使能
    USART_DMACmd(UART5, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);              //使能DMA串口收发
    /* 串口中断初始化 */
    NVIC_InitStruct.NVIC_IRQChannel                   = UART5_IRQn;   //中断名
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;         //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;         //子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;       //使能
    NVIC_Init(&NVIC_InitStruct);                                      //初始化
    
    USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);  //使能总线空闲中断,判断一帧数据接受完成
    /* DMA配置 */
    /* RxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //通道
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(UART5->DR);          //外设寄存器(多重模式)基地址
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Uart5DMA.Rx;           //存储器基地址
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralToMemory;      //DMA传输方向,外设到存储器
    DMA_InitStruct.DMA_BufferSize         = UART_DMA_RX_SIZE;                //传输的字节数
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //外设地址递增,关闭
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //存储器地址递增,使能
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //外设数据宽度,按字节对齐,8位
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;         //存储器数据宽度,按字节对齐,8位
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;                 //非循环模式
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;               //优先级
    DMA_InitStruct.DMA_FIFOMode           = DMA_FIFOMode_Disable;            //FIFO,关闭
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;  //FIFO阈值级别,1/4
    DMA_InitStruct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;          //存储器突发传输配置,单次传输
    DMA_InitStruct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;      //外设突发传输配置,单次传输
    DMA_Init(DMA1_Stream0, &DMA_InitStruct);                                 //初始化
    /* TxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //通道
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Uart5DMA.Tx;           //存储器基地址
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;      //DMA传输方向,存储器到外设
    DMA_InitStruct.DMA_BufferSize         = 0;                               //传输的字节数,初始化时不传输
    DMA_Init(DMA1_Stream7, &DMA_InitStruct);                                 //初始化
    /* DMA中断配置 */
    /* RxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Stream0_IRQn;  //中断名
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;               //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;               //子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;             //使能
    NVIC_Init(&NVIC_InitStruct);                                            //初始化
    /* TxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Stream7_IRQn;  //中断名
    NVIC_Init(&NVIC_InitStruct);                                            //初始化
    
    DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);  //DMA中断使能,传输完成中断
    DMA_ITConfig(DMA1_Stream7, DMA_IT_TC, ENABLE);  //DMA中断使能,传输完成中断
    /* 启动传输 */
    DMA_Cmd(DMA1_Stream0, DISABLE);  //禁止DMA传输,Rx
    DMA_Cmd(DMA1_Stream7, DISABLE);  //禁止DMA传输,Tx
}
/**
* @brief  串口中断
* @attention 
*/
void UART5_IRQHandler(void)
{
    uint16_t len = 0;  //数据长度
    
    if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
    {
        USART_ClearITPendingBit(UART5, USART_IT_IDLE);

        DMA_Cmd(DMA1_Stream0, DISABLE);     //关闭接收DMA
        len = sizeof(Uart5DMA.Rx) - DMA_GetCurrDataCounter(DMA1_Stream0);  //获取接收数据长度
        
        Uart5Receive(Uart5DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA1_Stream0, UART_DMA_RX_SIZE);  //设置DMA接收长度
        DMA_Cmd(DMA1_Stream0, ENABLE);     //使能接收DMA
        
        //读DR避免频繁进中断
        len = UART5->DR;
        len = UART5->SR;
    }
}
/**
* @brief 串口 DMA接收中断
*/
void DMA1_Stream0_IRQHandler(void)
{
    uint16_t len = 0;  //数据长度
    
    if(DMA_GetITStatus(DMA1_Stream0, DMA_IT_TCIF0) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);
        
        DMA_Cmd(DMA1_Stream0, DISABLE);     //关闭接收DMA
        len = sizeof(Uart5DMA.Rx) - DMA_GetCurrDataCounter(DMA1_Stream0);  //获取接收数据长度
        
        Uart5Receive(Uart5DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA1_Stream0, UART_DMA_RX_SIZE);  //设置DMA接收长度
        DMA_Cmd(DMA1_Stream0, ENABLE);     //使能接收DMA
        
        //读DR避免频繁进中断
        len = UART5->DR;
        len = UART5->SR;
    }
}
/**
* @brief 串口 DMA发送中断
*/
void DMA1_Stream7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream7, DMA_IT_TCIF7) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7);

        DMA_Cmd(DMA1_Stream7, DISABLE);     //关闭接收DMA
        DMA_SetCurrDataCounter(DMA1_Stream7, 0);  //清除数据长度
    }
}
/**
* @brief     串口发送数据
*/
uint16_t Uart5Send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(USART_GetFlagStatus(UART5, USART_FLAG_TC) != SET)  //上次发送未完成
    {
        return 0;
    }

    if(len > UART_DMA_TX_SIZE)  //长度超出DMA缓存区
    {
        len = UART_DMA_TX_SIZE;
    }
    i = len;
    while(i--)
    {
        Uart5DMA.Tx[i] = data[i];
    }

    DMA_Cmd(DMA1_Stream7, DISABLE);  //关闭DMA传输,DMA传输数据需先关闭,写入长度,再启动
    DMA_SetCurrDataCounter(DMA1_Stream7, len);  //重新写入传输数据长度
    DMA_Cmd(DMA1_Stream7, ENABLE);  //启动DMA传输
    
    return len;
}
/*串口6*************************************************************************************************/
/**
* @brief  串口初始化
* @attention 使用了DMA功能,共开启三个中断,开启Tx和Rx的DMA传输完成中断,判断DMA传输完成
*            开启串口总线空闲中断用于判断一帧数据接收完成
*/
void Usart6Init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    GPIO_StructInit(&GPIO_InitStruct);
    USART_StructInit(&USART_InitStruct);
    DMA_StructInit(&DMA_InitStruct);
    DMA_DeInit(DMA2_Stream1);
    DMA_DeInit(DMA2_Stream6);
    /* 使能时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    /* GPIO配置 */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;  //通道
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       //模式
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO口驱动电路响应速度
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //输出模式
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //上下拉
    GPIO_Init(GPIOC, &GPIO_InitStruct);              //初始化
    
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
    /* 串口配置 */
    USART_InitStruct.USART_BaudRate            = USART6_BAUD;                     //波特率
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;             //8个数据位
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;                //1个停止位
    USART_InitStruct.USART_Parity              = USART_Parity_No;                 //无奇偶校验
    USART_InitStruct.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;   //收发模式
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件数据流控制
    USART_Init(USART6, &USART_InitStruct);                                         //初始化

    USART_Cmd(USART6, ENABLE);                                                    //使能
    USART_DMACmd(USART6, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);              //使能DMA串口收发
    /* 串口中断初始化 */
    NVIC_InitStruct.NVIC_IRQChannel                   = USART6_IRQn;  //中断名
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;         //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x03;         //子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;       //使能
    NVIC_Init(&NVIC_InitStruct);                                      //初始化
    
    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);  //使能总线空闲中断,判断一帧数据接受完成
    /* DMA配置 */
    /* RxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_5;                   //通道
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);         //外设寄存器(多重模式)基地址
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Usart6DMA.Rx;          //存储器基地址
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralToMemory;      //DMA传输方向,外设到存储器
    DMA_InitStruct.DMA_BufferSize         = UART_DMA_RX_SIZE;                //传输的字节数
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //外设地址递增,关闭
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //存储器地址递增,使能
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //外设数据宽度,按字节对齐,8位
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;         //存储器数据宽度,按字节对齐,8位
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;                 //非循环模式
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;               //优先级
    DMA_InitStruct.DMA_FIFOMode           = DMA_FIFOMode_Disable;            //FIFO,关闭
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;  //FIFO阈值级别,1/4
    DMA_InitStruct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;          //存储器突发传输配置,单次传输
    DMA_InitStruct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;      //外设突发传输配置,单次传输
    DMA_Init(DMA2_Stream1, &DMA_InitStruct);                                 //初始化
    /* TxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_5;                   //通道
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Usart6DMA.Tx;          //存储器基地址
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;      //DMA传输方向,存储器到外设
    DMA_InitStruct.DMA_BufferSize         = 0;                               //传输的字节数,初始化时不传输
    DMA_Init(DMA2_Stream6, &DMA_InitStruct);                                 //初始化
    /* DMA中断配置 */
    /* RxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream1_IRQn;  //中断名
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;               //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x03;               //子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;             //使能
    NVIC_Init(&NVIC_InitStruct);                                            //初始化
    /* TxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream6_IRQn;  //中断名
    NVIC_Init(&NVIC_InitStruct);                                            //初始化
    
    DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);  //DMA中断使能,传输完成中断
    DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);  //DMA中断使能,传输完成中断
    /* 启动传输 */
    DMA_Cmd(DMA2_Stream1, ENABLE);  //启动DMA传输,Rx
    DMA_Cmd(DMA2_Stream6, DISABLE);  //禁止DMA传输,Tx
}
/**
* @brief  串口中断
* @attention 
*/
void USART6_IRQHandler(void)
{
    uint16_t len = 0;  //数据长度
    
    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
    {
        USART_ClearITPendingBit(USART6, USART_IT_IDLE);
        
        DMA_Cmd(DMA2_Stream1, DISABLE);     //关闭接收DMA
        len = sizeof(Usart6DMA.Rx) - DMA_GetCurrDataCounter(DMA2_Stream1);  //获取接收数据长度
        
        Usart6Receive(Usart6DMA.Rx, len);  //数据输出
        
        DMA_SetCurrDataCounter(DMA2_Stream1, UART_DMA_RX_SIZE);  //设置DMA接收长度
        DMA_Cmd(DMA2_Stream1, ENABLE);     //使能接收DMA
        
        //读DR避免频繁进中断
        len = USART6->DR;
        len = USART6->SR;
    }
}
/**
* @brief 串口 DMA接收中断
*/
void DMA2_Stream1_IRQHandler(void)
{
    uint16_t len = 0;  //数据长度
    
    if(DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
        
        DMA_Cmd(DMA2_Stream1, DISABLE);     //关闭接收DMA
        len = sizeof(Usart6DMA.Rx) - DMA_GetCurrDataCounter(DMA2_Stream1);  //获取接收数据长度

        Usart6Receive(Usart6DMA.Rx, len);  //数据入队
        
        DMA_SetCurrDataCounter(DMA2_Stream1, UART_DMA_RX_SIZE);  //设置DMA接收长度
        DMA_Cmd(DMA2_Stream1, ENABLE);     //使能接收DMA
        USART_ClearITPendingBit(USART6, USART_IT_IDLE);
        
        //读DR避免频繁进中断
        len = USART6->DR;
        len = USART6->SR;
    }
}
/**
* @brief 串口 DMA发送中断
*/
void DMA2_Stream6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TCIF6);

        DMA_Cmd(DMA2_Stream6, DISABLE);     //关闭DMA
        DMA_SetCurrDataCounter(DMA2_Stream6, 0);  //清除数据长度
    }
}
/**
* @brief 串口发送数据
*/
uint16_t Usart6Send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(USART_GetFlagStatus(USART6, USART_FLAG_TC) != SET)  //上次发送未完成
    {
        return 0;
    }

    if(len > UART_DMA_TX_SIZE)  //长度超出DMA缓存区
    {
        len = UART_DMA_TX_SIZE;
    }
    i = len;
    while(i--)
    {
        Usart6DMA.Tx[i] = data[i];
    }

    DMA_Cmd(DMA2_Stream6, DISABLE);  //关闭DMA传输,DMA传输数据需先关闭,写入长度,再启动
    DMA_SetCurrDataCounter(DMA2_Stream6, len);  //重新写入传输数据长度
    DMA_Cmd(DMA2_Stream6, ENABLE);  //启动DMA传输

    return len;
}
