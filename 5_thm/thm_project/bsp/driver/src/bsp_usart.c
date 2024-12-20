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

//#ifndef TEST_PRINT
//#define UART8_BAUD  115200  //串口波特率
//#else
#define UART8_BAUD  1000000  //串口波特率
//#endif


/**
* @struct  uart_dma_buf_t
* @brief   串口dma缓存
*/
typedef struct
{
    uint8_t u8_tx[UART_DMA_TX_SIZE];
    uint8_t u8_rx[UART_DMA_RX_SIZE];
}uart_dma_buf_t;

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

uart_handle_t uart8_handle;  //串口8句柄
uart_dma_buf_t uart_dma_buf __attribute__((at(0x38000300)));  //串口DMA缓存

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/*串口8*************************************************************************************************/
/**
* @brief  串口初始化
* @attention 使用了DMA功能,共开启三个中断,开启Tx和Rx的DMA传输完成中断,判断DMA传输完成
*            开启串口总线空闲中断用于判断一帧数据接收完成
*/
void uart8_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_UART_DeInit(&uart8_handle.uart);
    /* 时钟使能 */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_UART8_CLK_ENABLE();
    /* IO配置 */
    gpio_init_struct.Pin       = GPIO_PIN_0 | GPIO_PIN_1; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //模式
    gpio_init_struct.Pull      = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF8_UART8;          //复用
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);  //初始化
    /* 串口DMA配置 */
    /* Tx DMA配置 */
    uart8_handle.dma_tx.Instance                 = DMA2_Stream2;             //数据流选择
    uart8_handle.dma_tx.Init.Request             = DMA_REQUEST_UART8_TX;     //请求设置，设置是哪个外设请求的
    uart8_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //传输方向
    uart8_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //外设（非）增量模式
    uart8_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //存储器（非）增量模式
    uart8_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //外设数据大小： 8/16/32 位
    uart8_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //存储器数据大小： 8/16/32 位
    uart8_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //模式：外设流控模式/循环模式/普通模式
    uart8_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA 优先级：低/中/高/非常高
    uart8_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO 模式开启或者禁止
    uart8_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO 阈值选择
    uart8_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //存储器突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    uart8_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //外设突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    HAL_DMA_DeInit(&uart8_handle.dma_tx);  //取消初始化
    HAL_DMA_Init(&uart8_handle.dma_tx);    //初始化
    __HAL_LINKDMA(&uart8_handle.uart, hdmatx, uart8_handle.dma_tx);  //将DMA与UART联系起来
    /* Rx DMA配置 */
    uart8_handle.dma_rx = uart8_handle.dma_tx;  //其它设置相同
    uart8_handle.dma_rx.Instance                 = DMA2_Stream3;             //数据流选择
    uart8_handle.dma_rx.Init.Request             = DMA_REQUEST_UART8_RX;     //请求设置，设置是哪个外设请求的
    uart8_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //传输方向
    HAL_DMA_DeInit(&uart8_handle.dma_rx);  //取消初始化
    HAL_DMA_Init(&uart8_handle.dma_rx);    //初始化
    __HAL_LINKDMA(&uart8_handle.uart, hdmarx, uart8_handle.dma_rx);  //将DMA与UART联系起来
    /* 串口配置 */
    uart8_handle.uart.Instance            = UART8;                 //UART 寄存器基地址
    uart8_handle.uart.Init.BaudRate       = UART8_BAUD;            //波特率
    uart8_handle.uart.Init.WordLength     = UART_WORDLENGTH_8B;    //字长
    uart8_handle.uart.Init.StopBits       = UART_STOPBITS_1;       //停止位
    uart8_handle.uart.Init.Parity         = UART_PARITY_NONE;      //校验位
    uart8_handle.uart.Init.Mode           = UART_MODE_TX_RX;       //UART 模式
    uart8_handle.uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;   //硬件流设置
    uart8_handle.uart.Init.OverSampling   = UART_OVERSAMPLING_16;  //过采样设置
    uart8_handle.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  //采样位方法选择
    uart8_handle.uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;   //时钟源的预分频值
    HAL_UART_Init(&uart8_handle.uart);  //初始化
    /* DMA中断配置 */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);          //使能中断
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);          //使能中断
    /* 串口中断配置 */
    HAL_NVIC_SetPriority(UART8_IRQn, 3, 0);  //抢占优先级3，子优先级3
    HAL_NVIC_EnableIRQ(UART8_IRQn);          //使能中断
    __HAL_UART_ENABLE_IT(&uart8_handle.uart, UART_IT_IDLE);  //开启串口空闲中断
    /* 启动 */
    HAL_UART_Receive_DMA(&uart8_handle.uart, uart_dma_buf.u8_rx, UART_DMA_RX_SIZE);  //rxdma启动，开启各dma中断
}
/**
* @brief  串口中断
* @attention 
*/
void UART8_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart8_handle.uart;

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //空闲中断
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart8_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u8_rx, UART_DMA_RX_SIZE); //重新关联DMA
        //读DR避免频繁进中断
        len = UART8->RDR;
        len = UART8->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //调用HAL库中断处理公用函数
    }
}
/**
* @brief 串口 DMA发送中断
*/
void DMA2_Stream2_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart8_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //调用HAL库中断处理公用函数
}
/**
* @brief 串口 DMA接收中断
*/
void DMA2_Stream3_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart8_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart8_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u8_rx, UART_DMA_RX_SIZE); //重新关联DMA
        
        //读DR避免频繁进中断
        len = UART8->RDR;
        len = UART8->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //调用HAL库中断处理公用函数
    }
}
/**
* @brief 串口发送数据
*/
uint16_t uart8_send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_UART_GET_FLAG(&uart8_handle.uart, USART_FLAG_TC) != SET)  //上次发送未完成
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
        uart_dma_buf.u8_tx[i] = data[i];
    }

    HAL_UART_Transmit_DMA(&uart8_handle.uart, uart_dma_buf.u8_tx, len);

    return len;
}
/**
* @brief  串口发送队列数据
*/
void uart8_send_q(queue_list_t *qp)
{
    uint32_t len = 0;
    
    len = queue_length(qp);
    
    if(len == 0)
    {
        return;
    }
    if(__HAL_UART_GET_FLAG(&uart8_handle.uart, USART_FLAG_TC) != SET)  //上次发送未完成
    {
        return;
    }

    if(len > UART_DMA_TX_SIZE)  //长度超出DMA缓存区
    {
        len = UART_DMA_TX_SIZE;
    }
    queue_output_u8(qp, uart_dma_buf.u8_tx, len);

    HAL_UART_Transmit_DMA(&uart8_handle.uart, uart_dma_buf.u8_tx, len);
}

/*串口8================================================================================================*/
