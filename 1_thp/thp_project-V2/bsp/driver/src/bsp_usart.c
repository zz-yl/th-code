/**@file   bsp_usart.c
* @brief   串口驱动
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_usart.h"
#include "bsp_cfg.h"
#include "thp_cfg.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define UART3_BAUD  115200  //串口波特率
#define UART4_BAUD  115200  //串口波特率
#define UART5_BAUD  115200  //串口波特率
#define UART6_BAUD  115200  //串口波特率
#define UART7_BAUD  115200  //串口波特率

#ifndef TEST_PRINT
#define UART8_BAUD  115200  //串口波特率
#else
#define UART8_BAUD  1000000  //串口波特率
#endif

/**
* @struct  uart_handle_t
* @brief   串口句柄
*/
typedef struct
{
    UART_HandleTypeDef uart;
    DMA_HandleTypeDef  dma_tx;
    DMA_HandleTypeDef  dma_rx;
}uart_handle_t;
/**
* @struct  uart_dma_buf_t
* @brief   串口dma缓存
*/
typedef struct
{
#ifdef UART_DMA
    uint8_t u3_tx[UART_DMA_TX_SIZE];
    uint8_t u3_rx[UART_DMA_RX_SIZE];
    uint8_t u4_tx[UART_DMA_TX_SIZE];
    uint8_t u4_rx[UART_DMA_RX_SIZE];
    uint8_t u5_tx[UART_DMA_TX_SIZE];
    uint8_t u5_rx[UART_DMA_RX_SIZE];
    uint8_t u6_tx[UART_DMA_TX_SIZE];
    uint8_t u6_rx[UART_DMA_RX_SIZE];
    uint8_t u7_tx[UART_DMA_TX_SIZE];
    uint8_t u7_rx[UART_DMA_RX_SIZE];
#endif
    uint8_t u8_tx[UART_DMA_TX_SIZE];
    uint8_t u8_rx[UART_DMA_RX_SIZE];
}uart_dma_buf_t;

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

#ifdef UART_DMA
uart_handle_t uart3_handle;  //串口3句柄
uart_handle_t uart4_handle;  //串口4句柄
uart_handle_t uart5_handle;  //串口5句柄
uart_handle_t uart6_handle;  //串口6句柄
uart_handle_t uart7_handle;  //串口7句柄
#endif
uart_handle_t uart8_handle;  //串口8句柄
uart_dma_buf_t uart_dma_buf __attribute__((at(0x38000300)));  //串口DMA缓存

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

#ifdef UART_DMA
/*串口3*************************************************************************************************/
/**
* @brief  串口初始化
* @attention 使用了DMA功能,共开启三个中断,开启Tx和Rx的DMA传输完成中断,判断DMA传输完成
*            开启串口总线空闲中断用于判断一帧数据接收完成
*/
void uart3_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_UART_DeInit(&uart3_handle.uart);
    /* 时钟使能 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    /* IO配置 */
    gpio_init_struct.Pin       = GPIO_PIN_11; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //模式
    gpio_init_struct.Pull      = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF7_USART3;         //复用
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);  //初始化
    gpio_init_struct.Pin       = GPIO_PIN_8; //引脚号
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);  //初始化
    /* 串口DMA配置 */
    /* Tx DMA配置 */
    uart3_handle.dma_tx.Instance                 = DMA1_Stream0;             //数据流选择
    uart3_handle.dma_tx.Init.Request             = DMA_REQUEST_USART3_TX;    //请求设置，设置是哪个外设请求的
    uart3_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //传输方向
    uart3_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //外设（非）增量模式
    uart3_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //存储器（非）增量模式
    uart3_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //外设数据大小： 8/16/32 位
    uart3_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //存储器数据大小： 8/16/32 位
    uart3_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //模式：外设流控模式/循环模式/普通模式
    uart3_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA 优先级：低/中/高/非常高
    uart3_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO 模式开启或者禁止
    uart3_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO 阈值选择
    uart3_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //存储器突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    uart3_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //外设突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    HAL_DMA_DeInit(&uart3_handle.dma_tx);  //取消初始化
    HAL_DMA_Init(&uart3_handle.dma_tx);    //初始化
    __HAL_LINKDMA(&uart3_handle.uart, hdmatx, uart3_handle.dma_tx);  //将DMA与UART联系起来
    /* Rx DMA配置 */
    uart3_handle.dma_rx = uart3_handle.dma_tx;  //其它设置相同
    uart3_handle.dma_rx.Instance                 = DMA1_Stream1;             //数据流选择
    uart3_handle.dma_rx.Init.Request             = DMA_REQUEST_USART3_RX;     //请求设置，设置是哪个外设请求的
    uart3_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //传输方向
    HAL_DMA_DeInit(&uart3_handle.dma_rx);  //取消初始化
    HAL_DMA_Init(&uart3_handle.dma_rx);    //初始化
    __HAL_LINKDMA(&uart3_handle.uart, hdmarx, uart3_handle.dma_rx);  //将DMA与UART联系起来
    /* 串口配置 */
    uart3_handle.uart.Instance            = USART3;                //UART 寄存器基地址
    uart3_handle.uart.Init.BaudRate       = UART3_BAUD;            //波特率
    uart3_handle.uart.Init.WordLength     = UART_WORDLENGTH_8B;    //字长
    uart3_handle.uart.Init.StopBits       = UART_STOPBITS_1;       //停止位
    uart3_handle.uart.Init.Parity         = UART_PARITY_NONE;      //校验位
    uart3_handle.uart.Init.Mode           = UART_MODE_TX_RX;       //UART 模式
    uart3_handle.uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;   //硬件流设置
    uart3_handle.uart.Init.OverSampling   = UART_OVERSAMPLING_16;  //过采样设置
    uart3_handle.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  //采样位方法选择
    uart3_handle.uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;   //时钟源的预分频值
    HAL_UART_Init(&uart3_handle.uart);  //初始化
    /* DMA中断配置 */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);          //使能中断
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);          //使能中断
    /* 串口中断配置 */
    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);  //抢占优先级3，子优先级3
    HAL_NVIC_EnableIRQ(USART3_IRQn);          //使能中断
    __HAL_UART_ENABLE_IT(&uart3_handle.uart, UART_IT_IDLE);  //开启串口空闲中断
    /* 启动 */
    HAL_UART_Receive_DMA(&uart3_handle.uart, uart_dma_buf.u3_rx, UART_DMA_RX_SIZE);  //rxdma启动，开启各dma中断
}
/**
* @brief  串口中断
* @attention 
*/
void USART3_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart3_handle.uart;

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //空闲中断
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart3_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u3_rx, UART_DMA_RX_SIZE); //重新关联DMA
        //读DR避免频繁进中断
        len = USART3->RDR;
        len = USART3->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //调用HAL库中断处理公用函数
    }
}
/**
* @brief 串口 DMA发送中断
*/
void DMA1_Stream0_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart3_handle.uart.hdmatx;

    HAL_DMA_IRQHandler(handle);  //调用HAL库中断处理公用函数
}
/**
* @brief 串口 DMA接收中断
*/
void DMA1_Stream1_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart3_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart3_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u3_rx, UART_DMA_RX_SIZE); //重新关联DMA
        
        //读DR避免频繁进中断
        len = USART3->RDR;
        len = USART3->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //调用HAL库中断处理公用函数
    }
}
/**
* @brief 串口发送数据
*/
uint16_t uart3_send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;

    if(len == 0)
    {
        return 0;
    }
    if(__HAL_UART_GET_FLAG(&uart3_handle.uart, USART_FLAG_TC) != SET)  //上次发送未完成
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
        uart_dma_buf.u3_tx[i] = data[i];
    }

    HAL_UART_Transmit_DMA(&uart3_handle.uart, uart_dma_buf.u3_tx, len);

    return len;
}
/*串口3================================================================================================*/
/*串口4*************************************************************************************************/
/**
* @brief  串口初始化
* @attention 使用了DMA功能,共开启三个中断,开启Tx和Rx的DMA传输完成中断,判断DMA传输完成
*            开启串口总线空闲中断用于判断一帧数据接收完成
*/
void uart4_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_UART_DeInit(&uart4_handle.uart);
    /* 时钟使能 */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    /* IO配置 */
    gpio_init_struct.Pin       = GPIO_PIN_0 | GPIO_PIN_1; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //模式
    gpio_init_struct.Pull      = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF8_UART4;          //复用
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);  //初始化
    /* 串口DMA配置 */
    /* Tx DMA配置 */
    uart4_handle.dma_tx.Instance                 = DMA1_Stream2;             //数据流选择
    uart4_handle.dma_tx.Init.Request             = DMA_REQUEST_UART4_TX;     //请求设置，设置是哪个外设请求的
    uart4_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //传输方向
    uart4_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //外设（非）增量模式
    uart4_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //存储器（非）增量模式
    uart4_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //外设数据大小： 8/16/32 位
    uart4_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //存储器数据大小： 8/16/32 位
    uart4_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //模式：外设流控模式/循环模式/普通模式
    uart4_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA 优先级：低/中/高/非常高
    uart4_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO 模式开启或者禁止
    uart4_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO 阈值选择
    uart4_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //存储器突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    uart4_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //外设突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    HAL_DMA_DeInit(&uart4_handle.dma_tx);  //取消初始化
    HAL_DMA_Init(&uart4_handle.dma_tx);    //初始化
    __HAL_LINKDMA(&uart4_handle.uart, hdmatx, uart4_handle.dma_tx);  //将DMA与UART联系起来
    /* Rx DMA配置 */
    uart4_handle.dma_rx = uart4_handle.dma_tx;  //其它设置相同
    uart4_handle.dma_rx.Instance                 = DMA1_Stream3;             //数据流选择
    uart4_handle.dma_rx.Init.Request             = DMA_REQUEST_UART4_RX;     //请求设置，设置是哪个外设请求的
    uart4_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //传输方向
    HAL_DMA_DeInit(&uart4_handle.dma_rx);  //取消初始化
    HAL_DMA_Init(&uart4_handle.dma_rx);    //初始化
    __HAL_LINKDMA(&uart4_handle.uart, hdmarx, uart4_handle.dma_rx);  //将DMA与UART联系起来
    /* 串口配置 */
    uart4_handle.uart.Instance            = UART4;                 //UART 寄存器基地址
    uart4_handle.uart.Init.BaudRate       = UART4_BAUD;            //波特率
    uart4_handle.uart.Init.WordLength     = UART_WORDLENGTH_8B;    //字长
    uart4_handle.uart.Init.StopBits       = UART_STOPBITS_1;       //停止位
    uart4_handle.uart.Init.Parity         = UART_PARITY_NONE;      //校验位
    uart4_handle.uart.Init.Mode           = UART_MODE_TX_RX;       //UART 模式
    uart4_handle.uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;   //硬件流设置
    uart4_handle.uart.Init.OverSampling   = UART_OVERSAMPLING_16;  //过采样设置
    uart4_handle.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  //采样位方法选择
    uart4_handle.uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;   //时钟源的预分频值
    HAL_UART_Init(&uart4_handle.uart);  //初始化
    /* DMA中断配置 */
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);          //使能中断
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);          //使能中断
    /* 串口中断配置 */
    HAL_NVIC_SetPriority(UART4_IRQn, 3, 0);  //抢占优先级3，子优先级3
    HAL_NVIC_EnableIRQ(UART4_IRQn);          //使能中断
    __HAL_UART_ENABLE_IT(&uart4_handle.uart, UART_IT_IDLE);  //开启串口空闲中断
    /* 启动 */
    HAL_UART_Receive_DMA(&uart4_handle.uart, uart_dma_buf.u4_rx, UART_DMA_RX_SIZE);  //rxdma启动，开启各dma中断
}
/**
* @brief  串口中断
* @attention 
*/
void UART4_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart4_handle.uart;

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //空闲中断
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart4_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u4_rx, UART_DMA_RX_SIZE); //重新关联DMA
        //读DR避免频繁进中断
        len = UART4->RDR;
        len = UART4->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //调用HAL库中断处理公用函数
    }
}
/**
* @brief 串口 DMA发送中断
*/
void DMA1_Stream2_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart4_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //调用HAL库中断处理公用函数
}
/**
* @brief 串口 DMA接收中断
*/
void DMA1_Stream3_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart4_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart4_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u4_rx, UART_DMA_RX_SIZE); //重新关联DMA
        
        //读DR避免频繁进中断
        len = UART4->RDR;
        len = UART4->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //调用HAL库中断处理公用函数
    }
}
/**
* @brief 串口发送数据
*/
uint16_t uart4_send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_UART_GET_FLAG(&uart4_handle.uart, USART_FLAG_TC) != SET)  //上次发送未完成
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
        uart_dma_buf.u4_tx[i] = data[i];
    }

    HAL_UART_Transmit_DMA(&uart4_handle.uart, uart_dma_buf.u4_tx, len);

    return len;
}
/*串口4================================================================================================*/
/*串口5*************************************************************************************************/
/**
* @brief  串口初始化
* @attention 使用了DMA功能,共开启三个中断,开启Tx和Rx的DMA传输完成中断,判断DMA传输完成
*            开启串口总线空闲中断用于判断一帧数据接收完成
*/
void uart5_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_UART_DeInit(&uart5_handle.uart);
    /* 时钟使能 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();
    /* IO配置 */
    gpio_init_struct.Pin       = GPIO_PIN_6; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //模式
    gpio_init_struct.Pull      = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF14_UART5;         //复用
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);  //初始化
    gpio_init_struct.Pin       = GPIO_PIN_2; //引脚号
    gpio_init_struct.Alternate = GPIO_AF8_UART5;          //复用
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);  //初始化
    /* 串口DMA配置 */
    /* Tx DMA配置 */
    uart5_handle.dma_tx.Instance                 = DMA1_Stream4;             //数据流选择
    uart5_handle.dma_tx.Init.Request             = DMA_REQUEST_UART5_TX;     //请求设置，设置是哪个外设请求的
    uart5_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //传输方向
    uart5_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //外设（非）增量模式
    uart5_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //存储器（非）增量模式
    uart5_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //外设数据大小： 8/16/32 位
    uart5_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //存储器数据大小： 8/16/32 位
    uart5_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //模式：外设流控模式/循环模式/普通模式
    uart5_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA 优先级：低/中/高/非常高
    uart5_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO 模式开启或者禁止
    uart5_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO 阈值选择
    uart5_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //存储器突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    uart5_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //外设突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    HAL_DMA_DeInit(&uart5_handle.dma_tx);  //取消初始化
    HAL_DMA_Init(&uart5_handle.dma_tx);    //初始化
    __HAL_LINKDMA(&uart5_handle.uart, hdmatx, uart5_handle.dma_tx);  //将DMA与UART联系起来
    /* Rx DMA配置 */
    uart5_handle.dma_rx = uart5_handle.dma_tx;  //其它设置相同
    uart5_handle.dma_rx.Instance                 = DMA1_Stream5;             //数据流选择
    uart5_handle.dma_rx.Init.Request             = DMA_REQUEST_UART5_RX;     //请求设置，设置是哪个外设请求的
    uart5_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //传输方向
    HAL_DMA_DeInit(&uart5_handle.dma_rx);  //取消初始化
    HAL_DMA_Init(&uart5_handle.dma_rx);    //初始化
    __HAL_LINKDMA(&uart5_handle.uart, hdmarx, uart5_handle.dma_rx);  //将DMA与UART联系起来
    /* 串口配置 */
    uart5_handle.uart.Instance            = UART5;                 //UART 寄存器基地址
    uart5_handle.uart.Init.BaudRate       = UART5_BAUD;            //波特率
    uart5_handle.uart.Init.WordLength     = UART_WORDLENGTH_8B;    //字长
    uart5_handle.uart.Init.StopBits       = UART_STOPBITS_1;       //停止位
    uart5_handle.uart.Init.Parity         = UART_PARITY_NONE;      //校验位
    uart5_handle.uart.Init.Mode           = UART_MODE_TX_RX;       //UART 模式
    uart5_handle.uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;   //硬件流设置
    uart5_handle.uart.Init.OverSampling   = UART_OVERSAMPLING_16;  //过采样设置
    uart5_handle.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  //采样位方法选择
    uart5_handle.uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;   //时钟源的预分频值
    HAL_UART_Init(&uart5_handle.uart);  //初始化
    /* DMA中断配置 */
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);          //使能中断
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);          //使能中断
    /* 串口中断配置 */
    HAL_NVIC_SetPriority(UART5_IRQn, 3, 0);  //抢占优先级3，子优先级3
    HAL_NVIC_EnableIRQ(UART5_IRQn);          //使能中断
    __HAL_UART_ENABLE_IT(&uart5_handle.uart, UART_IT_IDLE);  //开启串口空闲中断
    /* 启动 */
    HAL_UART_Receive_DMA(&uart5_handle.uart, uart_dma_buf.u5_rx, UART_DMA_RX_SIZE);  //rxdma启动，开启各dma中断
}
/**
* @brief  串口中断
* @attention 
*/
void UART5_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart5_handle.uart;

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //空闲中断
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart5_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u5_rx, UART_DMA_RX_SIZE); //重新关联DMA
        //读DR避免频繁进中断
        len = UART5->RDR;
        len = UART5->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //调用HAL库中断处理公用函数
    }
}
/**
* @brief 串口 DMA发送中断
*/
void DMA1_Stream4_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart5_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //调用HAL库中断处理公用函数
}
/**
* @brief 串口 DMA接收中断
*/
void DMA1_Stream5_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart5_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart5_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u5_rx, UART_DMA_RX_SIZE); //重新关联DMA
        
        //读DR避免频繁进中断
        len = UART5->RDR;
        len = UART5->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //调用HAL库中断处理公用函数
    }
}
/**
* @brief 串口发送数据
*/
uint16_t uart5_send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_UART_GET_FLAG(&uart5_handle.uart, USART_FLAG_TC) != SET)  //上次发送未完成
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
        uart_dma_buf.u5_tx[i] = data[i];
    }

    HAL_UART_Transmit_DMA(&uart5_handle.uart, uart_dma_buf.u5_tx, len);

    return len;
}
/*串口5================================================================================================*/
/*串口6*************************************************************************************************/
/**
* @brief  串口初始化
* @attention 使用了DMA功能,共开启三个中断,开启Tx和Rx的DMA传输完成中断,判断DMA传输完成
*            开启串口总线空闲中断用于判断一帧数据接收完成
*/
void uart6_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_UART_DeInit(&uart6_handle.uart);
    /* 时钟使能 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();
    /* IO配置 */
    gpio_init_struct.Pin       = GPIO_PIN_6 | GPIO_PIN_7; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //模式
    gpio_init_struct.Pull      = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF7_USART6;         //复用
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);  //初始化
    /* 串口DMA配置 */
    /* Tx DMA配置 */
    uart6_handle.dma_tx.Instance                 = DMA1_Stream6;             //数据流选择
    uart6_handle.dma_tx.Init.Request             = DMA_REQUEST_USART6_TX;    //请求设置，设置是哪个外设请求的
    uart6_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //传输方向
    uart6_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //外设（非）增量模式
    uart6_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //存储器（非）增量模式
    uart6_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //外设数据大小： 8/16/32 位
    uart6_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //存储器数据大小： 8/16/32 位
    uart6_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //模式：外设流控模式/循环模式/普通模式
    uart6_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA 优先级：低/中/高/非常高
    uart6_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO 模式开启或者禁止
    uart6_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO 阈值选择
    uart6_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //存储器突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    uart6_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //外设突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    HAL_DMA_DeInit(&uart6_handle.dma_tx);  //取消初始化
    HAL_DMA_Init(&uart6_handle.dma_tx);    //初始化
    __HAL_LINKDMA(&uart6_handle.uart, hdmatx, uart6_handle.dma_tx);  //将DMA与UART联系起来
    /* Rx DMA配置 */
    uart6_handle.dma_rx = uart6_handle.dma_tx;  //其它设置相同
    uart6_handle.dma_rx.Instance                 = DMA1_Stream7;             //数据流选择
    uart6_handle.dma_rx.Init.Request             = DMA_REQUEST_USART6_RX;    //请求设置，设置是哪个外设请求的
    uart6_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //传输方向
    HAL_DMA_DeInit(&uart6_handle.dma_rx);  //取消初始化
    HAL_DMA_Init(&uart6_handle.dma_rx);    //初始化
    __HAL_LINKDMA(&uart6_handle.uart, hdmarx, uart6_handle.dma_rx);  //将DMA与UART联系起来
    /* 串口配置 */
    uart6_handle.uart.Instance            = USART6;                //UART 寄存器基地址
    uart6_handle.uart.Init.BaudRate       = UART6_BAUD;            //波特率
    uart6_handle.uart.Init.WordLength     = UART_WORDLENGTH_8B;    //字长
    uart6_handle.uart.Init.StopBits       = UART_STOPBITS_1;       //停止位
    uart6_handle.uart.Init.Parity         = UART_PARITY_NONE;      //校验位
    uart6_handle.uart.Init.Mode           = UART_MODE_TX_RX;       //UART 模式
    uart6_handle.uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;   //硬件流设置
    uart6_handle.uart.Init.OverSampling   = UART_OVERSAMPLING_16;  //过采样设置
    uart6_handle.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  //采样位方法选择
    uart6_handle.uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;   //时钟源的预分频值
    HAL_UART_Init(&uart6_handle.uart);  //初始化
    /* DMA中断配置 */
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);          //使能中断
    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);          //使能中断
    /* 串口中断配置 */
    HAL_NVIC_SetPriority(USART6_IRQn, 3, 0);  //抢占优先级3，子优先级3
    HAL_NVIC_EnableIRQ(USART6_IRQn);          //使能中断
    __HAL_UART_ENABLE_IT(&uart6_handle.uart, UART_IT_IDLE);  //开启串口空闲中断
    /* 启动 */
    HAL_UART_Receive_DMA(&uart6_handle.uart, uart_dma_buf.u6_rx, UART_DMA_RX_SIZE);  //rxdma启动，开启各dma中断
}
/**
* @brief  串口中断
* @attention 
*/
void USART6_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart6_handle.uart;

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //空闲中断
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart6_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u6_rx, UART_DMA_RX_SIZE); //重新关联DMA
        //读DR避免频繁进中断
        len = USART6->RDR;
        len = USART6->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //调用HAL库中断处理公用函数
    }
}
/**
* @brief 串口 DMA发送中断
*/
void DMA1_Stream6_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart6_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //调用HAL库中断处理公用函数
}
/**
* @brief 串口 DMA接收中断
*/
void DMA1_Stream7_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart6_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart6_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u6_rx, UART_DMA_RX_SIZE); //重新关联DMA
        
        //读DR避免频繁进中断
        len = USART6->RDR;
        len = USART6->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //调用HAL库中断处理公用函数
    }
}
/**
* @brief 串口发送数据
*/
uint16_t uart6_send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_UART_GET_FLAG(&uart6_handle.uart, USART_FLAG_TC) != SET)  //上次发送未完成
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
        uart_dma_buf.u6_tx[i] = data[i];
    }

    HAL_UART_Transmit_DMA(&uart6_handle.uart, uart_dma_buf.u6_tx, len);

    return len;
}
/*串口6================================================================================================*/
/*串口7*************************************************************************************************/
/**
* @brief  串口初始化
* @attention 使用了DMA功能,共开启三个中断,开启Tx和Rx的DMA传输完成中断,判断DMA传输完成
*            开启串口总线空闲中断用于判断一帧数据接收完成
*/
void uart7_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_UART_DeInit(&uart7_handle.uart);
    /* 时钟使能 */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_UART7_CLK_ENABLE();
    /* IO配置 */
    gpio_init_struct.Pin       = GPIO_PIN_7 | GPIO_PIN_8; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //模式
    gpio_init_struct.Pull      = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF7_UART7;          //复用
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);  //初始化
    /* 串口DMA配置 */
    /* Tx DMA配置 */
    uart7_handle.dma_tx.Instance                 = DMA2_Stream0;             //数据流选择
    uart7_handle.dma_tx.Init.Request             = DMA_REQUEST_UART7_TX;     //请求设置，设置是哪个外设请求的
    uart7_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //传输方向
    uart7_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //外设（非）增量模式
    uart7_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //存储器（非）增量模式
    uart7_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //外设数据大小： 8/16/32 位
    uart7_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //存储器数据大小： 8/16/32 位
    uart7_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //模式：外设流控模式/循环模式/普通模式
    uart7_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA 优先级：低/中/高/非常高
    uart7_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO 模式开启或者禁止
    uart7_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO 阈值选择
    uart7_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //存储器突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    uart7_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //外设突发模式：单次/4 个节拍/8 个节拍/16 个节拍
    HAL_DMA_DeInit(&uart7_handle.dma_tx);  //取消初始化
    HAL_DMA_Init(&uart7_handle.dma_tx);    //初始化
    __HAL_LINKDMA(&uart7_handle.uart, hdmatx, uart7_handle.dma_tx);  //将DMA与UART联系起来
    /* Rx DMA配置 */
    uart7_handle.dma_rx = uart7_handle.dma_tx;  //其它设置相同
    uart7_handle.dma_rx.Instance                 = DMA2_Stream1;             //数据流选择
    uart7_handle.dma_rx.Init.Request             = DMA_REQUEST_UART7_RX;     //请求设置，设置是哪个外设请求的
    uart7_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //传输方向
    HAL_DMA_DeInit(&uart7_handle.dma_rx);  //取消初始化
    HAL_DMA_Init(&uart7_handle.dma_rx);    //初始化
    __HAL_LINKDMA(&uart7_handle.uart, hdmarx, uart7_handle.dma_rx);  //将DMA与UART联系起来
    /* 串口配置 */
    uart7_handle.uart.Instance            = UART7;                 //UART 寄存器基地址
    uart7_handle.uart.Init.BaudRate       = UART7_BAUD;            //波特率
    uart7_handle.uart.Init.WordLength     = UART_WORDLENGTH_8B;    //字长
    uart7_handle.uart.Init.StopBits       = UART_STOPBITS_1;       //停止位
    uart7_handle.uart.Init.Parity         = UART_PARITY_NONE;      //校验位
    uart7_handle.uart.Init.Mode           = UART_MODE_TX_RX;       //UART 模式
    uart7_handle.uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;   //硬件流设置
    uart7_handle.uart.Init.OverSampling   = UART_OVERSAMPLING_16;  //过采样设置
    uart7_handle.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  //采样位方法选择
    uart7_handle.uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;   //时钟源的预分频值
    HAL_UART_Init(&uart7_handle.uart);  //初始化
    /* DMA中断配置 */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);          //使能中断
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);          //使能中断
    /* 串口中断配置 */
    HAL_NVIC_SetPriority(UART7_IRQn, 3, 0);  //抢占优先级3，子优先级3
    HAL_NVIC_EnableIRQ(UART7_IRQn);          //使能中断
    __HAL_UART_ENABLE_IT(&uart7_handle.uart, UART_IT_IDLE);  //开启串口空闲中断
    /* 启动 */
    HAL_UART_Receive_DMA(&uart7_handle.uart, uart_dma_buf.u7_rx, UART_DMA_RX_SIZE);  //rxdma启动，开启各dma中断
}
/**
* @brief  串口中断
* @attention 
*/
void UART7_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart7_handle.uart;

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //空闲中断
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart7_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u7_rx, UART_DMA_RX_SIZE); //重新关联DMA
        //读DR避免频繁进中断
        len = UART7->RDR;
        len = UART7->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //调用HAL库中断处理公用函数
    }
}
/**
* @brief 串口 DMA发送中断
*/
void DMA2_Stream0_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart7_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //调用HAL库中断处理公用函数
}
/**
* @brief 串口 DMA接收中断
*/
void DMA2_Stream1_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart7_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart7_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u7_rx, UART_DMA_RX_SIZE); //重新关联DMA
        
        //读DR避免频繁进中断
        len = UART7->RDR;
        len = UART7->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //调用HAL库中断处理公用函数
    }
}
/**
* @brief 串口发送数据
*/
uint16_t uart7_send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_UART_GET_FLAG(&uart7_handle.uart, USART_FLAG_TC) != SET)  //上次发送未完成
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
        uart_dma_buf.u7_tx[i] = data[i];
    }

    HAL_UART_Transmit_DMA(&uart7_handle.uart, uart_dma_buf.u7_tx, len);

    return len;
}
/*串口7================================================================================================*/
#endif
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
/*串口8================================================================================================*/
