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

#define UART1_BAUD  1000000  //串口波特率
#define UART2_BAUD  115200  //串口波特率
#define UART3_BAUD  115200  //串口波特率

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
    uint8_t u1_tx[UART1_DMA_SIZE];
    uint8_t u1_rx[UART1_DMA_SIZE];
    uint8_t u2_tx[UART2_DMA_SIZE];
    uint8_t u2_rx[UART2_DMA_SIZE];
    uint8_t u3_tx[UART3_DMA_SIZE];
    uint8_t u3_rx[UART3_DMA_SIZE];
}uart_dma_buf_t;

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

uart_handle_t uart1_handle;  //串口句柄
uart_handle_t uart2_handle;  //串口句柄
uart_handle_t uart3_handle;  //串口句柄

uart_dma_buf_t uart_dma_buf;  //串口DMA缓存

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/*串口1*************************************************************************************************/
/**
* @brief  串口初始化
* @attention 使用了DMA功能,共开启三个中断,开启Tx和Rx的DMA传输完成中断,判断DMA传输完成
*            开启串口总线空闲中断用于判断一帧数据接收完成
*/
void uart1_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_UART_DeInit(&uart1_handle.uart);
    /* 时钟使能 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    /* IO配置 */
    gpio_init_struct.Pin       = GPIO_PIN_9 | GPIO_PIN_10; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //模式
    gpio_init_struct.Pull      = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF7_USART1;         //复用
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);  //初始化
    /* 串口DMA配置 */
    /* Tx DMA配置 */
    uart1_handle.dma_tx.Instance                 = DMA1_Channel4;            //数据流选择
    uart1_handle.dma_tx.Init.Request             = DMA_REQUEST_2;            //请求设置，设置是哪个外设请求的
    uart1_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //传输方向
    uart1_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //外设（非）增量模式
    uart1_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //存储器（非）增量模式
    uart1_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //外设数据大小： 8/16/32 位
    uart1_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //存储器数据大小： 8/16/32 位
    uart1_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //模式：外设流控模式/循环模式/普通模式
    uart1_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA 优先级：低/中/高/非常高
    HAL_DMA_DeInit(&uart1_handle.dma_tx);  //取消初始化
    HAL_DMA_Init(&uart1_handle.dma_tx);    //初始化
    __HAL_LINKDMA(&uart1_handle.uart, hdmatx, uart1_handle.dma_tx);  //将DMA与UART联系起来
    /* Rx DMA配置 */
    uart1_handle.dma_rx = uart1_handle.dma_tx;  //其它设置相同
    uart1_handle.dma_rx.Instance                 = DMA1_Channel5;            //数据流选择
    uart1_handle.dma_rx.Init.Request             = DMA_REQUEST_2;            //请求设置，设置是哪个外设请求的
    uart1_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //传输方向
    HAL_DMA_DeInit(&uart1_handle.dma_rx);  //取消初始化
    HAL_DMA_Init(&uart1_handle.dma_rx);    //初始化
    __HAL_LINKDMA(&uart1_handle.uart, hdmarx, uart1_handle.dma_rx);  //将DMA与UART联系起来
    /* 串口配置 */
    uart1_handle.uart.Instance            = USART1;                //UART 寄存器基地址
    uart1_handle.uart.Init.BaudRate       = UART1_BAUD;            //波特率
    uart1_handle.uart.Init.WordLength     = UART_WORDLENGTH_8B;    //字长
    uart1_handle.uart.Init.StopBits       = UART_STOPBITS_1;       //停止位
    uart1_handle.uart.Init.Parity         = UART_PARITY_NONE;      //校验位
    uart1_handle.uart.Init.Mode           = UART_MODE_TX_RX;       //UART 模式
    uart1_handle.uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;   //硬件流设置
    uart1_handle.uart.Init.OverSampling   = UART_OVERSAMPLING_16;  //过采样设置
    uart1_handle.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  //采样位方法选择
    HAL_UART_Init(&uart1_handle.uart);  //初始化
    /* DMA中断配置 */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);          //使能中断
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);          //使能中断
    /* 启动 */
    HAL_UART_Receive_DMA(&uart1_handle.uart, uart_dma_buf.u1_rx, sizeof(uart_dma_buf.u1_rx));  //rxdma启动，开启各dma中断
    /* 串口中断配置 */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(USART1_IRQn);          //使能中断
    __HAL_UART_ENABLE_IT(&uart1_handle.uart, UART_IT_IDLE);  //开启串口空闲中断
}
/**
* @brief  串口中断
* @attention 
*/
void USART1_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart1_handle.uart;

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //空闲中断
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = sizeof(uart_dma_buf.u1_rx) - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart1_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u1_rx, sizeof(uart_dma_buf.u1_rx)); //重新关联DMA
        //读DR避免频繁进中断
        len = USART1->RDR;
        len = USART1->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //调用HAL库中断处理公用函数
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u1_rx, sizeof(uart_dma_buf.u1_rx)); //重新关联DMA
    }
}
/**
* @brief 串口 DMA发送中断
*/
void DMA1_Channel4_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart1_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //调用HAL库中断处理公用函数
}
/**
* @brief 串口 DMA接收中断
*/
void DMA1_Channel5_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart1_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = sizeof(uart_dma_buf.u1_rx) - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart1_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u1_rx, sizeof(uart_dma_buf.u1_rx)); //重新关联DMA
        
        //读DR避免频繁进中断
        len = USART1->RDR;
        len = USART1->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //调用HAL库中断处理公用函数
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u1_rx, sizeof(uart_dma_buf.u1_rx)); //重新关联DMA
    }
}
/**
* @brief 串口发送数据
*/
uint16_t uart1_send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_UART_GET_FLAG(&uart1_handle.uart, USART_FLAG_TC) != SET)  //上次发送未完成
    {
        return 0;
    }

    if(len > sizeof(uart_dma_buf.u1_tx))  //长度超出DMA缓存区
    {
        len = sizeof(uart_dma_buf.u1_tx);
    }
    i = len;
    while(i--)
    {
        uart_dma_buf.u1_tx[i] = data[i];
    }

    HAL_UART_Transmit_DMA(&uart1_handle.uart, uart_dma_buf.u1_tx, len);

    return len;
}
/*串口1================================================================================================*/
/*串口2*************************************************************************************************/
/**
* @brief  串口初始化
* @attention 使用了DMA功能,共开启三个中断,开启Tx和Rx的DMA传输完成中断,判断DMA传输完成
*            开启串口总线空闲中断用于判断一帧数据接收完成
*/
void uart2_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_UART_DeInit(&uart2_handle.uart);
    /* 时钟使能 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    /* IO配置 */
    gpio_init_struct.Pin       = GPIO_PIN_2 | GPIO_PIN_3; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //模式
    gpio_init_struct.Pull      = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF7_USART2;         //复用
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);  //初始化
    /* 串口DMA配置 */
    /* Tx DMA配置 */
    uart2_handle.dma_tx.Instance                 = DMA1_Channel7;            //数据流选择
    uart2_handle.dma_tx.Init.Request             = DMA_REQUEST_2;            //请求设置，设置是哪个外设请求的
    uart2_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //传输方向
    uart2_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //外设（非）增量模式
    uart2_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //存储器（非）增量模式
    uart2_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //外设数据大小： 8/16/32 位
    uart2_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //存储器数据大小： 8/16/32 位
    uart2_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //模式：外设流控模式/循环模式/普通模式
    uart2_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA 优先级：低/中/高/非常高
    HAL_DMA_DeInit(&uart2_handle.dma_tx);  //取消初始化
    HAL_DMA_Init(&uart2_handle.dma_tx);    //初始化
    __HAL_LINKDMA(&uart2_handle.uart, hdmatx, uart2_handle.dma_tx);  //将DMA与UART联系起来
    /* Rx DMA配置 */
    uart2_handle.dma_rx = uart2_handle.dma_tx;  //其它设置相同
    uart2_handle.dma_rx.Instance                 = DMA1_Channel6;            //数据流选择
    uart2_handle.dma_rx.Init.Request             = DMA_REQUEST_2;            //请求设置，设置是哪个外设请求的
    uart2_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //传输方向
    HAL_DMA_DeInit(&uart2_handle.dma_rx);  //取消初始化
    HAL_DMA_Init(&uart2_handle.dma_rx);    //初始化
    __HAL_LINKDMA(&uart2_handle.uart, hdmarx, uart2_handle.dma_rx);  //将DMA与UART联系起来
    /* 串口配置 */
    uart2_handle.uart.Instance            = USART2;                //UART 寄存器基地址
    uart2_handle.uart.Init.BaudRate       = UART2_BAUD;            //波特率
    uart2_handle.uart.Init.WordLength     = UART_WORDLENGTH_8B;    //字长
    uart2_handle.uart.Init.StopBits       = UART_STOPBITS_1;       //停止位
    uart2_handle.uart.Init.Parity         = UART_PARITY_NONE;      //校验位
    uart2_handle.uart.Init.Mode           = UART_MODE_TX_RX;       //UART 模式
    uart2_handle.uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;   //硬件流设置
    uart2_handle.uart.Init.OverSampling   = UART_OVERSAMPLING_16;  //过采样设置
    uart2_handle.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  //采样位方法选择
    HAL_UART_Init(&uart2_handle.uart);  //初始化
    /* DMA中断配置 */
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);          //使能中断
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);          //使能中断
    /* 串口中断配置 */
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(USART2_IRQn);          //使能中断
    __HAL_UART_ENABLE_IT(&uart2_handle.uart, UART_IT_IDLE);  //开启串口空闲中断
    /* 启动 */
    HAL_UART_Receive_DMA(&uart2_handle.uart, uart_dma_buf.u2_rx, sizeof(uart_dma_buf.u2_rx));  //rxdma启动，开启各dma中断
}
/**
* @brief  串口中断
* @attention 
*/
void USART2_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart2_handle.uart;

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //空闲中断
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = sizeof(uart_dma_buf.u2_rx) - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart2_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u2_rx, sizeof(uart_dma_buf.u2_rx)); //重新关联DMA
        //读DR避免频繁进中断
        len = USART2->RDR;
        len = USART2->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //调用HAL库中断处理公用函数
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u2_rx, sizeof(uart_dma_buf.u2_rx)); //重新关联DMA
    }
}
/**
* @brief 串口 DMA发送中断
*/
void DMA1_Channel7_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart2_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //调用HAL库中断处理公用函数
}
/**
* @brief 串口 DMA接收中断
*/
void DMA1_Channel6_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart2_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = sizeof(uart_dma_buf.u2_rx) - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart2_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u2_rx, sizeof(uart_dma_buf.u2_rx)); //重新关联DMA
        
        //读DR避免频繁进中断
        len = USART2->RDR;
        len = USART2->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //调用HAL库中断处理公用函数
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u2_rx, sizeof(uart_dma_buf.u2_rx)); //重新关联DMA
    }
}
/**
* @brief 串口发送数据
*/
uint16_t uart2_send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_UART_GET_FLAG(&uart2_handle.uart, USART_FLAG_TC) != SET)  //上次发送未完成
    {
        return 0;
    }

    if(len > sizeof(uart_dma_buf.u2_tx))  //长度超出DMA缓存区
    {
        len = sizeof(uart_dma_buf.u2_tx);
    }
    i = len;
    while(i--)
    {
        uart_dma_buf.u2_tx[i] = data[i];
    }

    HAL_UART_Transmit_DMA(&uart2_handle.uart, uart_dma_buf.u2_tx, len);

    return len;
}
/*串口2================================================================================================*/
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
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    /* IO配置 */
    gpio_init_struct.Pin       = GPIO_PIN_4 | GPIO_PIN_5; //引脚号
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //模式
    gpio_init_struct.Pull      = GPIO_PULLUP;             //上下拉电阻
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO口驱动电路响应速度
    gpio_init_struct.Alternate = GPIO_AF7_USART3;         //复用
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);  //初始化
    /* 串口DMA配置 */
    /* Tx DMA配置 */
    uart3_handle.dma_tx.Instance                 = DMA1_Channel2;            //数据流选择
    uart3_handle.dma_tx.Init.Request             = DMA_REQUEST_2;            //请求设置，设置是哪个外设请求的
    uart3_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //传输方向
    uart3_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //外设（非）增量模式
    uart3_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //存储器（非）增量模式
    uart3_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //外设数据大小： 8/16/32 位
    uart3_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //存储器数据大小： 8/16/32 位
    uart3_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //模式：外设流控模式/循环模式/普通模式
    uart3_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA 优先级：低/中/高/非常高
    HAL_DMA_DeInit(&uart3_handle.dma_tx);  //取消初始化
    HAL_DMA_Init(&uart3_handle.dma_tx);    //初始化
    __HAL_LINKDMA(&uart3_handle.uart, hdmatx, uart3_handle.dma_tx);  //将DMA与UART联系起来
    /* Rx DMA配置 */
    uart3_handle.dma_rx = uart3_handle.dma_tx;  //其它设置相同
    uart3_handle.dma_rx.Instance                 = DMA1_Channel3;            //数据流选择
    uart3_handle.dma_rx.Init.Request             = DMA_REQUEST_2;            //请求设置，设置是哪个外设请求的
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
    HAL_UART_Init(&uart3_handle.uart);  //初始化
    /* DMA中断配置 */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);          //使能中断
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);          //使能中断
    /* 串口中断配置 */
    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);  //抢占优先级，子优先级
    HAL_NVIC_EnableIRQ(USART3_IRQn);          //使能中断
    __HAL_UART_ENABLE_IT(&uart3_handle.uart, UART_IT_IDLE);  //开启串口空闲中断
    /* 启动 */
    HAL_UART_Receive_DMA(&uart3_handle.uart, uart_dma_buf.u3_rx, sizeof(uart_dma_buf.u3_rx));  //rxdma启动，开启各dma中断
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
        len = sizeof(uart_dma_buf.u3_rx) - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart3_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u3_rx, sizeof(uart_dma_buf.u3_rx)); //重新关联DMA
        //读DR避免频繁进中断
        len = USART3->RDR;
        len = USART3->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //调用HAL库中断处理公用函数
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u3_rx, sizeof(uart_dma_buf.u3_rx)); //重新关联DMA
    }
}
/**
* @brief 串口 DMA发送中断
*/
void DMA1_Channel2_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart3_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //调用HAL库中断处理公用函数
}
/**
* @brief 串口 DMA接收中断
*/
void DMA1_Channel3_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart3_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //传输完成中断
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //清除中断标志位
        
        HAL_UART_DMAStop(handle);  //停止DMA传送
        len = sizeof(uart_dma_buf.u3_rx) - __HAL_DMA_GET_COUNTER(handle->hdmarx); //获取接收的数据大小
        uart3_receive(handle->pRxBuffPtr, len);  //数据输出
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u3_rx, sizeof(uart_dma_buf.u3_rx)); //重新关联DMA
        
        //读DR避免频繁进中断
        len = USART3->RDR;
        len = USART3->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //调用HAL库中断处理公用函数
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u3_rx, sizeof(uart_dma_buf.u3_rx)); //重新关联DMA
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

    if(len > sizeof(uart_dma_buf.u3_tx))  //长度超出DMA缓存区
    {
        len = sizeof(uart_dma_buf.u3_tx);
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
