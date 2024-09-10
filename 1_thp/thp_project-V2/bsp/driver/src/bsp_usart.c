/**@file   bsp_usart.c
* @brief   ��������
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

#define UART3_BAUD  115200  //���ڲ�����
#define UART4_BAUD  115200  //���ڲ�����
#define UART5_BAUD  115200  //���ڲ�����
#define UART6_BAUD  115200  //���ڲ�����
#define UART7_BAUD  115200  //���ڲ�����

#ifndef TEST_PRINT
#define UART8_BAUD  115200  //���ڲ�����
#else
#define UART8_BAUD  1000000  //���ڲ�����
#endif

/**
* @struct  uart_handle_t
* @brief   ���ھ��
*/
typedef struct
{
    UART_HandleTypeDef uart;
    DMA_HandleTypeDef  dma_tx;
    DMA_HandleTypeDef  dma_rx;
}uart_handle_t;
/**
* @struct  uart_dma_buf_t
* @brief   ����dma����
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
uart_handle_t uart3_handle;  //����3���
uart_handle_t uart4_handle;  //����4���
uart_handle_t uart5_handle;  //����5���
uart_handle_t uart6_handle;  //����6���
uart_handle_t uart7_handle;  //����7���
#endif
uart_handle_t uart8_handle;  //����8���
uart_dma_buf_t uart_dma_buf __attribute__((at(0x38000300)));  //����DMA����

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

#ifdef UART_DMA
/*����3*************************************************************************************************/
/**
* @brief  ���ڳ�ʼ��
* @attention ʹ����DMA����,�����������ж�,����Tx��Rx��DMA��������ж�,�ж�DMA�������
*            �����������߿����ж������ж�һ֡���ݽ������
*/
void uart3_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_UART_DeInit(&uart3_handle.uart);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    /* IO���� */
    gpio_init_struct.Pin       = GPIO_PIN_11; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //ģʽ
    gpio_init_struct.Pull      = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF7_USART3;         //����
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);  //��ʼ��
    gpio_init_struct.Pin       = GPIO_PIN_8; //���ź�
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);  //��ʼ��
    /* ����DMA���� */
    /* Tx DMA���� */
    uart3_handle.dma_tx.Instance                 = DMA1_Stream0;             //������ѡ��
    uart3_handle.dma_tx.Init.Request             = DMA_REQUEST_USART3_TX;    //�������ã��������ĸ����������
    uart3_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //���䷽��
    uart3_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //���裨�ǣ�����ģʽ
    uart3_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //�洢�����ǣ�����ģʽ
    uart3_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //�������ݴ�С�� 8/16/32 λ
    uart3_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //�洢�����ݴ�С�� 8/16/32 λ
    uart3_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //ģʽ����������ģʽ/ѭ��ģʽ/��ͨģʽ
    uart3_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA ���ȼ�����/��/��/�ǳ���
    uart3_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO ģʽ�������߽�ֹ
    uart3_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO ��ֵѡ��
    uart3_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //�洢��ͻ��ģʽ������/4 ������/8 ������/16 ������
    uart3_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //����ͻ��ģʽ������/4 ������/8 ������/16 ������
    HAL_DMA_DeInit(&uart3_handle.dma_tx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart3_handle.dma_tx);    //��ʼ��
    __HAL_LINKDMA(&uart3_handle.uart, hdmatx, uart3_handle.dma_tx);  //��DMA��UART��ϵ����
    /* Rx DMA���� */
    uart3_handle.dma_rx = uart3_handle.dma_tx;  //����������ͬ
    uart3_handle.dma_rx.Instance                 = DMA1_Stream1;             //������ѡ��
    uart3_handle.dma_rx.Init.Request             = DMA_REQUEST_USART3_RX;     //�������ã��������ĸ����������
    uart3_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //���䷽��
    HAL_DMA_DeInit(&uart3_handle.dma_rx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart3_handle.dma_rx);    //��ʼ��
    __HAL_LINKDMA(&uart3_handle.uart, hdmarx, uart3_handle.dma_rx);  //��DMA��UART��ϵ����
    /* �������� */
    uart3_handle.uart.Instance            = USART3;                //UART �Ĵ�������ַ
    uart3_handle.uart.Init.BaudRate       = UART3_BAUD;            //������
    uart3_handle.uart.Init.WordLength     = UART_WORDLENGTH_8B;    //�ֳ�
    uart3_handle.uart.Init.StopBits       = UART_STOPBITS_1;       //ֹͣλ
    uart3_handle.uart.Init.Parity         = UART_PARITY_NONE;      //У��λ
    uart3_handle.uart.Init.Mode           = UART_MODE_TX_RX;       //UART ģʽ
    uart3_handle.uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;   //Ӳ��������
    uart3_handle.uart.Init.OverSampling   = UART_OVERSAMPLING_16;  //����������
    uart3_handle.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  //����λ����ѡ��
    uart3_handle.uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;   //ʱ��Դ��Ԥ��Ƶֵ
    HAL_UART_Init(&uart3_handle.uart);  //��ʼ��
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);          //ʹ���ж�
    /* �����ж����� */
    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);  //��ռ���ȼ�3�������ȼ�3
    HAL_NVIC_EnableIRQ(USART3_IRQn);          //ʹ���ж�
    __HAL_UART_ENABLE_IT(&uart3_handle.uart, UART_IT_IDLE);  //�������ڿ����ж�
    /* ���� */
    HAL_UART_Receive_DMA(&uart3_handle.uart, uart_dma_buf.u3_rx, UART_DMA_RX_SIZE);  //rxdma������������dma�ж�
}
/**
* @brief  �����ж�
* @attention 
*/
void USART3_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart3_handle.uart;

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //�����ж�
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart3_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u3_rx, UART_DMA_RX_SIZE); //���¹���DMA
        //��DR����Ƶ�����ж�
        len = USART3->RDR;
        len = USART3->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //����HAL���жϴ����ú���
    }
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Stream0_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart3_handle.uart.hdmatx;

    HAL_DMA_IRQHandler(handle);  //����HAL���жϴ����ú���
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Stream1_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart3_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart3_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u3_rx, UART_DMA_RX_SIZE); //���¹���DMA
        
        //��DR����Ƶ�����ж�
        len = USART3->RDR;
        len = USART3->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //����HAL���жϴ����ú���
    }
}
/**
* @brief ���ڷ�������
*/
uint16_t uart3_send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;

    if(len == 0)
    {
        return 0;
    }
    if(__HAL_UART_GET_FLAG(&uart3_handle.uart, USART_FLAG_TC) != SET)  //�ϴη���δ���
    {
        return 0;
    }

    if(len > UART_DMA_TX_SIZE)  //���ȳ���DMA������
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
/*����3================================================================================================*/
/*����4*************************************************************************************************/
/**
* @brief  ���ڳ�ʼ��
* @attention ʹ����DMA����,�����������ж�,����Tx��Rx��DMA��������ж�,�ж�DMA�������
*            �����������߿����ж������ж�һ֡���ݽ������
*/
void uart4_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_UART_DeInit(&uart4_handle.uart);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    /* IO���� */
    gpio_init_struct.Pin       = GPIO_PIN_0 | GPIO_PIN_1; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //ģʽ
    gpio_init_struct.Pull      = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF8_UART4;          //����
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);  //��ʼ��
    /* ����DMA���� */
    /* Tx DMA���� */
    uart4_handle.dma_tx.Instance                 = DMA1_Stream2;             //������ѡ��
    uart4_handle.dma_tx.Init.Request             = DMA_REQUEST_UART4_TX;     //�������ã��������ĸ����������
    uart4_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //���䷽��
    uart4_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //���裨�ǣ�����ģʽ
    uart4_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //�洢�����ǣ�����ģʽ
    uart4_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //�������ݴ�С�� 8/16/32 λ
    uart4_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //�洢�����ݴ�С�� 8/16/32 λ
    uart4_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //ģʽ����������ģʽ/ѭ��ģʽ/��ͨģʽ
    uart4_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA ���ȼ�����/��/��/�ǳ���
    uart4_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO ģʽ�������߽�ֹ
    uart4_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO ��ֵѡ��
    uart4_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //�洢��ͻ��ģʽ������/4 ������/8 ������/16 ������
    uart4_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //����ͻ��ģʽ������/4 ������/8 ������/16 ������
    HAL_DMA_DeInit(&uart4_handle.dma_tx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart4_handle.dma_tx);    //��ʼ��
    __HAL_LINKDMA(&uart4_handle.uart, hdmatx, uart4_handle.dma_tx);  //��DMA��UART��ϵ����
    /* Rx DMA���� */
    uart4_handle.dma_rx = uart4_handle.dma_tx;  //����������ͬ
    uart4_handle.dma_rx.Instance                 = DMA1_Stream3;             //������ѡ��
    uart4_handle.dma_rx.Init.Request             = DMA_REQUEST_UART4_RX;     //�������ã��������ĸ����������
    uart4_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //���䷽��
    HAL_DMA_DeInit(&uart4_handle.dma_rx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart4_handle.dma_rx);    //��ʼ��
    __HAL_LINKDMA(&uart4_handle.uart, hdmarx, uart4_handle.dma_rx);  //��DMA��UART��ϵ����
    /* �������� */
    uart4_handle.uart.Instance            = UART4;                 //UART �Ĵ�������ַ
    uart4_handle.uart.Init.BaudRate       = UART4_BAUD;            //������
    uart4_handle.uart.Init.WordLength     = UART_WORDLENGTH_8B;    //�ֳ�
    uart4_handle.uart.Init.StopBits       = UART_STOPBITS_1;       //ֹͣλ
    uart4_handle.uart.Init.Parity         = UART_PARITY_NONE;      //У��λ
    uart4_handle.uart.Init.Mode           = UART_MODE_TX_RX;       //UART ģʽ
    uart4_handle.uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;   //Ӳ��������
    uart4_handle.uart.Init.OverSampling   = UART_OVERSAMPLING_16;  //����������
    uart4_handle.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  //����λ����ѡ��
    uart4_handle.uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;   //ʱ��Դ��Ԥ��Ƶֵ
    HAL_UART_Init(&uart4_handle.uart);  //��ʼ��
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);          //ʹ���ж�
    /* �����ж����� */
    HAL_NVIC_SetPriority(UART4_IRQn, 3, 0);  //��ռ���ȼ�3�������ȼ�3
    HAL_NVIC_EnableIRQ(UART4_IRQn);          //ʹ���ж�
    __HAL_UART_ENABLE_IT(&uart4_handle.uart, UART_IT_IDLE);  //�������ڿ����ж�
    /* ���� */
    HAL_UART_Receive_DMA(&uart4_handle.uart, uart_dma_buf.u4_rx, UART_DMA_RX_SIZE);  //rxdma������������dma�ж�
}
/**
* @brief  �����ж�
* @attention 
*/
void UART4_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart4_handle.uart;

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //�����ж�
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart4_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u4_rx, UART_DMA_RX_SIZE); //���¹���DMA
        //��DR����Ƶ�����ж�
        len = UART4->RDR;
        len = UART4->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //����HAL���жϴ����ú���
    }
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Stream2_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart4_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //����HAL���жϴ����ú���
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Stream3_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart4_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart4_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u4_rx, UART_DMA_RX_SIZE); //���¹���DMA
        
        //��DR����Ƶ�����ж�
        len = UART4->RDR;
        len = UART4->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //����HAL���жϴ����ú���
    }
}
/**
* @brief ���ڷ�������
*/
uint16_t uart4_send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_UART_GET_FLAG(&uart4_handle.uart, USART_FLAG_TC) != SET)  //�ϴη���δ���
    {
        return 0;
    }

    if(len > UART_DMA_TX_SIZE)  //���ȳ���DMA������
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
/*����4================================================================================================*/
/*����5*************************************************************************************************/
/**
* @brief  ���ڳ�ʼ��
* @attention ʹ����DMA����,�����������ж�,����Tx��Rx��DMA��������ж�,�ж�DMA�������
*            �����������߿����ж������ж�һ֡���ݽ������
*/
void uart5_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_UART_DeInit(&uart5_handle.uart);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();
    /* IO���� */
    gpio_init_struct.Pin       = GPIO_PIN_6; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //ģʽ
    gpio_init_struct.Pull      = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF14_UART5;         //����
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);  //��ʼ��
    gpio_init_struct.Pin       = GPIO_PIN_2; //���ź�
    gpio_init_struct.Alternate = GPIO_AF8_UART5;          //����
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);  //��ʼ��
    /* ����DMA���� */
    /* Tx DMA���� */
    uart5_handle.dma_tx.Instance                 = DMA1_Stream4;             //������ѡ��
    uart5_handle.dma_tx.Init.Request             = DMA_REQUEST_UART5_TX;     //�������ã��������ĸ����������
    uart5_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //���䷽��
    uart5_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //���裨�ǣ�����ģʽ
    uart5_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //�洢�����ǣ�����ģʽ
    uart5_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //�������ݴ�С�� 8/16/32 λ
    uart5_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //�洢�����ݴ�С�� 8/16/32 λ
    uart5_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //ģʽ����������ģʽ/ѭ��ģʽ/��ͨģʽ
    uart5_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA ���ȼ�����/��/��/�ǳ���
    uart5_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO ģʽ�������߽�ֹ
    uart5_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO ��ֵѡ��
    uart5_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //�洢��ͻ��ģʽ������/4 ������/8 ������/16 ������
    uart5_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //����ͻ��ģʽ������/4 ������/8 ������/16 ������
    HAL_DMA_DeInit(&uart5_handle.dma_tx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart5_handle.dma_tx);    //��ʼ��
    __HAL_LINKDMA(&uart5_handle.uart, hdmatx, uart5_handle.dma_tx);  //��DMA��UART��ϵ����
    /* Rx DMA���� */
    uart5_handle.dma_rx = uart5_handle.dma_tx;  //����������ͬ
    uart5_handle.dma_rx.Instance                 = DMA1_Stream5;             //������ѡ��
    uart5_handle.dma_rx.Init.Request             = DMA_REQUEST_UART5_RX;     //�������ã��������ĸ����������
    uart5_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //���䷽��
    HAL_DMA_DeInit(&uart5_handle.dma_rx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart5_handle.dma_rx);    //��ʼ��
    __HAL_LINKDMA(&uart5_handle.uart, hdmarx, uart5_handle.dma_rx);  //��DMA��UART��ϵ����
    /* �������� */
    uart5_handle.uart.Instance            = UART5;                 //UART �Ĵ�������ַ
    uart5_handle.uart.Init.BaudRate       = UART5_BAUD;            //������
    uart5_handle.uart.Init.WordLength     = UART_WORDLENGTH_8B;    //�ֳ�
    uart5_handle.uart.Init.StopBits       = UART_STOPBITS_1;       //ֹͣλ
    uart5_handle.uart.Init.Parity         = UART_PARITY_NONE;      //У��λ
    uart5_handle.uart.Init.Mode           = UART_MODE_TX_RX;       //UART ģʽ
    uart5_handle.uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;   //Ӳ��������
    uart5_handle.uart.Init.OverSampling   = UART_OVERSAMPLING_16;  //����������
    uart5_handle.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  //����λ����ѡ��
    uart5_handle.uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;   //ʱ��Դ��Ԥ��Ƶֵ
    HAL_UART_Init(&uart5_handle.uart);  //��ʼ��
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);          //ʹ���ж�
    /* �����ж����� */
    HAL_NVIC_SetPriority(UART5_IRQn, 3, 0);  //��ռ���ȼ�3�������ȼ�3
    HAL_NVIC_EnableIRQ(UART5_IRQn);          //ʹ���ж�
    __HAL_UART_ENABLE_IT(&uart5_handle.uart, UART_IT_IDLE);  //�������ڿ����ж�
    /* ���� */
    HAL_UART_Receive_DMA(&uart5_handle.uart, uart_dma_buf.u5_rx, UART_DMA_RX_SIZE);  //rxdma������������dma�ж�
}
/**
* @brief  �����ж�
* @attention 
*/
void UART5_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart5_handle.uart;

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //�����ж�
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart5_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u5_rx, UART_DMA_RX_SIZE); //���¹���DMA
        //��DR����Ƶ�����ж�
        len = UART5->RDR;
        len = UART5->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //����HAL���жϴ����ú���
    }
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Stream4_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart5_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //����HAL���жϴ����ú���
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Stream5_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart5_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart5_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u5_rx, UART_DMA_RX_SIZE); //���¹���DMA
        
        //��DR����Ƶ�����ж�
        len = UART5->RDR;
        len = UART5->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //����HAL���жϴ����ú���
    }
}
/**
* @brief ���ڷ�������
*/
uint16_t uart5_send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_UART_GET_FLAG(&uart5_handle.uart, USART_FLAG_TC) != SET)  //�ϴη���δ���
    {
        return 0;
    }

    if(len > UART_DMA_TX_SIZE)  //���ȳ���DMA������
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
/*����5================================================================================================*/
/*����6*************************************************************************************************/
/**
* @brief  ���ڳ�ʼ��
* @attention ʹ����DMA����,�����������ж�,����Tx��Rx��DMA��������ж�,�ж�DMA�������
*            �����������߿����ж������ж�һ֡���ݽ������
*/
void uart6_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_UART_DeInit(&uart6_handle.uart);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();
    /* IO���� */
    gpio_init_struct.Pin       = GPIO_PIN_6 | GPIO_PIN_7; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //ģʽ
    gpio_init_struct.Pull      = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF7_USART6;         //����
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);  //��ʼ��
    /* ����DMA���� */
    /* Tx DMA���� */
    uart6_handle.dma_tx.Instance                 = DMA1_Stream6;             //������ѡ��
    uart6_handle.dma_tx.Init.Request             = DMA_REQUEST_USART6_TX;    //�������ã��������ĸ����������
    uart6_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //���䷽��
    uart6_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //���裨�ǣ�����ģʽ
    uart6_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //�洢�����ǣ�����ģʽ
    uart6_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //�������ݴ�С�� 8/16/32 λ
    uart6_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //�洢�����ݴ�С�� 8/16/32 λ
    uart6_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //ģʽ����������ģʽ/ѭ��ģʽ/��ͨģʽ
    uart6_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA ���ȼ�����/��/��/�ǳ���
    uart6_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO ģʽ�������߽�ֹ
    uart6_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO ��ֵѡ��
    uart6_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //�洢��ͻ��ģʽ������/4 ������/8 ������/16 ������
    uart6_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //����ͻ��ģʽ������/4 ������/8 ������/16 ������
    HAL_DMA_DeInit(&uart6_handle.dma_tx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart6_handle.dma_tx);    //��ʼ��
    __HAL_LINKDMA(&uart6_handle.uart, hdmatx, uart6_handle.dma_tx);  //��DMA��UART��ϵ����
    /* Rx DMA���� */
    uart6_handle.dma_rx = uart6_handle.dma_tx;  //����������ͬ
    uart6_handle.dma_rx.Instance                 = DMA1_Stream7;             //������ѡ��
    uart6_handle.dma_rx.Init.Request             = DMA_REQUEST_USART6_RX;    //�������ã��������ĸ����������
    uart6_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //���䷽��
    HAL_DMA_DeInit(&uart6_handle.dma_rx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart6_handle.dma_rx);    //��ʼ��
    __HAL_LINKDMA(&uart6_handle.uart, hdmarx, uart6_handle.dma_rx);  //��DMA��UART��ϵ����
    /* �������� */
    uart6_handle.uart.Instance            = USART6;                //UART �Ĵ�������ַ
    uart6_handle.uart.Init.BaudRate       = UART6_BAUD;            //������
    uart6_handle.uart.Init.WordLength     = UART_WORDLENGTH_8B;    //�ֳ�
    uart6_handle.uart.Init.StopBits       = UART_STOPBITS_1;       //ֹͣλ
    uart6_handle.uart.Init.Parity         = UART_PARITY_NONE;      //У��λ
    uart6_handle.uart.Init.Mode           = UART_MODE_TX_RX;       //UART ģʽ
    uart6_handle.uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;   //Ӳ��������
    uart6_handle.uart.Init.OverSampling   = UART_OVERSAMPLING_16;  //����������
    uart6_handle.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  //����λ����ѡ��
    uart6_handle.uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;   //ʱ��Դ��Ԥ��Ƶֵ
    HAL_UART_Init(&uart6_handle.uart);  //��ʼ��
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);          //ʹ���ж�
    /* �����ж����� */
    HAL_NVIC_SetPriority(USART6_IRQn, 3, 0);  //��ռ���ȼ�3�������ȼ�3
    HAL_NVIC_EnableIRQ(USART6_IRQn);          //ʹ���ж�
    __HAL_UART_ENABLE_IT(&uart6_handle.uart, UART_IT_IDLE);  //�������ڿ����ж�
    /* ���� */
    HAL_UART_Receive_DMA(&uart6_handle.uart, uart_dma_buf.u6_rx, UART_DMA_RX_SIZE);  //rxdma������������dma�ж�
}
/**
* @brief  �����ж�
* @attention 
*/
void USART6_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart6_handle.uart;

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //�����ж�
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart6_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u6_rx, UART_DMA_RX_SIZE); //���¹���DMA
        //��DR����Ƶ�����ж�
        len = USART6->RDR;
        len = USART6->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //����HAL���жϴ����ú���
    }
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Stream6_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart6_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //����HAL���жϴ����ú���
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Stream7_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart6_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart6_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u6_rx, UART_DMA_RX_SIZE); //���¹���DMA
        
        //��DR����Ƶ�����ж�
        len = USART6->RDR;
        len = USART6->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //����HAL���жϴ����ú���
    }
}
/**
* @brief ���ڷ�������
*/
uint16_t uart6_send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_UART_GET_FLAG(&uart6_handle.uart, USART_FLAG_TC) != SET)  //�ϴη���δ���
    {
        return 0;
    }

    if(len > UART_DMA_TX_SIZE)  //���ȳ���DMA������
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
/*����6================================================================================================*/
/*����7*************************************************************************************************/
/**
* @brief  ���ڳ�ʼ��
* @attention ʹ����DMA����,�����������ж�,����Tx��Rx��DMA��������ж�,�ж�DMA�������
*            �����������߿����ж������ж�һ֡���ݽ������
*/
void uart7_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_UART_DeInit(&uart7_handle.uart);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_UART7_CLK_ENABLE();
    /* IO���� */
    gpio_init_struct.Pin       = GPIO_PIN_7 | GPIO_PIN_8; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //ģʽ
    gpio_init_struct.Pull      = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF7_UART7;          //����
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);  //��ʼ��
    /* ����DMA���� */
    /* Tx DMA���� */
    uart7_handle.dma_tx.Instance                 = DMA2_Stream0;             //������ѡ��
    uart7_handle.dma_tx.Init.Request             = DMA_REQUEST_UART7_TX;     //�������ã��������ĸ����������
    uart7_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //���䷽��
    uart7_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //���裨�ǣ�����ģʽ
    uart7_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //�洢�����ǣ�����ģʽ
    uart7_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //�������ݴ�С�� 8/16/32 λ
    uart7_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //�洢�����ݴ�С�� 8/16/32 λ
    uart7_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //ģʽ����������ģʽ/ѭ��ģʽ/��ͨģʽ
    uart7_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA ���ȼ�����/��/��/�ǳ���
    uart7_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO ģʽ�������߽�ֹ
    uart7_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO ��ֵѡ��
    uart7_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //�洢��ͻ��ģʽ������/4 ������/8 ������/16 ������
    uart7_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //����ͻ��ģʽ������/4 ������/8 ������/16 ������
    HAL_DMA_DeInit(&uart7_handle.dma_tx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart7_handle.dma_tx);    //��ʼ��
    __HAL_LINKDMA(&uart7_handle.uart, hdmatx, uart7_handle.dma_tx);  //��DMA��UART��ϵ����
    /* Rx DMA���� */
    uart7_handle.dma_rx = uart7_handle.dma_tx;  //����������ͬ
    uart7_handle.dma_rx.Instance                 = DMA2_Stream1;             //������ѡ��
    uart7_handle.dma_rx.Init.Request             = DMA_REQUEST_UART7_RX;     //�������ã��������ĸ����������
    uart7_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //���䷽��
    HAL_DMA_DeInit(&uart7_handle.dma_rx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart7_handle.dma_rx);    //��ʼ��
    __HAL_LINKDMA(&uart7_handle.uart, hdmarx, uart7_handle.dma_rx);  //��DMA��UART��ϵ����
    /* �������� */
    uart7_handle.uart.Instance            = UART7;                 //UART �Ĵ�������ַ
    uart7_handle.uart.Init.BaudRate       = UART7_BAUD;            //������
    uart7_handle.uart.Init.WordLength     = UART_WORDLENGTH_8B;    //�ֳ�
    uart7_handle.uart.Init.StopBits       = UART_STOPBITS_1;       //ֹͣλ
    uart7_handle.uart.Init.Parity         = UART_PARITY_NONE;      //У��λ
    uart7_handle.uart.Init.Mode           = UART_MODE_TX_RX;       //UART ģʽ
    uart7_handle.uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;   //Ӳ��������
    uart7_handle.uart.Init.OverSampling   = UART_OVERSAMPLING_16;  //����������
    uart7_handle.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  //����λ����ѡ��
    uart7_handle.uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;   //ʱ��Դ��Ԥ��Ƶֵ
    HAL_UART_Init(&uart7_handle.uart);  //��ʼ��
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);          //ʹ���ж�
    /* �����ж����� */
    HAL_NVIC_SetPriority(UART7_IRQn, 3, 0);  //��ռ���ȼ�3�������ȼ�3
    HAL_NVIC_EnableIRQ(UART7_IRQn);          //ʹ���ж�
    __HAL_UART_ENABLE_IT(&uart7_handle.uart, UART_IT_IDLE);  //�������ڿ����ж�
    /* ���� */
    HAL_UART_Receive_DMA(&uart7_handle.uart, uart_dma_buf.u7_rx, UART_DMA_RX_SIZE);  //rxdma������������dma�ж�
}
/**
* @brief  �����ж�
* @attention 
*/
void UART7_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart7_handle.uart;

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //�����ж�
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart7_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u7_rx, UART_DMA_RX_SIZE); //���¹���DMA
        //��DR����Ƶ�����ж�
        len = UART7->RDR;
        len = UART7->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //����HAL���жϴ����ú���
    }
}
/**
* @brief ���� DMA�����ж�
*/
void DMA2_Stream0_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart7_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //����HAL���жϴ����ú���
}
/**
* @brief ���� DMA�����ж�
*/
void DMA2_Stream1_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart7_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart7_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u7_rx, UART_DMA_RX_SIZE); //���¹���DMA
        
        //��DR����Ƶ�����ж�
        len = UART7->RDR;
        len = UART7->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //����HAL���жϴ����ú���
    }
}
/**
* @brief ���ڷ�������
*/
uint16_t uart7_send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_UART_GET_FLAG(&uart7_handle.uart, USART_FLAG_TC) != SET)  //�ϴη���δ���
    {
        return 0;
    }

    if(len > UART_DMA_TX_SIZE)  //���ȳ���DMA������
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
/*����7================================================================================================*/
#endif
/*����8*************************************************************************************************/
/**
* @brief  ���ڳ�ʼ��
* @attention ʹ����DMA����,�����������ж�,����Tx��Rx��DMA��������ж�,�ж�DMA�������
*            �����������߿����ж������ж�һ֡���ݽ������
*/
void uart8_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_UART_DeInit(&uart8_handle.uart);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_UART8_CLK_ENABLE();
    /* IO���� */
    gpio_init_struct.Pin       = GPIO_PIN_0 | GPIO_PIN_1; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //ģʽ
    gpio_init_struct.Pull      = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF8_UART8;          //����
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);  //��ʼ��
    /* ����DMA���� */
    /* Tx DMA���� */
    uart8_handle.dma_tx.Instance                 = DMA2_Stream2;             //������ѡ��
    uart8_handle.dma_tx.Init.Request             = DMA_REQUEST_UART8_TX;     //�������ã��������ĸ����������
    uart8_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //���䷽��
    uart8_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //���裨�ǣ�����ģʽ
    uart8_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //�洢�����ǣ�����ģʽ
    uart8_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //�������ݴ�С�� 8/16/32 λ
    uart8_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //�洢�����ݴ�С�� 8/16/32 λ
    uart8_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //ģʽ����������ģʽ/ѭ��ģʽ/��ͨģʽ
    uart8_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA ���ȼ�����/��/��/�ǳ���
    uart8_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO ģʽ�������߽�ֹ
    uart8_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO ��ֵѡ��
    uart8_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //�洢��ͻ��ģʽ������/4 ������/8 ������/16 ������
    uart8_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //����ͻ��ģʽ������/4 ������/8 ������/16 ������
    HAL_DMA_DeInit(&uart8_handle.dma_tx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart8_handle.dma_tx);    //��ʼ��
    __HAL_LINKDMA(&uart8_handle.uart, hdmatx, uart8_handle.dma_tx);  //��DMA��UART��ϵ����
    /* Rx DMA���� */
    uart8_handle.dma_rx = uart8_handle.dma_tx;  //����������ͬ
    uart8_handle.dma_rx.Instance                 = DMA2_Stream3;             //������ѡ��
    uart8_handle.dma_rx.Init.Request             = DMA_REQUEST_UART8_RX;     //�������ã��������ĸ����������
    uart8_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //���䷽��
    HAL_DMA_DeInit(&uart8_handle.dma_rx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart8_handle.dma_rx);    //��ʼ��
    __HAL_LINKDMA(&uart8_handle.uart, hdmarx, uart8_handle.dma_rx);  //��DMA��UART��ϵ����
    /* �������� */
    uart8_handle.uart.Instance            = UART8;                 //UART �Ĵ�������ַ
    uart8_handle.uart.Init.BaudRate       = UART8_BAUD;            //������
    uart8_handle.uart.Init.WordLength     = UART_WORDLENGTH_8B;    //�ֳ�
    uart8_handle.uart.Init.StopBits       = UART_STOPBITS_1;       //ֹͣλ
    uart8_handle.uart.Init.Parity         = UART_PARITY_NONE;      //У��λ
    uart8_handle.uart.Init.Mode           = UART_MODE_TX_RX;       //UART ģʽ
    uart8_handle.uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;   //Ӳ��������
    uart8_handle.uart.Init.OverSampling   = UART_OVERSAMPLING_16;  //����������
    uart8_handle.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  //����λ����ѡ��
    uart8_handle.uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;   //ʱ��Դ��Ԥ��Ƶֵ
    HAL_UART_Init(&uart8_handle.uart);  //��ʼ��
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);          //ʹ���ж�
    /* �����ж����� */
    HAL_NVIC_SetPriority(UART8_IRQn, 3, 0);  //��ռ���ȼ�3�������ȼ�3
    HAL_NVIC_EnableIRQ(UART8_IRQn);          //ʹ���ж�
    __HAL_UART_ENABLE_IT(&uart8_handle.uart, UART_IT_IDLE);  //�������ڿ����ж�
    /* ���� */
    HAL_UART_Receive_DMA(&uart8_handle.uart, uart_dma_buf.u8_rx, UART_DMA_RX_SIZE);  //rxdma������������dma�ж�
}
/**
* @brief  �����ж�
* @attention 
*/
void UART8_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart8_handle.uart;

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //�����ж�
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart8_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u8_rx, UART_DMA_RX_SIZE); //���¹���DMA
        //��DR����Ƶ�����ж�
        len = UART8->RDR;
        len = UART8->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //����HAL���жϴ����ú���
    }
}
/**
* @brief ���� DMA�����ж�
*/
void DMA2_Stream2_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart8_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //����HAL���жϴ����ú���
}
/**
* @brief ���� DMA�����ж�
*/
void DMA2_Stream3_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart8_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = UART_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart8_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u8_rx, UART_DMA_RX_SIZE); //���¹���DMA
        
        //��DR����Ƶ�����ж�
        len = UART8->RDR;
        len = UART8->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //����HAL���жϴ����ú���
    }
}
/**
* @brief ���ڷ�������
*/
uint16_t uart8_send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_UART_GET_FLAG(&uart8_handle.uart, USART_FLAG_TC) != SET)  //�ϴη���δ���
    {
        return 0;
    }

    if(len > UART_DMA_TX_SIZE)  //���ȳ���DMA������
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
/*����8================================================================================================*/
