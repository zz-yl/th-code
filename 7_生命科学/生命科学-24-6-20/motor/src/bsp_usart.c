/**@file   bsp_usart.c
* @brief   ��������
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_usart.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define UART2_BAUD  38400  //���ڲ�����

#define UART2_485_EN(cmd)     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (cmd ? GPIO_PIN_SET : GPIO_PIN_RESET))

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
    uint8_t u2_tx[UART2_DMA_SIZE];
    uint8_t u2_rx[UART2_DMA_SIZE];
}uart_dma_buf_t;

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

uart_handle_t uart2_handle;  //���ھ��

uart_dma_buf_t uart_dma_buf;  //����DMA����

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

__weak void uart2_receive(uint8_t *data, uint16_t len)
{
    
}

/*����2*************************************************************************************************/
/**
* @brief  ���ڳ�ʼ��
* @attention ʹ����DMA����,�����������ж�,����Tx��Rx��DMA��������ж�,�ж�DMA�������
*            �����������߿����ж������ж�һ֡���ݽ������
*/
void uart2_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_UART_DeInit(&uart2_handle.uart);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    /* IO���� */
    gpio_init_struct.Pin       = GPIO_PIN_2 | GPIO_PIN_3; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //ģʽ
    gpio_init_struct.Pull      = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF7_USART2;         //����
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);  //��ʼ��
    /* ����DMA���� */
    /* Tx DMA���� */
    uart2_handle.dma_tx.Instance                 = DMA1_Stream6;             //������ѡ��
    uart2_handle.dma_tx.Init.Channel             = DMA_CHANNEL_4;            //ͨ��ѡ��
    uart2_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //���䷽��
    uart2_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //���裨�ǣ�����ģʽ
    uart2_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //�洢�����ǣ�����ģʽ
    uart2_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //�������ݴ�С�� 8/16/32 λ
    uart2_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //�洢�����ݴ�С�� 8/16/32 λ
    uart2_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //ģʽ����������ģʽ/ѭ��ģʽ/��ͨģʽ
    uart2_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA ���ȼ�����/��/��/�ǳ���
    HAL_DMA_DeInit(&uart2_handle.dma_tx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart2_handle.dma_tx);    //��ʼ��
    __HAL_LINKDMA(&uart2_handle.uart, hdmatx, uart2_handle.dma_tx);  //��DMA��UART��ϵ����
    /* Rx DMA���� */
    uart2_handle.dma_rx = uart2_handle.dma_tx;  //����������ͬ
    uart2_handle.dma_rx.Instance                 = DMA1_Stream5;            //������ѡ��
    uart2_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //���䷽��
    HAL_DMA_DeInit(&uart2_handle.dma_rx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart2_handle.dma_rx);    //��ʼ��
    __HAL_LINKDMA(&uart2_handle.uart, hdmarx, uart2_handle.dma_rx);  //��DMA��UART��ϵ����
    /* �������� */
    uart2_handle.uart.Instance            = USART2;                //UART �Ĵ�������ַ
    uart2_handle.uart.Init.BaudRate       = UART2_BAUD;            //������
    uart2_handle.uart.Init.WordLength     = UART_WORDLENGTH_8B;    //�ֳ�
    uart2_handle.uart.Init.StopBits       = UART_STOPBITS_1;       //ֹͣλ
    uart2_handle.uart.Init.Parity         = UART_PARITY_NONE;      //У��λ
    uart2_handle.uart.Init.Mode           = UART_MODE_TX_RX;       //UART ģʽ
    uart2_handle.uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;   //Ӳ��������
    uart2_handle.uart.Init.OverSampling   = UART_OVERSAMPLING_16;  //����������
    HAL_UART_Init(&uart2_handle.uart);  //��ʼ��
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);          //ʹ���ж�
    /* �����ж����� */
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(USART2_IRQn);          //ʹ���ж�
    __HAL_UART_ENABLE_IT(&uart2_handle.uart, UART_IT_IDLE);  //�������ڿ����ж�
    /* ���� */
    HAL_UART_Receive_DMA(&uart2_handle.uart, uart_dma_buf.u2_rx, sizeof(uart_dma_buf.u2_rx));  //rxdma������������dma�ж�
}
/**
* @brief  �����ж�
* @attention 
*/
void USART2_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart2_handle.uart;

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_TC) != RESET)  //��������ж�
    {
        UART2_485_EN(0);
    }
    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //�����ж�
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = sizeof(uart_dma_buf.u2_rx) - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart2_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u2_rx, sizeof(uart_dma_buf.u2_rx)); //���¹���DMA
        //��DR����Ƶ�����ж�
        len = USART2->DR;
        len = USART2->SR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //����HAL���жϴ����ú���
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u2_rx, sizeof(uart_dma_buf.u2_rx)); //���¹���DMA
    }
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Stream6_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart2_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //����HAL���жϴ����ú���
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Stream5_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart2_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = sizeof(uart_dma_buf.u2_rx) - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart2_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u2_rx, sizeof(uart_dma_buf.u2_rx)); //���¹���DMA
        
        //��DR����Ƶ�����ж�
        len = USART2->DR;
        len = USART2->SR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //����HAL���жϴ����ú���
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u2_rx, sizeof(uart_dma_buf.u2_rx)); //���¹���DMA
    }
}
/**
* @brief ���ڷ�������
*/
uint16_t uart2_send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(__HAL_UART_GET_FLAG(&uart2_handle.uart, UART_FLAG_TC) != SET)  //�ϴη���δ���
    {
        return 0;
    }
    
    if(len == 0)
    {
        return 0;
    }

    if(len > sizeof(uart_dma_buf.u2_tx))  //���ȳ���DMA������
    {
        len = sizeof(uart_dma_buf.u2_tx);
    }
    i = len;
    while(i--)
    {
        uart_dma_buf.u2_tx[i] = data[i];
    }

    UART2_485_EN(1);
    
    HAL_UART_Transmit_DMA(&uart2_handle.uart, uart_dma_buf.u2_tx, len);
    __HAL_UART_ENABLE_IT(&uart2_handle.uart, UART_IT_TC);  //�������ڷ�������ж�
    

    return len;
}
/*����2================================================================================================*/
