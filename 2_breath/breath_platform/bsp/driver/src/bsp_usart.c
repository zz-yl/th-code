/**@file   bsp_usart.c
* @brief   ��������
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

#define UART1_BAUD  1000000  //���ڲ�����
#define UART2_BAUD  115200  //���ڲ�����
#define UART3_BAUD  115200  //���ڲ�����

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

uart_handle_t uart1_handle;  //���ھ��
uart_handle_t uart2_handle;  //���ھ��
uart_handle_t uart3_handle;  //���ھ��

uart_dma_buf_t uart_dma_buf;  //����DMA����

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/*����1*************************************************************************************************/
/**
* @brief  ���ڳ�ʼ��
* @attention ʹ����DMA����,�����������ж�,����Tx��Rx��DMA��������ж�,�ж�DMA�������
*            �����������߿����ж������ж�һ֡���ݽ������
*/
void uart1_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_UART_DeInit(&uart1_handle.uart);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    /* IO���� */
    gpio_init_struct.Pin       = GPIO_PIN_9 | GPIO_PIN_10; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //ģʽ
    gpio_init_struct.Pull      = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF7_USART1;         //����
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);  //��ʼ��
    /* ����DMA���� */
    /* Tx DMA���� */
    uart1_handle.dma_tx.Instance                 = DMA1_Channel4;            //������ѡ��
    uart1_handle.dma_tx.Init.Request             = DMA_REQUEST_2;            //�������ã��������ĸ����������
    uart1_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //���䷽��
    uart1_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //���裨�ǣ�����ģʽ
    uart1_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //�洢�����ǣ�����ģʽ
    uart1_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //�������ݴ�С�� 8/16/32 λ
    uart1_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //�洢�����ݴ�С�� 8/16/32 λ
    uart1_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //ģʽ����������ģʽ/ѭ��ģʽ/��ͨģʽ
    uart1_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA ���ȼ�����/��/��/�ǳ���
    HAL_DMA_DeInit(&uart1_handle.dma_tx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart1_handle.dma_tx);    //��ʼ��
    __HAL_LINKDMA(&uart1_handle.uart, hdmatx, uart1_handle.dma_tx);  //��DMA��UART��ϵ����
    /* Rx DMA���� */
    uart1_handle.dma_rx = uart1_handle.dma_tx;  //����������ͬ
    uart1_handle.dma_rx.Instance                 = DMA1_Channel5;            //������ѡ��
    uart1_handle.dma_rx.Init.Request             = DMA_REQUEST_2;            //�������ã��������ĸ����������
    uart1_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //���䷽��
    HAL_DMA_DeInit(&uart1_handle.dma_rx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart1_handle.dma_rx);    //��ʼ��
    __HAL_LINKDMA(&uart1_handle.uart, hdmarx, uart1_handle.dma_rx);  //��DMA��UART��ϵ����
    /* �������� */
    uart1_handle.uart.Instance            = USART1;                //UART �Ĵ�������ַ
    uart1_handle.uart.Init.BaudRate       = UART1_BAUD;            //������
    uart1_handle.uart.Init.WordLength     = UART_WORDLENGTH_8B;    //�ֳ�
    uart1_handle.uart.Init.StopBits       = UART_STOPBITS_1;       //ֹͣλ
    uart1_handle.uart.Init.Parity         = UART_PARITY_NONE;      //У��λ
    uart1_handle.uart.Init.Mode           = UART_MODE_TX_RX;       //UART ģʽ
    uart1_handle.uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;   //Ӳ��������
    uart1_handle.uart.Init.OverSampling   = UART_OVERSAMPLING_16;  //����������
    uart1_handle.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  //����λ����ѡ��
    HAL_UART_Init(&uart1_handle.uart);  //��ʼ��
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);          //ʹ���ж�
    /* ���� */
    HAL_UART_Receive_DMA(&uart1_handle.uart, uart_dma_buf.u1_rx, sizeof(uart_dma_buf.u1_rx));  //rxdma������������dma�ж�
    /* �����ж����� */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(USART1_IRQn);          //ʹ���ж�
    __HAL_UART_ENABLE_IT(&uart1_handle.uart, UART_IT_IDLE);  //�������ڿ����ж�
}
/**
* @brief  �����ж�
* @attention 
*/
void USART1_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart1_handle.uart;

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //�����ж�
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = sizeof(uart_dma_buf.u1_rx) - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart1_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u1_rx, sizeof(uart_dma_buf.u1_rx)); //���¹���DMA
        //��DR����Ƶ�����ж�
        len = USART1->RDR;
        len = USART1->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //����HAL���жϴ����ú���
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u1_rx, sizeof(uart_dma_buf.u1_rx)); //���¹���DMA
    }
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Channel4_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart1_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //����HAL���жϴ����ú���
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Channel5_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart1_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = sizeof(uart_dma_buf.u1_rx) - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart1_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u1_rx, sizeof(uart_dma_buf.u1_rx)); //���¹���DMA
        
        //��DR����Ƶ�����ж�
        len = USART1->RDR;
        len = USART1->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //����HAL���жϴ����ú���
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u1_rx, sizeof(uart_dma_buf.u1_rx)); //���¹���DMA
    }
}
/**
* @brief ���ڷ�������
*/
uint16_t uart1_send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_UART_GET_FLAG(&uart1_handle.uart, USART_FLAG_TC) != SET)  //�ϴη���δ���
    {
        return 0;
    }

    if(len > sizeof(uart_dma_buf.u1_tx))  //���ȳ���DMA������
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
/*����1================================================================================================*/
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
    uart2_handle.dma_tx.Instance                 = DMA1_Channel7;            //������ѡ��
    uart2_handle.dma_tx.Init.Request             = DMA_REQUEST_2;            //�������ã��������ĸ����������
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
    uart2_handle.dma_rx.Instance                 = DMA1_Channel6;            //������ѡ��
    uart2_handle.dma_rx.Init.Request             = DMA_REQUEST_2;            //�������ã��������ĸ����������
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
    uart2_handle.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  //����λ����ѡ��
    HAL_UART_Init(&uart2_handle.uart);  //��ʼ��
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);          //ʹ���ж�
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

    if(__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)  //�����ж�
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = sizeof(uart_dma_buf.u2_rx) - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart2_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u2_rx, sizeof(uart_dma_buf.u2_rx)); //���¹���DMA
        //��DR����Ƶ�����ж�
        len = USART2->RDR;
        len = USART2->ISR;
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
void DMA1_Channel7_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart2_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //����HAL���жϴ����ú���
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Channel6_IRQHandler(void)
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
        len = USART2->RDR;
        len = USART2->ISR;
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
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_UART_GET_FLAG(&uart2_handle.uart, USART_FLAG_TC) != SET)  //�ϴη���δ���
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

    HAL_UART_Transmit_DMA(&uart2_handle.uart, uart_dma_buf.u2_tx, len);

    return len;
}
/*����2================================================================================================*/
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
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    /* IO���� */
    gpio_init_struct.Pin       = GPIO_PIN_4 | GPIO_PIN_5; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //ģʽ
    gpio_init_struct.Pull      = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF7_USART3;         //����
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);  //��ʼ��
    /* ����DMA���� */
    /* Tx DMA���� */
    uart3_handle.dma_tx.Instance                 = DMA1_Channel2;            //������ѡ��
    uart3_handle.dma_tx.Init.Request             = DMA_REQUEST_2;            //�������ã��������ĸ����������
    uart3_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //���䷽��
    uart3_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //���裨�ǣ�����ģʽ
    uart3_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //�洢�����ǣ�����ģʽ
    uart3_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //�������ݴ�С�� 8/16/32 λ
    uart3_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //�洢�����ݴ�С�� 8/16/32 λ
    uart3_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //ģʽ����������ģʽ/ѭ��ģʽ/��ͨģʽ
    uart3_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA ���ȼ�����/��/��/�ǳ���
    HAL_DMA_DeInit(&uart3_handle.dma_tx);  //ȡ����ʼ��
    HAL_DMA_Init(&uart3_handle.dma_tx);    //��ʼ��
    __HAL_LINKDMA(&uart3_handle.uart, hdmatx, uart3_handle.dma_tx);  //��DMA��UART��ϵ����
    /* Rx DMA���� */
    uart3_handle.dma_rx = uart3_handle.dma_tx;  //����������ͬ
    uart3_handle.dma_rx.Instance                 = DMA1_Channel3;            //������ѡ��
    uart3_handle.dma_rx.Init.Request             = DMA_REQUEST_2;            //�������ã��������ĸ����������
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
    HAL_UART_Init(&uart3_handle.uart);  //��ʼ��
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);          //ʹ���ж�
    /* �����ж����� */
    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(USART3_IRQn);          //ʹ���ж�
    __HAL_UART_ENABLE_IT(&uart3_handle.uart, UART_IT_IDLE);  //�������ڿ����ж�
    /* ���� */
    HAL_UART_Receive_DMA(&uart3_handle.uart, uart_dma_buf.u3_rx, sizeof(uart_dma_buf.u3_rx));  //rxdma������������dma�ж�
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
        len = sizeof(uart_dma_buf.u3_rx) - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart3_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u3_rx, sizeof(uart_dma_buf.u3_rx)); //���¹���DMA
        //��DR����Ƶ�����ж�
        len = USART3->RDR;
        len = USART3->ISR;
    }
    else
    {
        HAL_UART_IRQHandler(handle);  //����HAL���жϴ����ú���
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u3_rx, sizeof(uart_dma_buf.u3_rx)); //���¹���DMA
    }
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Channel2_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = uart3_handle.uart.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //����HAL���жϴ����ú���
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Channel3_IRQHandler(void)
{
    uint32_t len = 0;
    UART_HandleTypeDef *handle = &uart3_handle.uart;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
        
        HAL_UART_DMAStop(handle);  //ֹͣDMA����
        len = sizeof(uart_dma_buf.u3_rx) - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        uart3_receive(handle->pRxBuffPtr, len);  //�������
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u3_rx, sizeof(uart_dma_buf.u3_rx)); //���¹���DMA
        
        //��DR����Ƶ�����ж�
        len = USART3->RDR;
        len = USART3->ISR;
    }
    else
    {
        HAL_DMA_IRQHandler(handle->hdmarx);  //����HAL���жϴ����ú���
        HAL_UART_Receive_DMA(handle, uart_dma_buf.u3_rx, sizeof(uart_dma_buf.u3_rx)); //���¹���DMA
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

    if(len > sizeof(uart_dma_buf.u3_tx))  //���ȳ���DMA������
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
/*����3================================================================================================*/
