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

#define USART1_BAUD  115200  //���ڲ�����
#define USART2_BAUD  115200  //���ڲ�����
#define USART3_BAUD  115200  //���ڲ�����
#define UART4_BAUD   115200  //���ڲ�����
#define UART5_BAUD   115200  //���ڲ�����
#define USART6_BAUD  115200  //���ڲ�����

/**
* @struct  UART_DMA_DATA
* @brief   ����DMA����
*/
typedef struct
{
    uint8_t Rx[UART_DMA_RX_SIZE];
    uint8_t Tx[UART_DMA_TX_SIZE];
}UART_DMA_DATA;


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

UART_DMA_DATA Usart1DMA = {0};  //����1 DMA����
UART_DMA_DATA Usart2DMA = {0};  //����2 DMA����
UART_DMA_DATA Usart3DMA = {0};  //����3 DMA����
UART_DMA_DATA Uart4DMA  = {0};  //����4 DMA����
UART_DMA_DATA Uart5DMA  = {0};  //����5 DMA����
UART_DMA_DATA Usart6DMA = {0};  //����6 DMA����

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/*����1*************************************************************************************************/
/**
* @brief  ���ڳ�ʼ��
* @attention ʹ����DMA����,�����������ж�,����Tx��Rx��DMA��������ж�,�ж�DMA�������
*            �����������߿����ж������ж�һ֡���ݽ������
*/
void Usart1Init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef   DMA_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;

    GPIO_StructInit(&GPIO_InitStruct);
    USART_StructInit(&USART_InitStruct);
    DMA_StructInit(&DMA_InitStruct);
    DMA_DeInit(DMA2_Stream2);
    DMA_DeInit(DMA2_Stream7);
    /* ʹ��ʱ�� */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    /* GPIO���� */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;  //ͨ��
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       //ģʽ
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO��������·��Ӧ�ٶ�
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //���ģʽ
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //������
    GPIO_Init(GPIOA, &GPIO_InitStruct);              //��ʼ��
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    /* �������� */
    USART_InitStruct.USART_BaudRate            = USART1_BAUD;                     //������
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;             //8������λ
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;                //1��ֹͣλ
    USART_InitStruct.USART_Parity              = USART_Parity_No;                 //����żУ��
    USART_InitStruct.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;   //�շ�ģʽ
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //��Ӳ������������
    USART_Init(USART1, &USART_InitStruct);                                        //��ʼ��

    USART_Cmd(USART1, ENABLE);                                                    //ʹ��
    USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);              //ʹ��DMA�����շ�
    /* �����жϳ�ʼ�� */
    NVIC_InitStruct.NVIC_IRQChannel                   = USART1_IRQn;  //�ж���
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;         //��ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;         //�����ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;       //ʹ��
    NVIC_Init(&NVIC_InitStruct);                                      //��ʼ��
    
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);  //ʹ�����߿����ж�,�ж�һ֡���ݽ������
    /* DMA���� */
    /* RxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //ͨ��
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);         //����Ĵ���(����ģʽ)����ַ
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Usart1DMA.Rx;          //�洢������ַ
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralToMemory;      //DMA���䷽��,���赽�洢��
    DMA_InitStruct.DMA_BufferSize         = UART_DMA_RX_SIZE;                //������ֽ���
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //�����ַ����,�ر�
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //�洢����ַ����,ʹ��
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //�������ݿ��,���ֽڶ���,8λ
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;         //�洢�����ݿ��,���ֽڶ���,8λ
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;                 //��ѭ��ģʽ
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;               //���ȼ�
    DMA_InitStruct.DMA_FIFOMode           = DMA_FIFOMode_Disable;            //FIFO,�ر�
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;  //FIFO��ֵ����,1/4
    DMA_InitStruct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;          //�洢��ͻ����������,���δ���
    DMA_InitStruct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;      //����ͻ����������,���δ���
    DMA_Init(DMA2_Stream2, &DMA_InitStruct);                                 //��ʼ��
    /* TxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //ͨ��
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Usart1DMA.Tx;          //�洢������ַ
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;      //DMA���䷽��,�洢��������
    DMA_InitStruct.DMA_BufferSize         = 0;                               //������ֽ���,��ʼ��ʱ������
    DMA_Init(DMA2_Stream7, &DMA_InitStruct);                                 //��ʼ��
    /* DMA�ж����� */
    /* RxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream2_IRQn;  //�ж���
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;               //��ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;               //�����ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;             //ʹ��
    NVIC_Init(&NVIC_InitStruct);                                            //��ʼ��
    /* TxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream7_IRQn;  //�ж���
    NVIC_Init(&NVIC_InitStruct);                                            //��ʼ��
    
    DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);  //DMA�ж�ʹ��,��������ж�
    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);  //DMA�ж�ʹ��,��������ж�
    /* �������� */
    DMA_Cmd(DMA2_Stream2, ENABLE);   //����DMA����,Rx
    DMA_Cmd(DMA2_Stream7, DISABLE);  //��ֹDMA����,Tx
}
/**
* @brief  �����ж�
* @attention 
*/
void USART1_IRQHandler(void)
{
    uint16_t len = 0;  //���ݳ���
    
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_IDLE);
        
        DMA_Cmd(DMA2_Stream2, DISABLE);  //�رս���DMA
        len = sizeof(Usart1DMA.Rx) - DMA_GetCurrDataCounter(DMA2_Stream2);  //��ȡ�������ݳ���
        
        Usart1Receive(Usart1DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA2_Stream2, UART_DMA_RX_SIZE);  //����DMA���ճ���
        DMA_Cmd(DMA2_Stream2, ENABLE);     //ʹ�ܽ���DMA
        
        //��DR����Ƶ�����ж�
        len = USART1->DR;
        len = USART1->SR;
    }
}
/**
* @brief ���� DMA�ж�,RX
*/
void DMA2_Stream2_IRQHandler(void)
{
    uint16_t len = 0;  //���ݳ���
    
    if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);

        DMA_Cmd(DMA2_Stream2, DISABLE);     //�رս���DMA
        len = sizeof(Usart1DMA.Rx) - DMA_GetCurrDataCounter(DMA2_Stream2);  //��ȡ�������ݳ���
        
        Usart1Receive(Usart1DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA2_Stream2, UART_DMA_RX_SIZE);  //����DMA���ճ���
        DMA_Cmd(DMA2_Stream2, ENABLE);     //ʹ�ܽ���DMA
        USART_ClearITPendingBit(USART1, USART_IT_IDLE);  //�����ڴ�λ�������һ֡�����ж�
        
        //��DR����Ƶ�����ж�
        len = USART1->DR;
        len = USART1->SR;
    }
}
/**
* @brief ���� DMA�ж�,TX
*/
void DMA2_Stream7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);

        DMA_Cmd(DMA2_Stream7, DISABLE);     //�رս���DMA
        DMA_SetCurrDataCounter(DMA2_Stream7, 0);  //������ݳ���
    }
}
/**
* @brief ���ڷ�������
*/
uint16_t Usart1Send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET)  //�ϴη���δ���
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
        Usart1DMA.Tx[i] = data[i];
    }

    DMA_Cmd(DMA2_Stream7, DISABLE);  //�ر�DMA����,DMA�����������ȹر�,д�볤��,������
    DMA_SetCurrDataCounter(DMA2_Stream7, len);  //����д�봫�����ݳ���
    DMA_Cmd(DMA2_Stream7, ENABLE);  //����DMA����

    return len;
}
/*����2*************************************************************************************************/
/**
* @brief  ���ڳ�ʼ��
* @attention ʹ����DMA����,�����������ж�,����Tx��Rx��DMA��������ж�,�ж�DMA�������
*            �����������߿����ж������ж�һ֡���ݽ������
*/
void Usart2Init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef   DMA_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;

    GPIO_StructInit(&GPIO_InitStruct);
    USART_StructInit(&USART_InitStruct);
    DMA_StructInit(&DMA_InitStruct);
    DMA_DeInit(DMA1_Stream5);
    DMA_DeInit(DMA1_Stream6);
    /* ʹ��ʱ�� */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    /* GPIO���� */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;  //ͨ��
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       //ģʽ
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO��������·��Ӧ�ٶ�
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //���ģʽ
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //������
    GPIO_Init(GPIOA, &GPIO_InitStruct);              //��ʼ��
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
    /* �������� */
    USART_InitStruct.USART_BaudRate            = USART2_BAUD;                     //������
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;             //8������λ
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;                //1��ֹͣλ
    USART_InitStruct.USART_Parity              = USART_Parity_No;                 //����żУ��
    USART_InitStruct.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;   //�շ�ģʽ
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //��Ӳ������������
    USART_Init(USART2, &USART_InitStruct);                                        //��ʼ��

    USART_Cmd(USART2, ENABLE);                                                    //ʹ��
    USART_DMACmd(USART2, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);              //ʹ��DMA�����շ�
    /* �����жϳ�ʼ�� */
    NVIC_InitStruct.NVIC_IRQChannel                   = USART2_IRQn;  //�ж���
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;         //��ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;         //�����ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;       //ʹ��
    NVIC_Init(&NVIC_InitStruct);                                      //��ʼ��
    
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);  //ʹ�����߿����ж�,�ж�һ֡���ݽ������
    /* DMA���� */
    /* RxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //ͨ��
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);         //����Ĵ���(����ģʽ)����ַ
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Usart2DMA.Rx;          //�洢������ַ
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralToMemory;      //DMA���䷽��,���赽�洢��
    DMA_InitStruct.DMA_BufferSize         = UART_DMA_RX_SIZE;                //������ֽ���
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //�����ַ����,�ر�
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //�洢����ַ����,ʹ��
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //�������ݿ��,���ֽڶ���,8λ
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;         //�洢�����ݿ��,���ֽڶ���,8λ
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;                 //��ѭ��ģʽ
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;               //���ȼ�
    DMA_InitStruct.DMA_FIFOMode           = DMA_FIFOMode_Disable;            //FIFO,�ر�
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;  //FIFO��ֵ����,1/4
    DMA_InitStruct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;          //�洢��ͻ����������,���δ���
    DMA_InitStruct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;      //����ͻ����������,���δ���
    DMA_Init(DMA1_Stream5, &DMA_InitStruct);                                 //��ʼ��
    /* TxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //ͨ��
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Usart2DMA.Tx;          //�洢������ַ
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;      //DMA���䷽��,�洢��������
    DMA_InitStruct.DMA_BufferSize         = 0;                               //������ֽ���,��ʼ��ʱ������
    DMA_Init(DMA1_Stream6, &DMA_InitStruct);                                 //��ʼ��
    /* DMA�ж����� */
    /* RxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Stream5_IRQn;  //�ж���
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;               //��ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;               //�����ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;             //ʹ��
    NVIC_Init(&NVIC_InitStruct);                                            //��ʼ��
    /* TxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Stream6_IRQn;  //�ж���
    NVIC_Init(&NVIC_InitStruct);                                            //��ʼ��
    
    DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);  //DMA�ж�ʹ��,��������ж�
    DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);  //DMA�ж�ʹ��,��������ж�
    /* �������� */
    DMA_Cmd(DMA1_Stream5, ENABLE);   //����DMA����,Rx
    DMA_Cmd(DMA1_Stream6, DISABLE);  //��ֹDMA����,Tx
}
/**
* @brief  �����ж�
* @attention 
*/
void USART2_IRQHandler(void)
{
    uint16_t len = 0;  //���ݳ���
    
    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        USART_ClearITPendingBit(USART2, USART_IT_IDLE);
        
        DMA_Cmd(DMA1_Stream5, DISABLE);  //�رս���DMA
        len = sizeof(Usart2DMA.Rx) - DMA_GetCurrDataCounter(DMA1_Stream5);  //��ȡ�������ݳ���
        
        Usart2Receive(Usart2DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA1_Stream5, UART_DMA_RX_SIZE);  //����DMA���ճ���
        DMA_Cmd(DMA1_Stream5, ENABLE);     //ʹ�ܽ���DMA
        
        //��DR����Ƶ�����ж�
        len = USART2->DR;
        len = USART2->SR;
    }
}
/**
* @brief ���� DMA�ж�,RX
*/
void DMA1_Stream5_IRQHandler(void)
{
    uint16_t len = 0;  //���ݳ���
    
    if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);

        DMA_Cmd(DMA1_Stream5, DISABLE);     //�رս���DMA
        len = sizeof(Usart2DMA.Rx) - DMA_GetCurrDataCounter(DMA1_Stream5);  //��ȡ�������ݳ���
        
        Usart2Receive(Usart2DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA1_Stream5, UART_DMA_RX_SIZE);  //����DMA���ճ���
        DMA_Cmd(DMA1_Stream5, ENABLE);     //ʹ�ܽ���DMA
        USART_ClearITPendingBit(USART2, USART_IT_IDLE);  //�����ڴ�λ�������һ֡�����ж�
        
        //��DR����Ƶ�����ж�
        len = USART2->DR;
        len = USART2->SR;
    }
}
/**
* @brief ���� DMA�ж�,TX
*/
void DMA1_Stream6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);

        DMA_Cmd(DMA1_Stream6, DISABLE);     //�رս���DMA
        DMA_SetCurrDataCounter(DMA1_Stream6, 0);  //������ݳ���
    }
}
/**
* @brief ���ڷ�������
*/
uint16_t Usart2Send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(USART_GetFlagStatus(USART2, USART_FLAG_TC) != SET)  //�ϴη���δ���
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
        Usart2DMA.Tx[i] = data[i];
    }

    DMA_Cmd(DMA1_Stream6, DISABLE);  //�ر�DMA����,DMA�����������ȹر�,д�볤��,������
    DMA_SetCurrDataCounter(DMA1_Stream6, len);  //����д�봫�����ݳ���
    DMA_Cmd(DMA1_Stream6, ENABLE);  //����DMA����

    return len;
}
/*����3*************************************************************************************************/
/**
* @brief  ���ڳ�ʼ��
* @attention ʹ����DMA����,�����������ж�,����Tx��Rx��DMA��������ж�,�ж�DMA�������
*            �����������߿����ж������ж�һ֡���ݽ������
*/
void Usart3Init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef   DMA_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;

    GPIO_StructInit(&GPIO_InitStruct);
    USART_StructInit(&USART_InitStruct);
    DMA_StructInit(&DMA_InitStruct);
    DMA_DeInit(DMA1_Stream1);
    DMA_DeInit(DMA1_Stream3);
    /* ʹ��ʱ�� */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    /* GPIO���� */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;  //ͨ��
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       //ģʽ
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO��������·��Ӧ�ٶ�
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //���ģʽ
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //������
    GPIO_Init(GPIOC, &GPIO_InitStruct);              //��ʼ��
    
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
    /* �������� */
    USART_InitStruct.USART_BaudRate            = USART3_BAUD;                     //������
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;             //8������λ
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;                //1��ֹͣλ
    USART_InitStruct.USART_Parity              = USART_Parity_No;                 //����żУ��
    USART_InitStruct.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;   //�շ�ģʽ
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //��Ӳ������������
    USART_Init(USART3, &USART_InitStruct);                                        //��ʼ��

    USART_Cmd(USART3, ENABLE);                                                    //ʹ��
    USART_DMACmd(USART3, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);              //ʹ��DMA�����շ�
    /* �����жϳ�ʼ�� */
    NVIC_InitStruct.NVIC_IRQChannel                   = USART3_IRQn;  //�ж���
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;         //��ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;         //�����ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;       //ʹ��
    NVIC_Init(&NVIC_InitStruct);                                      //��ʼ��
    
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);  //ʹ�����߿����ж�,�ж�һ֡���ݽ������
    /* DMA���� */
    /* RxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //ͨ��
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);         //����Ĵ���(����ģʽ)����ַ
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Usart3DMA.Rx;          //�洢������ַ
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralToMemory;      //DMA���䷽��,���赽�洢��
    DMA_InitStruct.DMA_BufferSize         = UART_DMA_RX_SIZE;                //������ֽ���
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //�����ַ����,�ر�
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //�洢����ַ����,ʹ��
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //�������ݿ��,���ֽڶ���,8λ
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;         //�洢�����ݿ��,���ֽڶ���,8λ
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;                 //��ѭ��ģʽ
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;               //���ȼ�
    DMA_InitStruct.DMA_FIFOMode           = DMA_FIFOMode_Disable;            //FIFO,�ر�
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;  //FIFO��ֵ����,1/4
    DMA_InitStruct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;          //�洢��ͻ����������,���δ���
    DMA_InitStruct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;      //����ͻ����������,���δ���
    DMA_Init(DMA1_Stream1, &DMA_InitStruct);                                 //��ʼ��
    /* TxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //ͨ��
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Usart3DMA.Tx;          //�洢������ַ
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;      //DMA���䷽��,�洢��������
    DMA_InitStruct.DMA_BufferSize         = 0;                               //������ֽ���,��ʼ��ʱ������
    DMA_Init(DMA1_Stream3, &DMA_InitStruct);                                 //��ʼ��
    /* DMA�ж����� */
    /* RxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Stream1_IRQn;  //�ж���
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;               //��ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;               //�����ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;             //ʹ��
    NVIC_Init(&NVIC_InitStruct);                                            //��ʼ��
    /* TxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Stream3_IRQn;  //�ж���
    NVIC_Init(&NVIC_InitStruct);                                            //��ʼ��
    
    DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);  //DMA�ж�ʹ��,��������ж�
    DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);  //DMA�ж�ʹ��,��������ж�
    /* �������� */
    DMA_Cmd(DMA1_Stream1, ENABLE);   //����DMA����,Rx
    DMA_Cmd(DMA1_Stream3, DISABLE);  //��ֹDMA����,Tx
}
/**
* @brief  �����ж�
* @attention 
*/
void USART3_IRQHandler(void)
{
    uint16_t len = 0;  //���ݳ���
    
    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);
        
        DMA_Cmd(DMA1_Stream1, DISABLE);  //�رս���DMA
        len = sizeof(Usart3DMA.Rx) - DMA_GetCurrDataCounter(DMA1_Stream1);  //��ȡ�������ݳ���
        
        Usart3Receive(Usart3DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA1_Stream1, UART_DMA_RX_SIZE);  //����DMA���ճ���
        DMA_Cmd(DMA1_Stream1, ENABLE);     //ʹ�ܽ���DMA
        
        //��DR����Ƶ�����ж�
        len = USART3->DR;
        len = USART3->SR;
    }
}
/**
* @brief ���� DMA�ж�,RX
*/
void DMA1_Stream1_IRQHandler(void)
{
    uint16_t len = 0;  //���ݳ���
    
    if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);

        DMA_Cmd(DMA1_Stream1, DISABLE);     //�رս���DMA
        len = sizeof(Usart3DMA.Rx) - DMA_GetCurrDataCounter(DMA1_Stream1);  //��ȡ�������ݳ���
        
        Usart3Receive(Usart3DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA1_Stream1, UART_DMA_RX_SIZE);  //����DMA���ճ���
        DMA_Cmd(DMA1_Stream1, ENABLE);     //ʹ�ܽ���DMA
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);  //�����ڴ�λ�������һ֡�����ж�
        
        //��DR����Ƶ�����ж�
        len = USART3->DR;
        len = USART3->SR;
    }
}
/**
* @brief ���� DMA�ж�,TX
*/
void DMA1_Stream3_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);

        DMA_Cmd(DMA1_Stream3, DISABLE);     //�رս���DMA
        DMA_SetCurrDataCounter(DMA1_Stream3, 0);  //������ݳ���
    }
}
/**
* @brief ���ڷ�������
*/
uint16_t Usart3Send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(USART_GetFlagStatus(USART3, USART_FLAG_TC) != SET)  //�ϴη���δ���
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
        Usart3DMA.Tx[i] = data[i];
    }

    DMA_Cmd(DMA1_Stream3, DISABLE);  //�ر�DMA����,DMA�����������ȹر�,д�볤��,������
    DMA_SetCurrDataCounter(DMA1_Stream3, len);  //����д�봫�����ݳ���
    DMA_Cmd(DMA1_Stream3, ENABLE);  //����DMA����

    return len;
}
/*����4*************************************************************************************************/
/**
* @brief  ���ڳ�ʼ��
* @attention ʹ����DMA����,�����������ж�,����Tx��Rx��DMA��������ж�,�ж�DMA�������
*            �����������߿����ж������ж�һ֡���ݽ������
*/
void Uart4Init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    GPIO_StructInit(&GPIO_InitStruct);
    USART_StructInit(&USART_InitStruct);
    DMA_StructInit(&DMA_InitStruct);
    DMA_DeInit(DMA1_Stream2);
    DMA_DeInit(DMA1_Stream4);
    /* ʹ��ʱ�� */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    /* GPIO���� */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1; //ͨ��
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       //ģʽ
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO��������·��Ӧ�ٶ�
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //���ģʽ
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //������
    GPIO_Init(GPIOA, &GPIO_InitStruct);              //��ʼ��
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
    /* �������� */
    USART_InitStruct.USART_BaudRate            = UART4_BAUD;                      //������
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;             //8������λ
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;                //1��ֹͣλ
    USART_InitStruct.USART_Parity              = USART_Parity_No;                 //����żУ��
    USART_InitStruct.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;   //�շ�ģʽ
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //��Ӳ������������
    USART_Init(UART4, &USART_InitStruct);                                         //��ʼ��

    USART_Cmd(UART4, ENABLE);                                                    //ʹ��
    USART_DMACmd(UART4, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);              //ʹ��DMA�����շ�
    /* �����жϳ�ʼ�� */
    NVIC_InitStruct.NVIC_IRQChannel                   = UART4_IRQn;   //�ж���
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;         //��ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;         //�����ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;       //ʹ��
    NVIC_Init(&NVIC_InitStruct);                                      //��ʼ��
    
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);  //ʹ�����߿����ж�,�ж�һ֡���ݽ������
    /* DMA���� */
    /* RxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //ͨ��
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR);          //����Ĵ���(����ģʽ)����ַ
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Uart4DMA.Rx;           //�洢������ַ
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralToMemory;      //DMA���䷽��,���赽�洢��
    DMA_InitStruct.DMA_BufferSize         = UART_DMA_RX_SIZE;                //������ֽ���
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //�����ַ����,�ر�
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //�洢����ַ����,ʹ��
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //�������ݿ��,���ֽڶ���,8λ
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;         //�洢�����ݿ��,���ֽڶ���,8λ
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;                 //��ѭ��ģʽ
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;               //���ȼ�
    DMA_InitStruct.DMA_FIFOMode           = DMA_FIFOMode_Disable;            //FIFO,�ر�
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;  //FIFO��ֵ����,1/4
    DMA_InitStruct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;          //�洢��ͻ����������,���δ���
    DMA_InitStruct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;      //����ͻ����������,���δ���
    DMA_Init(DMA1_Stream2, &DMA_InitStruct);                                 //��ʼ��
    /* TxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //ͨ��
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Uart4DMA.Tx;           //�洢������ַ
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;      //DMA���䷽��,�洢��������
    DMA_InitStruct.DMA_BufferSize         = 0;                               //������ֽ���,��ʼ��ʱ������
    DMA_Init(DMA1_Stream4, &DMA_InitStruct);                                 //��ʼ��
    /* DMA�ж����� */
    /* RxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Stream2_IRQn;  //�ж���
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;               //��ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;               //�����ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;             //ʹ��
    NVIC_Init(&NVIC_InitStruct);                                            //��ʼ��
    /* TxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Stream4_IRQn;  //�ж���
    NVIC_Init(&NVIC_InitStruct);                                            //��ʼ��
    
    DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);  //DMA�ж�ʹ��,��������ж�
    DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);  //DMA�ж�ʹ��,��������ж�
    /* �������� */
    DMA_Cmd(DMA1_Stream2, DISABLE);  //��ֹDMA����,Rx
    DMA_Cmd(DMA1_Stream4, DISABLE);  //��ֹDMA����,Tx
}
/**
* @brief  �����ж�
* @attention 
*/
void UART4_IRQHandler(void)
{
    uint16_t len = 0;  //���ݳ���
    
    if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
    {
        USART_ClearITPendingBit(UART4, USART_IT_IDLE);

        DMA_Cmd(DMA1_Stream2, DISABLE);     //�رս���DMA
        len = sizeof(Uart4DMA.Rx) - DMA_GetCurrDataCounter(DMA1_Stream2);  //��ȡ�������ݳ���
        
        Uart4Receive(Uart4DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA1_Stream2, UART_DMA_RX_SIZE);  //����DMA���ճ���
        DMA_Cmd(DMA1_Stream2, ENABLE);     //ʹ�ܽ���DMA
        
        //��DR����Ƶ�����ж�
        len = UART4->DR;
        len = UART4->SR;
    }
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Stream2_IRQHandler(void)
{
    uint16_t len = 0;  //���ݳ���
    
    if(DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
        
        DMA_Cmd(DMA1_Stream2, DISABLE);     //�رս���DMA
        len = sizeof(Uart4DMA.Rx) - DMA_GetCurrDataCounter(DMA1_Stream2);  //��ȡ�������ݳ���
        
        Uart4Receive(Uart4DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA1_Stream2, UART_DMA_RX_SIZE);  //����DMA���ճ���
        DMA_Cmd(DMA1_Stream2, ENABLE);     //ʹ�ܽ���DMA
        
        //��DR����Ƶ�����ж�
        len = UART4->DR;
        len = UART4->SR;
    }
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Stream4_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);

        DMA_Cmd(DMA1_Stream4, DISABLE);     //�رս���DMA
        DMA_SetCurrDataCounter(DMA1_Stream4, 0);  //������ݳ���
    }
}
/**
* @brief ���ڷ�������
*/
uint16_t Uart4Send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(USART_GetFlagStatus(UART4, USART_FLAG_TC) != SET)  //�ϴη���δ���
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
        Uart4DMA.Tx[i] = data[i];
    }

    DMA_Cmd(DMA1_Stream4, DISABLE);  //�ر�DMA����,DMA�����������ȹر�,д�볤��,������
    DMA_SetCurrDataCounter(DMA1_Stream4, len);  //����д�봫�����ݳ���
    DMA_Cmd(DMA1_Stream4, ENABLE);  //����DMA����
    
    return len;
}
/*����5*************************************************************************************************/
/**
* @brief  ���ڳ�ʼ��
* @attention ʹ����DMA����,�����������ж�,����Tx��Rx��DMA��������ж�,�ж�DMA�������
*            �����������߿����ж������ж�һ֡���ݽ������
*/
void Uart5Init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    GPIO_StructInit(&GPIO_InitStruct);
    USART_StructInit(&USART_InitStruct);
    DMA_StructInit(&DMA_InitStruct);
    DMA_DeInit(DMA1_Stream0);
    DMA_DeInit(DMA1_Stream7);
    /* ʹ��ʱ�� */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    /* GPIO���� */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_12;        //ͨ��
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       //ģʽ
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO��������·��Ӧ�ٶ�
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //���ģʽ
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //������
    GPIO_Init(GPIOC, &GPIO_InitStruct);              //��ʼ��

    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2;        //ͨ��
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;      //ģʽ
    GPIO_Init(GPIOD, &GPIO_InitStruct);             //��ʼ��
    
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
    /* �������� */
    USART_InitStruct.USART_BaudRate            = UART5_BAUD;                      //������
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;             //8������λ
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;                //1��ֹͣλ
    USART_InitStruct.USART_Parity              = USART_Parity_No;                 //����żУ��
    USART_InitStruct.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;   //�շ�ģʽ
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //��Ӳ������������
    USART_Init(UART5, &USART_InitStruct);                                         //��ʼ��

    USART_Cmd(UART5, ENABLE);                                                    //ʹ��
    USART_DMACmd(UART5, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);              //ʹ��DMA�����շ�
    /* �����жϳ�ʼ�� */
    NVIC_InitStruct.NVIC_IRQChannel                   = UART5_IRQn;   //�ж���
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;         //��ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;         //�����ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;       //ʹ��
    NVIC_Init(&NVIC_InitStruct);                                      //��ʼ��
    
    USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);  //ʹ�����߿����ж�,�ж�һ֡���ݽ������
    /* DMA���� */
    /* RxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //ͨ��
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(UART5->DR);          //����Ĵ���(����ģʽ)����ַ
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Uart5DMA.Rx;           //�洢������ַ
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralToMemory;      //DMA���䷽��,���赽�洢��
    DMA_InitStruct.DMA_BufferSize         = UART_DMA_RX_SIZE;                //������ֽ���
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //�����ַ����,�ر�
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //�洢����ַ����,ʹ��
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //�������ݿ��,���ֽڶ���,8λ
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;         //�洢�����ݿ��,���ֽڶ���,8λ
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;                 //��ѭ��ģʽ
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;               //���ȼ�
    DMA_InitStruct.DMA_FIFOMode           = DMA_FIFOMode_Disable;            //FIFO,�ر�
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;  //FIFO��ֵ����,1/4
    DMA_InitStruct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;          //�洢��ͻ����������,���δ���
    DMA_InitStruct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;      //����ͻ����������,���δ���
    DMA_Init(DMA1_Stream0, &DMA_InitStruct);                                 //��ʼ��
    /* TxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_4;                   //ͨ��
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Uart5DMA.Tx;           //�洢������ַ
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;      //DMA���䷽��,�洢��������
    DMA_InitStruct.DMA_BufferSize         = 0;                               //������ֽ���,��ʼ��ʱ������
    DMA_Init(DMA1_Stream7, &DMA_InitStruct);                                 //��ʼ��
    /* DMA�ж����� */
    /* RxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Stream0_IRQn;  //�ж���
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;               //��ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x02;               //�����ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;             //ʹ��
    NVIC_Init(&NVIC_InitStruct);                                            //��ʼ��
    /* TxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA1_Stream7_IRQn;  //�ж���
    NVIC_Init(&NVIC_InitStruct);                                            //��ʼ��
    
    DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);  //DMA�ж�ʹ��,��������ж�
    DMA_ITConfig(DMA1_Stream7, DMA_IT_TC, ENABLE);  //DMA�ж�ʹ��,��������ж�
    /* �������� */
    DMA_Cmd(DMA1_Stream0, DISABLE);  //��ֹDMA����,Rx
    DMA_Cmd(DMA1_Stream7, DISABLE);  //��ֹDMA����,Tx
}
/**
* @brief  �����ж�
* @attention 
*/
void UART5_IRQHandler(void)
{
    uint16_t len = 0;  //���ݳ���
    
    if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
    {
        USART_ClearITPendingBit(UART5, USART_IT_IDLE);

        DMA_Cmd(DMA1_Stream0, DISABLE);     //�رս���DMA
        len = sizeof(Uart5DMA.Rx) - DMA_GetCurrDataCounter(DMA1_Stream0);  //��ȡ�������ݳ���
        
        Uart5Receive(Uart5DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA1_Stream0, UART_DMA_RX_SIZE);  //����DMA���ճ���
        DMA_Cmd(DMA1_Stream0, ENABLE);     //ʹ�ܽ���DMA
        
        //��DR����Ƶ�����ж�
        len = UART5->DR;
        len = UART5->SR;
    }
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Stream0_IRQHandler(void)
{
    uint16_t len = 0;  //���ݳ���
    
    if(DMA_GetITStatus(DMA1_Stream0, DMA_IT_TCIF0) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);
        
        DMA_Cmd(DMA1_Stream0, DISABLE);     //�رս���DMA
        len = sizeof(Uart5DMA.Rx) - DMA_GetCurrDataCounter(DMA1_Stream0);  //��ȡ�������ݳ���
        
        Uart5Receive(Uart5DMA.Rx, len);
        
        DMA_SetCurrDataCounter(DMA1_Stream0, UART_DMA_RX_SIZE);  //����DMA���ճ���
        DMA_Cmd(DMA1_Stream0, ENABLE);     //ʹ�ܽ���DMA
        
        //��DR����Ƶ�����ж�
        len = UART5->DR;
        len = UART5->SR;
    }
}
/**
* @brief ���� DMA�����ж�
*/
void DMA1_Stream7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream7, DMA_IT_TCIF7) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7);

        DMA_Cmd(DMA1_Stream7, DISABLE);     //�رս���DMA
        DMA_SetCurrDataCounter(DMA1_Stream7, 0);  //������ݳ���
    }
}
/**
* @brief     ���ڷ�������
*/
uint16_t Uart5Send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(USART_GetFlagStatus(UART5, USART_FLAG_TC) != SET)  //�ϴη���δ���
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
        Uart5DMA.Tx[i] = data[i];
    }

    DMA_Cmd(DMA1_Stream7, DISABLE);  //�ر�DMA����,DMA�����������ȹر�,д�볤��,������
    DMA_SetCurrDataCounter(DMA1_Stream7, len);  //����д�봫�����ݳ���
    DMA_Cmd(DMA1_Stream7, ENABLE);  //����DMA����
    
    return len;
}
/*����6*************************************************************************************************/
/**
* @brief  ���ڳ�ʼ��
* @attention ʹ����DMA����,�����������ж�,����Tx��Rx��DMA��������ж�,�ж�DMA�������
*            �����������߿����ж������ж�һ֡���ݽ������
*/
void Usart6Init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    GPIO_StructInit(&GPIO_InitStruct);
    USART_StructInit(&USART_InitStruct);
    DMA_StructInit(&DMA_InitStruct);
    DMA_DeInit(DMA2_Stream1);
    DMA_DeInit(DMA2_Stream6);
    /* ʹ��ʱ�� */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    /* GPIO���� */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;  //ͨ��
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       //ģʽ
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO��������·��Ӧ�ٶ�
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //���ģʽ
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //������
    GPIO_Init(GPIOC, &GPIO_InitStruct);              //��ʼ��
    
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
    /* �������� */
    USART_InitStruct.USART_BaudRate            = USART6_BAUD;                     //������
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;             //8������λ
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;                //1��ֹͣλ
    USART_InitStruct.USART_Parity              = USART_Parity_No;                 //����żУ��
    USART_InitStruct.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;   //�շ�ģʽ
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //��Ӳ������������
    USART_Init(USART6, &USART_InitStruct);                                         //��ʼ��

    USART_Cmd(USART6, ENABLE);                                                    //ʹ��
    USART_DMACmd(USART6, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);              //ʹ��DMA�����շ�
    /* �����жϳ�ʼ�� */
    NVIC_InitStruct.NVIC_IRQChannel                   = USART6_IRQn;  //�ж���
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;         //��ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x03;         //�����ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;       //ʹ��
    NVIC_Init(&NVIC_InitStruct);                                      //��ʼ��
    
    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);  //ʹ�����߿����ж�,�ж�һ֡���ݽ������
    /* DMA���� */
    /* RxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_5;                   //ͨ��
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);         //����Ĵ���(����ģʽ)����ַ
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Usart6DMA.Rx;          //�洢������ַ
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralToMemory;      //DMA���䷽��,���赽�洢��
    DMA_InitStruct.DMA_BufferSize         = UART_DMA_RX_SIZE;                //������ֽ���
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //�����ַ����,�ر�
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //�洢����ַ����,ʹ��
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //�������ݿ��,���ֽڶ���,8λ
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;         //�洢�����ݿ��,���ֽڶ���,8λ
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;                 //��ѭ��ģʽ
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;               //���ȼ�
    DMA_InitStruct.DMA_FIFOMode           = DMA_FIFOMode_Disable;            //FIFO,�ر�
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;  //FIFO��ֵ����,1/4
    DMA_InitStruct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;          //�洢��ͻ����������,���δ���
    DMA_InitStruct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;      //����ͻ����������,���δ���
    DMA_Init(DMA2_Stream1, &DMA_InitStruct);                                 //��ʼ��
    /* TxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_5;                   //ͨ��
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Usart6DMA.Tx;          //�洢������ַ
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;      //DMA���䷽��,�洢��������
    DMA_InitStruct.DMA_BufferSize         = 0;                               //������ֽ���,��ʼ��ʱ������
    DMA_Init(DMA2_Stream6, &DMA_InitStruct);                                 //��ʼ��
    /* DMA�ж����� */
    /* RxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream1_IRQn;  //�ж���
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;               //��ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x03;               //�����ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;             //ʹ��
    NVIC_Init(&NVIC_InitStruct);                                            //��ʼ��
    /* TxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream6_IRQn;  //�ж���
    NVIC_Init(&NVIC_InitStruct);                                            //��ʼ��
    
    DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);  //DMA�ж�ʹ��,��������ж�
    DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);  //DMA�ж�ʹ��,��������ж�
    /* �������� */
    DMA_Cmd(DMA2_Stream1, ENABLE);  //����DMA����,Rx
    DMA_Cmd(DMA2_Stream6, DISABLE);  //��ֹDMA����,Tx
}
/**
* @brief  �����ж�
* @attention 
*/
void USART6_IRQHandler(void)
{
    uint16_t len = 0;  //���ݳ���
    
    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
    {
        USART_ClearITPendingBit(USART6, USART_IT_IDLE);
        
        DMA_Cmd(DMA2_Stream1, DISABLE);     //�رս���DMA
        len = sizeof(Usart6DMA.Rx) - DMA_GetCurrDataCounter(DMA2_Stream1);  //��ȡ�������ݳ���
        
        Usart6Receive(Usart6DMA.Rx, len);  //�������
        
        DMA_SetCurrDataCounter(DMA2_Stream1, UART_DMA_RX_SIZE);  //����DMA���ճ���
        DMA_Cmd(DMA2_Stream1, ENABLE);     //ʹ�ܽ���DMA
        
        //��DR����Ƶ�����ж�
        len = USART6->DR;
        len = USART6->SR;
    }
}
/**
* @brief ���� DMA�����ж�
*/
void DMA2_Stream1_IRQHandler(void)
{
    uint16_t len = 0;  //���ݳ���
    
    if(DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
        
        DMA_Cmd(DMA2_Stream1, DISABLE);     //�رս���DMA
        len = sizeof(Usart6DMA.Rx) - DMA_GetCurrDataCounter(DMA2_Stream1);  //��ȡ�������ݳ���

        Usart6Receive(Usart6DMA.Rx, len);  //�������
        
        DMA_SetCurrDataCounter(DMA2_Stream1, UART_DMA_RX_SIZE);  //����DMA���ճ���
        DMA_Cmd(DMA2_Stream1, ENABLE);     //ʹ�ܽ���DMA
        USART_ClearITPendingBit(USART6, USART_IT_IDLE);
        
        //��DR����Ƶ�����ж�
        len = USART6->DR;
        len = USART6->SR;
    }
}
/**
* @brief ���� DMA�����ж�
*/
void DMA2_Stream6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TCIF6);

        DMA_Cmd(DMA2_Stream6, DISABLE);     //�ر�DMA
        DMA_SetCurrDataCounter(DMA2_Stream6, 0);  //������ݳ���
    }
}
/**
* @brief ���ڷ�������
*/
uint16_t Usart6Send(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(USART_GetFlagStatus(USART6, USART_FLAG_TC) != SET)  //�ϴη���δ���
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
        Usart6DMA.Tx[i] = data[i];
    }

    DMA_Cmd(DMA2_Stream6, DISABLE);  //�ر�DMA����,DMA�����������ȹر�,д�볤��,������
    DMA_SetCurrDataCounter(DMA2_Stream6, len);  //����д�봫�����ݳ���
    DMA_Cmd(DMA2_Stream6, ENABLE);  //����DMA����

    return len;
}
