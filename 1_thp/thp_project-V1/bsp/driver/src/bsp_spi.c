/**@file   bsp_spi.c
* @brief   Ӳ��SPI����,DMA,�ж�,
*          ���ADCоƬ��SPI����
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
* @brief   SPI��DMA���ݽṹ
*/
typedef struct
{
    uint16_t Rx[SPI_DMA_RX_SIZE];
    uint16_t Tx[SPI_DMA_TX_SIZE];
}SPI_DMA_DATA;

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

SPI_DMA_DATA Spi1Dma = {0};  //SPI�շ�DMA����

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/*spi1*************************************************************************************************/
/* SPI1��ʼ�� */
void Spi1Init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_InitTypeDef  SPI_InitStruct;
    DMA_InitTypeDef  DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    GPIO_StructInit(&GPIO_InitStruct);
    SPI_StructInit(&SPI_InitStruct);
    DMA_StructInit(&DMA_InitStruct);
    
    DMA_DeInit(DMA2_Stream0);
    DMA_DeInit(DMA2_Stream5);
    /* ʱ������ */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //ʹ��GPIOʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);  //ʹ��SPIʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  //ʹ��DMAʱ��
    /* GPIO���� */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;  //ͨ��
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       //IOģʽ,���ù���
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO��������·��Ӧ�ٶ�100MHz
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //�������
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //����
    GPIO_Init(GPIOA, &GPIO_InitStruct);              //��ʼ��
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);  //����
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);  //����
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);  //����
    /* SPI���� */
    SPI_InitStruct.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;  //SPI����Ϊ˫��˫��ȫ˫��
    SPI_InitStruct.SPI_Mode              = SPI_Mode_Master;                  //SPI����ģʽΪ��SPI
    SPI_InitStruct.SPI_DataSize          = SPI_DataSize_8b;                  //SPI�շ����ݿ��xλ
    SPI_InitStruct.SPI_CPOL              = SPI_CPOL_Low;                     //����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ
    SPI_InitStruct.SPI_CPHA              = SPI_CPHA_2Edge;                   //����ͬ��ʱ�ӵĵ�x�������أ��������½������ݱ�����
    SPI_InitStruct.SPI_NSS               = SPI_NSS_Soft;                     //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;         //���岨����Ԥ��Ƶ��ֵ
    SPI_InitStruct.SPI_FirstBit          = SPI_FirstBit_MSB;                 //���ݴ����MSBλ��ʼ
    SPI_InitStruct.SPI_CRCPolynomial     = 7;                                //CRCֵ����Ķ���ʽ
    SPI_Init(SPI1, &SPI_InitStruct);                                         //��ʼ��

    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);     //ʹ��SPI��DMA�շ�
    /* DMA���� */
    /* RxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_3;                   //ͨ��
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(SPI1->DR);           //����Ĵ���(����ģʽ)����ַ
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Spi1Dma.Rx;            //�洢������ַ
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralToMemory;      //DMA���䷽��,���赽�洢��
    DMA_InitStruct.DMA_BufferSize         = 0;                               //������ֽ���
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //�����ַ����,�ر�
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //�洢����ַ����,ʹ��
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //�������ݿ��
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;         //�洢�����ݿ��
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;                 //��ѭ��ģʽ
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;               //���ȼ�
    DMA_InitStruct.DMA_FIFOMode           = DMA_FIFOMode_Disable;            //FIFO,�ر�
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;  //FIFO��ֵ����,1/4
    DMA_InitStruct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;          //�洢��ͻ����������,���δ���
    DMA_InitStruct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;      //����ͻ����������,���δ���
    DMA_Init(DMA2_Stream0, &DMA_InitStruct);                                 //��ʼ��
    /* TxDMA */
    DMA_InitStruct.DMA_Channel            = DMA_Channel_3;                   //ͨ��
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)Spi1Dma.Tx;            //�洢������ַ
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;      //DMA���䷽��,�洢��������
    DMA_InitStruct.DMA_BufferSize         = 0;                               //������ֽ���,��ʼ��ʱ������
    DMA_Init(DMA2_Stream5, &DMA_InitStruct);                                 //��ʼ��
    /* DMA�ж����� */
    /* RxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream0_IRQn;  //�ж���
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;               //��ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x03;               //�����ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;             //ʹ��
    NVIC_Init(&NVIC_InitStruct);                                            //��ʼ��
    /* TxDMA */
    NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream5_IRQn;  //�ж���
    NVIC_Init(&NVIC_InitStruct);                                            //��ʼ��
    
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);  //DMA�ж�ʹ��,��������ж�
    DMA_ITConfig(DMA2_Stream5, DMA_IT_TC | DMA_IT_FE, ENABLE);  //DMA�ж�ʹ��,��������ж�,���������ж�
    /* �������� */
    SPI_Cmd(SPI1, ENABLE);           //ʹ��SPI����
    DMA_Cmd(DMA2_Stream0, DISABLE);  //��ֹDMA����,Rx
    DMA_Cmd(DMA2_Stream5, DISABLE);  //��ֹDMA����,Tx
}
/**
* @brief DMA2������0�ж�,Spi1Rx
*/
void DMA2_Stream0_IRQHandler(void)
{
    uint16_t clean = 0;
    
    if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
        
//        Max1168GetBuf(Spi1Dma.Rx);
        
        //��״̬
        clean = SPI1->DR;
        clean = SPI1->SR;
    }
}
/**
* @brief DMA2������5�ж�,Spi1Tx
*/
void DMA2_Stream5_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
        
        DMA_Cmd(DMA2_Stream5, DISABLE);  //��ֹDMA
    }
    if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_FEIF5) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_FEIF5);
        
    }
}
/**
* @brief  SPI1������
* @attention ����:len:����;�趨Rx��DMA����,������
*/
void Spi1Read(uint16_t len)
{
    DMA_Cmd(DMA2_Stream0, DISABLE);             //��ֹ����DMA
    DMA_SetCurrDataCounter(DMA2_Stream0, len);  //����DMA���ճ���
    DMA_Cmd(DMA2_Stream0, ENABLE);              //ʹ�ܽ���DMA
}
/**
* @brief  SPI1д����
* @attention ����:data:����,len:����
*/
void Spi1Write(uint16_t *data, uint16_t len)
{
    uint16_t cnt = 0;

    if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) != RESET)  //����æ
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

    DMA_Cmd(DMA2_Stream5, DISABLE);  //��ֹDMA
    DMA_SetCurrDataCounter(DMA2_Stream5, cnt);  //���ó���
    DMA_Cmd(DMA2_Stream5, ENABLE);  //ʹ��DMA
}
/**
* @brief  SPI1 ��дһ���ֽ�
* @attention ����:tx_data:Ҫд����ֽ�;����ֵ:��ȡ�����ֽ�
*/
uint8_t Spi1ReadWriteByte(uint8_t tx_data)
{
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//�ȴ���������
    SPI_I2S_SendData(SPI1, tx_data); //ͨ������SPIx����һ��byte����
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} //�ȴ�������һ��byte

    return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����
}

