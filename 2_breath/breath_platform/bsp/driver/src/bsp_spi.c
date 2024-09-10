/**@file   bsp_spi.c
* @brief   Ӳ��SPI����,DMA,�ж�,
*          ADCоƬ��SPI,�ű�����SPI
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

#define SPI_DMA_SIZE  10
#define SPI2_DMA_SIZE  100

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  spi_handle_t
* @brief   SPI���
*/
typedef struct
{
    SPI_HandleTypeDef spi;
    DMA_HandleTypeDef dma_tx;
    DMA_HandleTypeDef dma_rx;
}spi_handle_t;
/**
* @struct  spi_dma_buf_t
* @brief   spi dma����
*/
typedef struct
{
    uint8_t s2_tx[SPI2_DMA_SIZE];
    uint8_t s2_rx[SPI2_DMA_SIZE];
}spi_dma_buf_t;

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

spi_handle_t  spi2_handle = {0};  //SPI���
spi_dma_buf_t spi_dma_buf;  //SPI,DMA����

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/*spi2*************************************************************************************************/
/* spi2��ʼ�� */
void spi2_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_SPI_DeInit(&spi2_handle.spi);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();
    /* IO���� */
    /**SPI2 GPIO Configuration
    PB12     ------> SPI2_NSS
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    gpio_init_struct.Pin       = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //ģʽ
    gpio_init_struct.Pull      = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF5_SPI2;           //����
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);  //��ʼ��
    /* DMA���� */
    /* Tx DMA���� */
    spi2_handle.dma_tx.Instance                 = DMA1_Channel3;            //������ѡ��
    spi2_handle.dma_tx.Init.Request             = DMA_REQUEST_1;            //�������ã��������ĸ����������
    spi2_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //���䷽��
    spi2_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //���裨�ǣ�����ģʽ
    spi2_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //�洢�����ǣ�����ģʽ
    spi2_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //�������ݴ�С�� 8/16/32 λ
    spi2_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //�洢�����ݴ�С�� 8/16/32 λ
    spi2_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //ģʽ����������ģʽ/ѭ��ģʽ/��ͨģʽ
    spi2_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA ���ȼ�����/��/��/�ǳ���
    HAL_DMA_DeInit(&spi2_handle.dma_tx);  //ȡ����ʼ��
    HAL_DMA_Init(&spi2_handle.dma_tx);    //��ʼ��
    __HAL_LINKDMA(&spi2_handle.spi, hdmatx, spi2_handle.dma_tx);  //��DMA��SPI��ϵ����
    /* Rx DMA���� */
    spi2_handle.dma_rx = spi2_handle.dma_tx;  //����������ͬ
    spi2_handle.dma_rx.Instance                 = DMA1_Channel2;            //������ѡ��
    spi2_handle.dma_rx.Init.Request             = DMA_REQUEST_1;            //�������ã��������ĸ����������
    spi2_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //���䷽��
    HAL_DMA_DeInit(&spi2_handle.dma_rx);  //ȡ����ʼ��
    HAL_DMA_Init(&spi2_handle.dma_rx);    //��ʼ��
    __HAL_LINKDMA(&spi2_handle.spi, hdmarx, spi2_handle.dma_rx);  //��DMA��SPI��ϵ����
    /* SPI���� */
    spi2_handle.spi.Instance                        = SPI2;                                     //�Ĵ�������ַ
    spi2_handle.spi.Init.Mode                       = SPI_MODE_MASTER;                          //ģʽ
    spi2_handle.spi.Init.Direction                  = SPI_DIRECTION_2LINES;                     //SPI˫��ģʽ״̬
    spi2_handle.spi.Init.DataSize                   = SPI_DATASIZE_8BIT;                        //���ݴ�С
    spi2_handle.spi.Init.CLKPolarity                = SPI_POLARITY_LOW;                         //ʱ�ӿ���״̬
    spi2_handle.spi.Init.CLKPhase                   = SPI_PHASE_2EDGE;                          //�ڼ���������
    spi2_handle.spi.Init.NSS                        = SPI_NSS_HARD_OUTPUT;                      //Ӳ��(NSS����)�����ʹ��SSIλ
    spi2_handle.spi.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_16;                 //������Ԥ����ֵ��Ƶ
    spi2_handle.spi.Init.FirstBit                   = SPI_FIRSTBIT_MSB;                         //���ݴ����Ǵ�MSBλ����LSBλ��ʼ
    spi2_handle.spi.Init.TIMode                     = SPI_TIMODE_DISABLE;                       //�Ƿ�����TIģʽ
    spi2_handle.spi.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;               //�Ƿ�����CRC����
    spi2_handle.spi.Init.CRCPolynomial              = 0x0UL;                                    //CRC����Ķ���ʽ
    spi2_handle.spi.Init.CRCLength                  = SPI_CRC_LENGTH_DATASIZE;                  //CRC����
    spi2_handle.spi.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;                     //�Ƿ�����NSSP�ź�
    HAL_SPI_Init(&spi2_handle.spi);  //��ʼ��
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);          //ʹ���ж�
}
/**
* @brief SPI DMA�����ж�
*/
void DMA1_Stream3_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = spi2_handle.spi.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //����HAL���жϴ����ú���
}
/**
* @brief SPI DMA�����ж�
*/
void DMA1_Stream2_IRQHandler(void)
{
    uint32_t len = 0;
    SPI_HandleTypeDef *handle = &spi2_handle.spi;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
        
        HAL_SPI_Abort(handle);  //ֹͣDMA����
        len = handle->RxXferSize - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        spi2_receive(handle->pRxBuffPtr, len);  //�������
        
//        //��DR����Ƶ�����ж�
//        len = SPI2->RXDR;
//        len = SPI2->SR;
    }
    else if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx)) != RESET)  //�봫������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
    }
    else if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
    }
    else  /* �봫������жϴ����п��ܻ������������ж�,�����Ȳ����������ж� */
    {
//        HAL_DMA_IRQHandler(handle->hdmarx);  //����HAL���жϴ����ú���
    }
}
/**
* @brief  SPI �ȴ�����
* @attention
*/
void spi2_wait(void)
{
    while(spi2_handle.spi.State != HAL_SPI_STATE_READY);
}
/**
* @brief  SPI ��д����
* @attention
*/
uint16_t spi2_read_write(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_SPI_GET_FLAG(&spi2_handle.spi, SPI_FLAG_TXE) != SET)  //�ϴη���δ���
    {
        return 0;
    }

    if(len > sizeof(spi_dma_buf.s2_tx))  //���ȳ���DMA������
    {
        len = sizeof(spi_dma_buf.s2_tx);
    }
    i = len;
    while(i--)
    {
        if(data != NULL)
        {
            spi_dma_buf.s2_tx[i] = data[i];
        }
        else
        {
            spi_dma_buf.s2_tx[i] = 0;
        }
    }

    HAL_SPI_TransmitReceive_DMA(&spi2_handle.spi, spi_dma_buf.s2_tx, spi_dma_buf.s2_rx, len);  //txdma������������dma�ж�
    
    return len;
}
/**
* @brief  SPI ��д����
* @attention
*/
uint8_t spi2_read_write_byte(uint8_t data)
{
    if(__HAL_SPI_GET_FLAG(&spi2_handle.spi, SPI_FLAG_TXE) != SET)  //�ϴη���δ���
    {
        return 0;
    }
    
    spi_dma_buf.s2_tx[0] = data;

    HAL_SPI_TransmitReceive_DMA(&spi2_handle.spi, spi_dma_buf.s2_tx, spi_dma_buf.s2_rx, 1);  //txdma������������dma�ж�
    spi2_wait();
    
    return spi_dma_buf.s2_rx[0];
}
/*spi2================================================================================================*/
