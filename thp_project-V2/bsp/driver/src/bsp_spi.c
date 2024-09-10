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
    uint8_t s1_tx[SPI_DMA_SIZE];
    uint8_t s1_rx[SPI_DMA_SIZE];
    uint8_t s2_tx[SPI2_DMA_SIZE];
    uint8_t s2_rx[SPI2_DMA_SIZE];
    uint8_t s3_tx[SPI_DMA_SIZE];
    uint8_t s3_rx[SPI_DMA_SIZE];
    uint8_t s4_tx[SPI_DMA_SIZE];
    uint8_t s4_rx[SPI_DMA_SIZE];
    uint8_t s5_tx[SPI_DMA_SIZE];
    uint8_t s5_rx[SPI_DMA_SIZE];
    uint8_t s6_tx[SPI_DMA_SIZE];
    uint8_t s6_rx[SPI_DMA_SIZE];
}spi_dma_buf_t;

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

spi_handle_t  spi1_handle = {0};  //SPI���
spi_handle_t  spi2_handle = {0};  //SPI���
spi_handle_t  spi3_handle = {0};  //SPI���
spi_handle_t  spi4_handle = {0};  //SPI���
spi_handle_t  spi5_handle = {0};  //SPI���
spi_handle_t  spi6_handle = {0};  //SPI���
spi_dma_buf_t spi_dma_buf __attribute__((at(0x38000000))) = {0} ;  //SPI,DMA����

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/*spi1*************************************************************************************************/
/* spi1��ʼ�� */
void spi1_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_SPI_DeInit(&spi1_handle.spi);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    /* IO���� */
    /**SPI1 GPIO Configuration
    PA4     ------> SPI1_NSS
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    gpio_init_struct.Pin       = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //ģʽ
    gpio_init_struct.Pull      = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF5_SPI1;           //����
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);  //��ʼ��
    /* DMA���� */
    /* Tx DMA���� */
    spi1_handle.dma_tx.Instance                 = DMA1_Stream0;             //������ѡ��
    spi1_handle.dma_tx.Init.Request             = DMA_REQUEST_SPI1_TX;      //�������ã��������ĸ����������
    spi1_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //���䷽��
    spi1_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //���裨�ǣ�����ģʽ
    spi1_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //�洢�����ǣ�����ģʽ
    spi1_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //�������ݴ�С�� 8/16/32 λ
    spi1_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //�洢�����ݴ�С�� 8/16/32 λ
    spi1_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //ģʽ����������ģʽ/ѭ��ģʽ/��ͨģʽ
    spi1_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA ���ȼ�����/��/��/�ǳ���
    spi1_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO ģʽ�������߽�ֹ
    spi1_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO ��ֵѡ��
    spi1_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //�洢��ͻ��ģʽ������/4 ������/8 ������/16 ������
    spi1_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //����ͻ��ģʽ������/4 ������/8 ������/16 ������
    HAL_DMA_DeInit(&spi1_handle.dma_tx);  //ȡ����ʼ��
    HAL_DMA_Init(&spi1_handle.dma_tx);    //��ʼ��
    __HAL_LINKDMA(&spi1_handle.spi, hdmatx, spi1_handle.dma_tx);  //��DMA��SPI��ϵ����
    /* Rx DMA���� */
    spi1_handle.dma_rx = spi1_handle.dma_tx;  //����������ͬ
    spi1_handle.dma_rx.Instance                 = DMA1_Stream1;             //������ѡ��
    spi1_handle.dma_rx.Init.Request             = DMA_REQUEST_SPI1_RX;      //�������ã��������ĸ����������
    spi1_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //���䷽��
    HAL_DMA_DeInit(&spi1_handle.dma_rx);  //ȡ����ʼ��
    HAL_DMA_Init(&spi1_handle.dma_rx);    //��ʼ��
    __HAL_LINKDMA(&spi1_handle.spi, hdmarx, spi1_handle.dma_rx);  //��DMA��SPI��ϵ����
    /* SPI���� */
    spi1_handle.spi.Instance                        = SPI1;                                     //�Ĵ�������ַ
    spi1_handle.spi.Init.Mode                       = SPI_MODE_MASTER;                          //ģʽ
    spi1_handle.spi.Init.Direction                  = SPI_DIRECTION_2LINES;                     //SPI˫��ģʽ״̬
    spi1_handle.spi.Init.DataSize                   = SPI_DATASIZE_8BIT;                        //���ݴ�С
    spi1_handle.spi.Init.CLKPolarity                = SPI_POLARITY_HIGH;                        //ʱ�ӿ���״̬
    spi1_handle.spi.Init.CLKPhase                   = SPI_PHASE_2EDGE;                          //�ڼ���������
    spi1_handle.spi.Init.NSS                        = SPI_NSS_HARD_OUTPUT;                      //Ӳ��(NSS����)�����ʹ��SSIλ
    spi1_handle.spi.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_32;                 //������Ԥ����ֵ��Ƶ
    spi1_handle.spi.Init.FirstBit                   = SPI_FIRSTBIT_MSB;                         //���ݴ����Ǵ�MSBλ����LSBλ��ʼ
    spi1_handle.spi.Init.TIMode                     = SPI_TIMODE_DISABLE;                       //�Ƿ�����TIģʽ
    spi1_handle.spi.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;               //�Ƿ�����CRC����
    spi1_handle.spi.Init.CRCPolynomial              = 0x0UL;                                    //CRC����Ķ���ʽ
    spi1_handle.spi.Init.CRCLength                  = SPI_CRC_LENGTH_DATASIZE;                  //CRC����
    spi1_handle.spi.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;                     //�Ƿ�����NSSP�ź�
    spi1_handle.spi.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;                     //ָ��SS����/����ⲿ�ź���Ч��ƽ
    spi1_handle.spi.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;                //FIFO��ֵ����
    spi1_handle.spi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC��ʼ��ģʽ
    spi1_handle.spi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC��ʼ��ģʽ
    spi1_handle.spi.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;           //ָ�������ӳ�
    spi1_handle.spi.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;    //ָ����Сʱ���ӳ�
    spi1_handle.spi.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;           //��������ģʽ�¿�������SPI���䲢�Զ������Ա��ⳬ�޵����
    spi1_handle.spi.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;         //���ù���gpio״̬����
    spi1_handle.spi.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;                      //��תMISO/MOSI
    HAL_SPI_Init(&spi1_handle.spi);  //��ʼ��
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);          //ʹ���ж�
}
/**
* @brief SPI DMA�����ж�
*/
void DMA1_Stream0_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = spi1_handle.spi.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //����HAL���жϴ����ú���
}
/**
* @brief SPI DMA�����ж�
*/
void DMA1_Stream1_IRQHandler(void)
{
    uint32_t len = 0;
    SPI_HandleTypeDef *handle = &spi1_handle.spi;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
        
        HAL_SPI_Abort(handle);  //ֹͣDMA����
        len = handle->RxXferSize - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        spi1_receive(handle->pRxBuffPtr, len);  //�������
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx)) != RESET)  //�봫������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
    }
}
/**
* @brief  SPI ��д����
* @attention
*/
uint16_t spi1_read_write(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_SPI_GET_FLAG(&spi1_handle.spi, SPI_SR_TXP) != SET)  //�ϴη���δ���
    {
        return 0;
    }

    if(len > sizeof(spi_dma_buf.s1_tx))  //���ȳ���DMA������
    {
        len = sizeof(spi_dma_buf.s1_tx);
    }
    i = len;
    while(i--)
    {
        spi_dma_buf.s1_tx[i] = data[i];
    }

    HAL_SPI_TransmitReceive_DMA(&spi1_handle.spi, spi_dma_buf.s1_tx, spi_dma_buf.s1_rx, len);  //txdma������������dma�ж�
    
    return len;
}
/*spi1================================================================================================*/
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
    spi2_handle.dma_tx.Instance                 = DMA1_Stream2;             //������ѡ��
    spi2_handle.dma_tx.Init.Request             = DMA_REQUEST_SPI2_TX;      //�������ã��������ĸ����������
    spi2_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //���䷽��
    spi2_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //���裨�ǣ�����ģʽ
    spi2_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //�洢�����ǣ�����ģʽ
    spi2_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //�������ݴ�С�� 8/16/32 λ
    spi2_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //�洢�����ݴ�С�� 8/16/32 λ
    spi2_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //ģʽ����������ģʽ/ѭ��ģʽ/��ͨģʽ
    spi2_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA ���ȼ�����/��/��/�ǳ���
    spi2_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO ģʽ�������߽�ֹ
    spi2_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO ��ֵѡ��
    spi2_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //�洢��ͻ��ģʽ������/4 ������/8 ������/16 ������
    spi2_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //����ͻ��ģʽ������/4 ������/8 ������/16 ������
    HAL_DMA_DeInit(&spi2_handle.dma_tx);  //ȡ����ʼ��
    HAL_DMA_Init(&spi2_handle.dma_tx);    //��ʼ��
    __HAL_LINKDMA(&spi2_handle.spi, hdmatx, spi2_handle.dma_tx);  //��DMA��SPI��ϵ����
    /* Rx DMA���� */
    spi2_handle.dma_rx = spi2_handle.dma_tx;  //����������ͬ
    spi2_handle.dma_rx.Instance                 = DMA1_Stream3;             //������ѡ��
    spi2_handle.dma_rx.Init.Request             = DMA_REQUEST_SPI2_RX;      //�������ã��������ĸ����������
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
    spi2_handle.spi.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;                     //ָ��SS����/����ⲿ�ź���Ч��ƽ
    spi2_handle.spi.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;                //FIFO��ֵ����
    spi2_handle.spi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC��ʼ��ģʽ
    spi2_handle.spi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC��ʼ��ģʽ
    spi2_handle.spi.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;           //ָ�������ӳ�
    spi2_handle.spi.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;    //ָ����Сʱ���ӳ�
    spi2_handle.spi.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;           //��������ģʽ�¿�������SPI���䲢�Զ������Ա��ⳬ�޵����
    spi2_handle.spi.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;         //���ù���gpio״̬����
    spi2_handle.spi.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;                      //��תMISO/MOSI
    HAL_SPI_Init(&spi2_handle.spi);  //��ʼ��
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);          //ʹ���ж�
}
/**
* @brief SPI DMA�����ж�
*/
void DMA1_Stream2_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = spi2_handle.spi.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //����HAL���жϴ����ú���
}
/**
* @brief SPI DMA�����ж�
*/
void DMA1_Stream3_IRQHandler(void)
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
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx)) != RESET)  //�봫������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
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
    if(__HAL_SPI_GET_FLAG(&spi2_handle.spi, SPI_SR_TXP) != SET)  //�ϴη���δ���
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
    if(__HAL_SPI_GET_FLAG(&spi2_handle.spi, SPI_SR_TXP) != SET)  //�ϴη���δ���
    {
        return 0;
    }
    
    spi_dma_buf.s2_tx[0] = data;

    HAL_SPI_TransmitReceive_DMA(&spi2_handle.spi, spi_dma_buf.s2_tx, spi_dma_buf.s2_rx, 1);  //txdma������������dma�ж�
    spi2_wait();
    
    return spi_dma_buf.s2_rx[0];
}
/*spi2================================================================================================*/
/*spi3*************************************************************************************************/
/* spi3��ʼ�� */
void spi3_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_SPI_DeInit(&spi3_handle.spi);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_SPI3_CLK_ENABLE();
    /* IO���� */
    /**SPI3 GPIO Configuration
    PA15 (JTDI)     ------> SPI3_NSS
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    gpio_init_struct.Pin       = GPIO_PIN_15; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //ģʽ
    gpio_init_struct.Pull      = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF6_SPI3;           //����
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);  //��ʼ��
    gpio_init_struct.Pin       = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12; //���ź�
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);  //��ʼ��
    /* DMA���� */
    /* Tx DMA���� */
    spi3_handle.dma_tx.Instance                 = DMA1_Stream4;             //������ѡ��
    spi3_handle.dma_tx.Init.Request             = DMA_REQUEST_SPI3_TX;      //�������ã��������ĸ����������
    spi3_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //���䷽��
    spi3_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //���裨�ǣ�����ģʽ
    spi3_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //�洢�����ǣ�����ģʽ
    spi3_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //�������ݴ�С�� 8/16/32 λ
    spi3_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //�洢�����ݴ�С�� 8/16/32 λ
    spi3_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //ģʽ����������ģʽ/ѭ��ģʽ/��ͨģʽ
    spi3_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA ���ȼ�����/��/��/�ǳ���
    spi3_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO ģʽ�������߽�ֹ
    spi3_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO ��ֵѡ��
    spi3_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //�洢��ͻ��ģʽ������/4 ������/8 ������/16 ������
    spi3_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //����ͻ��ģʽ������/4 ������/8 ������/16 ������
    HAL_DMA_DeInit(&spi3_handle.dma_tx);  //ȡ����ʼ��
    HAL_DMA_Init(&spi3_handle.dma_tx);    //��ʼ��
    __HAL_LINKDMA(&spi3_handle.spi, hdmatx, spi3_handle.dma_tx);  //��DMA��SPI��ϵ����
    /* Rx DMA���� */
    spi3_handle.dma_rx = spi3_handle.dma_tx;  //����������ͬ
    spi3_handle.dma_rx.Instance                 = DMA1_Stream5;             //������ѡ��
    spi3_handle.dma_rx.Init.Request             = DMA_REQUEST_SPI3_RX;      //�������ã��������ĸ����������
    spi3_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //���䷽��
    HAL_DMA_DeInit(&spi3_handle.dma_rx);  //ȡ����ʼ��
    HAL_DMA_Init(&spi3_handle.dma_rx);    //��ʼ��
    __HAL_LINKDMA(&spi3_handle.spi, hdmarx, spi3_handle.dma_rx);  //��DMA��SPI��ϵ����
    /* SPI���� */
    spi3_handle.spi.Instance                        = SPI3;                                     //�Ĵ�������ַ
    spi3_handle.spi.Init.Mode                       = SPI_MODE_MASTER;                          //ģʽ
    spi3_handle.spi.Init.Direction                  = SPI_DIRECTION_2LINES;                     //SPI˫��ģʽ״̬
    spi3_handle.spi.Init.DataSize                   = SPI_DATASIZE_8BIT;                        //���ݴ�С
    spi3_handle.spi.Init.CLKPolarity                = SPI_POLARITY_HIGH;                        //ʱ�ӿ���״̬
    spi3_handle.spi.Init.CLKPhase                   = SPI_PHASE_2EDGE;                          //�ڼ���������
    spi3_handle.spi.Init.NSS                        = SPI_NSS_HARD_OUTPUT;                      //Ӳ��(NSS����)�����ʹ��SSIλ
    spi3_handle.spi.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_32;                 //������Ԥ����ֵ��Ƶ
    spi3_handle.spi.Init.FirstBit                   = SPI_FIRSTBIT_MSB;                         //���ݴ����Ǵ�MSBλ����LSBλ��ʼ
    spi3_handle.spi.Init.TIMode                     = SPI_TIMODE_DISABLE;                       //�Ƿ�����TIģʽ
    spi3_handle.spi.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;               //�Ƿ�����CRC����
    spi3_handle.spi.Init.CRCPolynomial              = 0x0UL;                                    //CRC����Ķ���ʽ
    spi3_handle.spi.Init.CRCLength                  = SPI_CRC_LENGTH_DATASIZE;                  //CRC����
    spi3_handle.spi.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;                     //�Ƿ�����NSSP�ź�
    spi3_handle.spi.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;                     //ָ��SS����/����ⲿ�ź���Ч��ƽ
    spi3_handle.spi.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;                //FIFO��ֵ����
    spi3_handle.spi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC��ʼ��ģʽ
    spi3_handle.spi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC��ʼ��ģʽ
    spi3_handle.spi.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;           //ָ�������ӳ�
    spi3_handle.spi.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;    //ָ����Сʱ���ӳ�
    spi3_handle.spi.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;           //��������ģʽ�¿�������SPI���䲢�Զ������Ա��ⳬ�޵����
    spi3_handle.spi.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;         //���ù���gpio״̬����
    spi3_handle.spi.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;                      //��תMISO/MOSI
    HAL_SPI_Init(&spi3_handle.spi);  //��ʼ��
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);          //ʹ���ж�
}
/**
* @brief SPI DMA�����ж�
*/
void DMA1_Stream4_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = spi3_handle.spi.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //����HAL���жϴ����ú���
}
/**
* @brief SPI DMA�����ж�
*/
void DMA1_Stream5_IRQHandler(void)
{
    uint32_t len = 0;
    SPI_HandleTypeDef *handle = &spi3_handle.spi;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
        
        HAL_SPI_Abort(handle);  //ֹͣDMA����
        len = handle->RxXferSize - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        spi3_receive(handle->pRxBuffPtr, len);  //�������
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx)) != RESET)  //�봫������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
    }
}
/**
* @brief  SPI ��д����
* @attention
*/
uint16_t spi3_read_write(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_SPI_GET_FLAG(&spi3_handle.spi, SPI_SR_TXP) != SET)  //�ϴη���δ���
    {
        return 0;
    }

    if(len > sizeof(spi_dma_buf.s3_tx))  //���ȳ���DMA������
    {
        len = sizeof(spi_dma_buf.s3_tx);
    }
    i = len;
    while(i--)
    {
        spi_dma_buf.s3_tx[i] = data[i];
    }

    HAL_SPI_TransmitReceive_DMA(&spi3_handle.spi, spi_dma_buf.s3_tx, spi_dma_buf.s3_rx, len);  //txdma������������dma�ж�
    
    return len;
}
/*spi3================================================================================================*/
/*spi4*************************************************************************************************/
/* spi4��ʼ�� */
void spi4_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_SPI_DeInit(&spi4_handle.spi);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_SPI4_CLK_ENABLE();
    /* IO���� */
    /**SPI4 GPIO Configuration
    PE11     ------> SPI4_NSS
    PE12     ------> SPI4_SCK
    PE13     ------> SPI4_MISO
    PE14     ------> SPI4_MOSI
    */
    gpio_init_struct.Pin       = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //ģʽ
    gpio_init_struct.Pull      = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF5_SPI4;           //����
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);  //��ʼ��
    /* DMA���� */
    /* Tx DMA���� */
    spi4_handle.dma_tx.Instance                 = DMA1_Stream6;             //������ѡ��
    spi4_handle.dma_tx.Init.Request             = DMA_REQUEST_SPI4_TX;      //�������ã��������ĸ����������
    spi4_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //���䷽��
    spi4_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //���裨�ǣ�����ģʽ
    spi4_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //�洢�����ǣ�����ģʽ
    spi4_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //�������ݴ�С�� 8/16/32 λ
    spi4_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //�洢�����ݴ�С�� 8/16/32 λ
    spi4_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //ģʽ����������ģʽ/ѭ��ģʽ/��ͨģʽ
    spi4_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA ���ȼ�����/��/��/�ǳ���
    spi4_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO ģʽ�������߽�ֹ
    spi4_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO ��ֵѡ��
    spi4_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //�洢��ͻ��ģʽ������/4 ������/8 ������/16 ������
    spi4_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //����ͻ��ģʽ������/4 ������/8 ������/16 ������
    HAL_DMA_DeInit(&spi4_handle.dma_tx);  //ȡ����ʼ��
    HAL_DMA_Init(&spi4_handle.dma_tx);    //��ʼ��
    __HAL_LINKDMA(&spi4_handle.spi, hdmatx, spi4_handle.dma_tx);  //��DMA��SPI��ϵ����
    /* Rx DMA���� */
    spi4_handle.dma_rx = spi4_handle.dma_tx;  //����������ͬ
    spi4_handle.dma_rx.Instance                 = DMA1_Stream7;             //������ѡ��
    spi4_handle.dma_rx.Init.Request             = DMA_REQUEST_SPI4_RX;      //�������ã��������ĸ����������
    spi4_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //���䷽��
    HAL_DMA_DeInit(&spi4_handle.dma_rx);  //ȡ����ʼ��
    HAL_DMA_Init(&spi4_handle.dma_rx);    //��ʼ��
    __HAL_LINKDMA(&spi4_handle.spi, hdmarx, spi4_handle.dma_rx);  //��DMA��SPI��ϵ����
    /* SPI���� */
    spi4_handle.spi.Instance                        = SPI4;                                     //�Ĵ�������ַ
    spi4_handle.spi.Init.Mode                       = SPI_MODE_MASTER;                          //ģʽ
    spi4_handle.spi.Init.Direction                  = SPI_DIRECTION_2LINES;                     //SPI˫��ģʽ״̬
    spi4_handle.spi.Init.DataSize                   = SPI_DATASIZE_8BIT;                        //���ݴ�С
    spi4_handle.spi.Init.CLKPolarity                = SPI_POLARITY_HIGH;                        //ʱ�ӿ���״̬
    spi4_handle.spi.Init.CLKPhase                   = SPI_PHASE_2EDGE;                          //�ڼ���������
    spi4_handle.spi.Init.NSS                        = SPI_NSS_HARD_OUTPUT;                      //Ӳ��(NSS����)�����ʹ��SSIλ
    spi4_handle.spi.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_32;                 //������Ԥ����ֵ��Ƶ
    spi4_handle.spi.Init.FirstBit                   = SPI_FIRSTBIT_MSB;                         //���ݴ����Ǵ�MSBλ����LSBλ��ʼ
    spi4_handle.spi.Init.TIMode                     = SPI_TIMODE_DISABLE;                       //�Ƿ�����TIģʽ
    spi4_handle.spi.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;               //�Ƿ�����CRC����
    spi4_handle.spi.Init.CRCPolynomial              = 0x0UL;                                    //CRC����Ķ���ʽ
    spi4_handle.spi.Init.CRCLength                  = SPI_CRC_LENGTH_DATASIZE;                  //CRC����
    spi4_handle.spi.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;                     //�Ƿ�����NSSP�ź�
    spi4_handle.spi.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;                     //ָ��SS����/����ⲿ�ź���Ч��ƽ
    spi4_handle.spi.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;                //FIFO��ֵ����
    spi4_handle.spi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC��ʼ��ģʽ
    spi4_handle.spi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC��ʼ��ģʽ
    spi4_handle.spi.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;           //ָ�������ӳ�
    spi4_handle.spi.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;    //ָ����Сʱ���ӳ�
    spi4_handle.spi.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;           //��������ģʽ�¿�������SPI���䲢�Զ������Ա��ⳬ�޵����
    spi4_handle.spi.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;         //���ù���gpio״̬����
    spi4_handle.spi.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;                      //��תMISO/MOSI
    HAL_SPI_Init(&spi4_handle.spi);  //��ʼ��
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);          //ʹ���ж�
}
/**
* @brief SPI DMA�����ж�
*/
void DMA1_Stream6_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = spi4_handle.spi.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //����HAL���жϴ����ú���
}
/**
* @brief SPI DMA�����ж�
*/
void DMA1_Stream7_IRQHandler(void)
{
    uint32_t len = 0;
    SPI_HandleTypeDef *handle = &spi4_handle.spi;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
        
        HAL_SPI_Abort(handle);  //ֹͣDMA����
        len = handle->RxXferSize - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        spi4_receive(handle->pRxBuffPtr, len);  //�������
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx)) != RESET)  //�봫������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
    }
}
/**
* @brief  SPI ��д����
* @attention
*/
uint16_t spi4_read_write(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_SPI_GET_FLAG(&spi4_handle.spi, SPI_SR_TXP) != SET)  //�ϴη���δ���
    {
        return 0;
    }

    if(len > sizeof(spi_dma_buf.s4_tx))  //���ȳ���DMA������
    {
        len = sizeof(spi_dma_buf.s4_tx);
    }
    i = len;
    while(i--)
    {
        spi_dma_buf.s4_tx[i] = data[i];
    }

    HAL_SPI_TransmitReceive_DMA(&spi4_handle.spi, spi_dma_buf.s4_tx, spi_dma_buf.s4_rx, len);  //txdma������������dma�ж�
    
    return len;
}
/*spi4================================================================================================*/
/*spi6*************************************************************************************************/
/* spi6��ʼ�� */
void spi6_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_SPI_DeInit(&spi6_handle.spi);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_BDMA_CLK_ENABLE();
    __HAL_RCC_SPI6_CLK_ENABLE();
    /* IO���� */
    /**SPI6 GPIO Configuration
    PB3 (JTDO/TRACESWO)     ------> SPI6_SCK
    PB4 (NJTRST)     ------> SPI6_MISO
    PB5     ------> SPI6_MOSI
    PD7     ------> SPI6_NSS
    */
    gpio_init_struct.Pin       = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //ģʽ
    gpio_init_struct.Pull      = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF8_SPI6;           //����
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);  //��ʼ��
    gpio_init_struct.Pin       = GPIO_PIN_7; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_OUTPUT_PP;     //ģʽ
    gpio_init_struct.Pull      = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);  //��ʼ��
    /* DMA���� */
    /* Tx DMA���� */
    spi6_handle.dma_tx.Instance                 = BDMA_Channel0;            //������ѡ��
    spi6_handle.dma_tx.Init.Request             = BDMA_REQUEST_SPI6_TX;     //�������ã��������ĸ����������
    spi6_handle.dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;     //���䷽��
    spi6_handle.dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;         //���裨�ǣ�����ģʽ
    spi6_handle.dma_tx.Init.MemInc              = DMA_MINC_ENABLE;          //�洢�����ǣ�����ģʽ
    spi6_handle.dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      //�������ݴ�С�� 8/16/32 λ
    spi6_handle.dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      //�洢�����ݴ�С�� 8/16/32 λ
    spi6_handle.dma_tx.Init.Mode                = DMA_NORMAL;               //ģʽ����������ģʽ/ѭ��ģʽ/��ͨģʽ
    spi6_handle.dma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;      //DMA ���ȼ�����/��/��/�ǳ���
    spi6_handle.dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;     //FIFO ģʽ�������߽�ֹ
    spi6_handle.dma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;  //FIFO ��ֵѡ��
    spi6_handle.dma_tx.Init.MemBurst            = DMA_MBURST_SINGLE;        //�洢��ͻ��ģʽ������/4 ������/8 ������/16 ������
    spi6_handle.dma_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;        //����ͻ��ģʽ������/4 ������/8 ������/16 ������
    HAL_DMA_DeInit(&spi6_handle.dma_tx);  //ȡ����ʼ��
    HAL_DMA_Init(&spi6_handle.dma_tx);    //��ʼ��
    __HAL_LINKDMA(&spi6_handle.spi, hdmatx, spi6_handle.dma_tx);  //��DMA��SPI��ϵ����
    /* Rx DMA���� */
    spi6_handle.dma_rx = spi6_handle.dma_tx;  //����������ͬ
    spi6_handle.dma_rx.Instance                 = BDMA_Channel1;             //������ѡ��
    spi6_handle.dma_rx.Init.Request             = BDMA_REQUEST_SPI6_RX;     //�������ã��������ĸ����������
    spi6_handle.dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;     //���䷽��
    HAL_DMA_DeInit(&spi6_handle.dma_rx);  //ȡ����ʼ��
    HAL_DMA_Init(&spi6_handle.dma_rx);    //��ʼ��
    __HAL_LINKDMA(&spi6_handle.spi, hdmarx, spi6_handle.dma_rx);  //��DMA��SPI��ϵ����
    /* SPI���� */
    spi6_handle.spi.Instance                        = SPI6;                                     //�Ĵ�������ַ
    spi6_handle.spi.Init.Mode                       = SPI_MODE_MASTER;                          //ģʽ
    spi6_handle.spi.Init.Direction                  = SPI_DIRECTION_2LINES;                     //SPI˫��ģʽ״̬
    spi6_handle.spi.Init.DataSize                   = SPI_DATASIZE_8BIT;                        //���ݴ�С
    spi6_handle.spi.Init.CLKPolarity                = SPI_POLARITY_HIGH;                        //ʱ�ӿ���״̬
    spi6_handle.spi.Init.CLKPhase                   = SPI_PHASE_2EDGE;                          //�ڼ���������
    spi6_handle.spi.Init.NSS                        = SPI_NSS_SOFT;                             //Ӳ��(NSS����)�����ʹ��SSIλ
    spi6_handle.spi.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_32;                 //������Ԥ����ֵ��Ƶ
    spi6_handle.spi.Init.FirstBit                   = SPI_FIRSTBIT_MSB;                         //���ݴ����Ǵ�MSBλ����LSBλ��ʼ
    spi6_handle.spi.Init.TIMode                     = SPI_TIMODE_DISABLE;                       //�Ƿ�����TIģʽ
    spi6_handle.spi.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;               //�Ƿ�����CRC����
    spi6_handle.spi.Init.CRCPolynomial              = 0x0UL;                                    //CRC����Ķ���ʽ
    spi6_handle.spi.Init.CRCLength                  = SPI_CRC_LENGTH_DATASIZE;                  //CRC����
    spi6_handle.spi.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;                     //�Ƿ�����NSSP�ź�
    spi6_handle.spi.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;                     //ָ��SS����/����ⲿ�ź���Ч��ƽ
    spi6_handle.spi.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;                //FIFO��ֵ����
    spi6_handle.spi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC��ʼ��ģʽ
    spi6_handle.spi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;  //CRC��ʼ��ģʽ
    spi6_handle.spi.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;           //ָ�������ӳ�
    spi6_handle.spi.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;    //ָ����Сʱ���ӳ�
    spi6_handle.spi.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;           //��������ģʽ�¿�������SPI���䲢�Զ������Ա��ⳬ�޵����
    spi6_handle.spi.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;         //���ù���gpio״̬����
    spi6_handle.spi.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;                      //��תMISO/MOSI
    HAL_SPI_Init(&spi6_handle.spi);  //��ʼ��
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(BDMA_Channel1_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(BDMA_Channel1_IRQn);          //ʹ���ж�
    /* ���� */
    SPI6_NSS(1);
}
/**
* @brief SPI DMA�����ж�
*/
void BDMA_Channel0_IRQHandler(void)
{
    DMA_HandleTypeDef *handle = spi6_handle.spi.hdmatx;
    
    HAL_DMA_IRQHandler(handle);  //����HAL���жϴ����ú���
}
/**
* @brief SPI DMA�����ж�
*/
void BDMA_Channel1_IRQHandler(void)
{
    uint32_t len = 0;
    SPI_HandleTypeDef *handle = &spi6_handle.spi;

    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
        
        HAL_SPI_Abort(handle);  //ֹͣDMA����
        len = handle->RxXferSize - __HAL_DMA_GET_COUNTER(handle->hdmarx); //��ȡ���յ����ݴ�С
        spi6_receive(handle->pRxBuffPtr, len);  //�������
        
        SPI6_NSS(1);
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx)) != RESET)  //�봫������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
    }
    if(__HAL_DMA_GET_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx)) != RESET)  //��������ж�
    {
        __HAL_DMA_CLEAR_FLAG(handle->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(handle->hdmarx));  //����жϱ�־λ
    }
}
/**
* @brief  SPI ��д����
* @attention
*/
uint16_t spi6_read_write(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    
    if(len == 0)
    {
        return 0;
    }
    if(__HAL_SPI_GET_FLAG(&spi6_handle.spi, SPI_SR_TXP) != SET)  //�ϴη���δ���
    {
        return 0;
    }

    if(len > sizeof(spi_dma_buf.s6_tx))  //���ȳ���DMA������
    {
        len = sizeof(spi_dma_buf.s6_tx);
    }
    i = len;
    while(i--)
    {
        spi_dma_buf.s6_tx[i] = data[i];
    }
    SPI6_NSS(0);
    HAL_SPI_TransmitReceive_DMA(&spi6_handle.spi, spi_dma_buf.s6_tx, spi_dma_buf.s6_rx, len);  //txdma������������dma�ж�
    
    return len;
}
/*spi6================================================================================================*/
