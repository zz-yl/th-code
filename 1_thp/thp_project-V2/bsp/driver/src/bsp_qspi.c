/**@file   bsp_qspi.c
* @brief   Ӳ��QSPI����,DMA,�ж�,
*          �ű�����SPI
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_qspi.h"
#include "bsp_cfg.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define QSPI_DMA_SIZE  10

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  qspi_handle_t
* @brief   QSPI���
*/
typedef struct
{
    QSPI_HandleTypeDef qspi;
    MDMA_HandleTypeDef mdma;
}qspi_handle_t;

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

qspi_handle_t  qspi_handle = {0};  //QSPI���
uint8_t qspi_dma_buf[QSPI_DMA_SIZE] __attribute__((at(0x38000200))) = {0} ;  //QSPI,DMA����

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/* qspi��ʼ�� */
void qspi_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_QSPI_DeInit(&qspi_handle.qspi);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_MDMA_CLK_ENABLE();
    __HAL_RCC_QSPI_CLK_ENABLE();
    /* IO���� */
    gpio_init_struct.Pin       = GPIO_PIN_2 | GPIO_PIN_10; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;         //ģʽ
    gpio_init_struct.Pull      = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF9_QUADSPI;        //����
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);  //��ʼ��
    gpio_init_struct.Pin       = GPIO_PIN_11 | GPIO_PIN_12; //���ź�
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);  //��ʼ��
    /* MDMA���� */
    qspi_handle.mdma.Instance                      = MDMA_Channel0;                  //������ѡ��
    qspi_handle.mdma.Init.Request                  = MDMA_REQUEST_QUADSPI_FIFO_TH;   //�������ã��������ĸ����������
    qspi_handle.mdma.Init.TransferTriggerMode      = MDMA_BUFFER_TRANSFER;
    qspi_handle.mdma.Init.Priority                 = MDMA_PRIORITY_LOW;
    qspi_handle.mdma.Init.Endianness               = MDMA_LITTLE_ENDIANNESS_PRESERVE;
    qspi_handle.mdma.Init.SourceInc                = MDMA_SRC_INC_BYTE;
    qspi_handle.mdma.Init.DestinationInc           = MDMA_DEST_INC_DISABLE;
    qspi_handle.mdma.Init.SourceDataSize           = MDMA_SRC_DATASIZE_BYTE;
    qspi_handle.mdma.Init.DestDataSize             = MDMA_DEST_DATASIZE_BYTE;
    qspi_handle.mdma.Init.DataAlignment            = MDMA_DATAALIGN_PACKENABLE;
    qspi_handle.mdma.Init.BufferTransferLength     = 1;
    qspi_handle.mdma.Init.SourceBurst              = MDMA_SOURCE_BURST_SINGLE;
    qspi_handle.mdma.Init.DestBurst                = MDMA_DEST_BURST_SINGLE;
    qspi_handle.mdma.Init.SourceBlockAddressOffset = 0;
    qspi_handle.mdma.Init.DestBlockAddressOffset   = 0;
    HAL_MDMA_DeInit(&qspi_handle.mdma);  //ȡ����ʼ��
    if(HAL_MDMA_Init(&qspi_handle.mdma) != HAL_OK)      //��ʼ��
    {
        Error_Handler();
    }
    if(HAL_MDMA_ConfigPostRequestMask(&qspi_handle.mdma, 0, 0) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_LINKDMA(&qspi_handle.qspi, hmdma, qspi_handle.mdma);  //��MDMA��QSPI��ϵ����
    /* QSPI���� */
    qspi_handle.qspi.Instance                = QUADSPI;
    qspi_handle.qspi.Init.ClockPrescaler     = 31;
    qspi_handle.qspi.Init.FifoThreshold      = 1;
    qspi_handle.qspi.Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
    qspi_handle.qspi.Init.FlashSize          = 8;
    qspi_handle.qspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
    qspi_handle.qspi.Init.ClockMode          = QSPI_CLOCK_MODE_3;
    qspi_handle.qspi.Init.FlashID            = QSPI_FLASH_ID_1;
    qspi_handle.qspi.Init.DualFlash          = QSPI_DUALFLASH_DISABLE;
    if(HAL_QSPI_Init(&qspi_handle.qspi) != HAL_OK)
    {
        Error_Handler();
    }
    /* DMA�ж����� */
    HAL_NVIC_SetPriority(QUADSPI_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(QUADSPI_IRQn);          //ʹ���ж�
}
/**
* @brief QSPI DMA�����ж�
*/
void MDMA_IRQHandler(void)
{
    uint32_t len = 0;
    QSPI_HandleTypeDef *handle = &qspi_handle.qspi;

    if(__HAL_MDMA_GET_FLAG(handle->hmdma, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hmdma)) != RESET)  //��������ж�
    {
        __HAL_MDMA_CLEAR_FLAG(handle->hmdma, __HAL_DMA_GET_TC_FLAG_INDEX(handle->hmdma));  //����жϱ�־λ
        
        HAL_QSPI_Abort(handle);  //ֹͣDMA����
        len = QSPI_DMA_SIZE - __HAL_MDMA_GET_COUNTER(handle->hmdma); //��ȡ���յ����ݴ�С
        qspi_receive(handle->pRxBuffPtr, len);  //�������
        
        //��DR����Ƶ�����ж�
        len = QUADSPI->DR;
        len = QUADSPI->SR;
    }
    else
    {
        HAL_MDMA_IRQHandler(handle->hmdma);  //����HAL���жϴ����ú���
    }
}
/**
* @brief  QSPI ��д����
* @attention
*/
uint16_t qspi_read_write(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    QSPI_CommandTypeDef cmd = 
    {
		.InstructionMode    = QSPI_INSTRUCTION_1_LINE,
		.Instruction        = 0x83,
		.SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
		.DataMode = QSPI_DATA_1_LINE,
		.NbData = len,
	};
    
    if(len == 0)
    {
        return 0;
    }
//    if(__HAL_QSPI_GET_FLAG(&qspi_handle.qspi, SPI_SR_TXP) != SET)  //�ϴη���δ���
//    {
//        return 0;
//    }

    if(len > QSPI_DMA_SIZE)  //���ȳ���DMA������
    {
        len = QSPI_DMA_SIZE;
    }
    i = len;
    while(i--)
    {
        qspi_dma_buf[i] = data[i];
    }

	HAL_QSPI_Command(&qspi_handle.qspi, &cmd, 100);
    HAL_QSPI_Receive_DMA(&qspi_handle.qspi, qspi_dma_buf);
    
    return len;
}

