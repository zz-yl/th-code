/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */
#include <math.h>
float offsetPre1 = 1.2 ,offsetPre2 = 3;
/* USER CODE END 0 */

SPI_HandleTypeDef hspi2;

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint8_t SpiReadWriteByte(uint8_t txdata)
{
	uint8_t rxdata;
	HAL_SPI_TransmitReceive(&hspi2, &txdata, &rxdata, 1, 1000);
	return rxdata; /* �����յ������� */
}
//spiflash���͵�ַ
static void SpiFlashSendAddr(uint32_t address)
{
	SpiReadWriteByte((uint8_t)((address)>>16));     /* ���� bit23 ~ bit16 ��ַ */
	SpiReadWriteByte((uint8_t)((address)>>8));      /* ���� bit15 ~ bit8  ��ַ */
	SpiReadWriteByte((uint8_t)address);             /* ���� bit7  ~ bit0  ��ַ */
}
static void SpiFlashWaitBusy(void)
{
    while ((SpiFlashReadSR(1) & 0x01) == 0x01);   /* �ȴ�BUSYλ��� */
}

uint16_t SpiFlashReadID(void)
{
	uint16_t deviceid;
	HAL_GPIO_WritePin(Flash_CS_GPIO_Port,Flash_CS_Pin,GPIO_PIN_RESET);
	SpiReadWriteByte(0xff);   /* ���Ͷ� ID ���� */
	HAL_GPIO_WritePin(Flash_CS_GPIO_Port,Flash_CS_Pin,GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(Flash_CS_GPIO_Port,Flash_CS_Pin,GPIO_PIN_RESET);
	SpiReadWriteByte(FLASH_ManufactDeviceID);   /* ���Ͷ� ID ���� */
	SpiReadWriteByte(0);    /* д��һ���ֽ� */
	SpiReadWriteByte(0);
	SpiReadWriteByte(0);
	deviceid = SpiReadWriteByte(0xFF) << 8;     /* ��ȡ��8λ�ֽ� */
	deviceid |= SpiReadWriteByte(0xFF);         /* ��ȡ��8λ�ֽ� */
	HAL_GPIO_WritePin(Flash_CS_GPIO_Port,Flash_CS_Pin,GPIO_PIN_SET);
	return deviceid;
}

//SPIFLASH��ȡ����
void SpiFlashRead(uint8_t *pbuf, uint32_t addr, uint16_t len)
{
	uint16_t i;

	HAL_GPIO_WritePin(Flash_CS_GPIO_Port,Flash_CS_Pin,GPIO_PIN_RESET);
	SpiReadWriteByte(FLASH_ReadData);       /* ���Ͷ�ȡ���� */
	SpiFlashSendAddr(addr);                /* ���͵�ַ */

	for(i = 0;i < len; i ++)
	{
			pbuf[i] = SpiReadWriteByte(0XFF);   /* ѭ����ȡ */
	}

	HAL_GPIO_WritePin(Flash_CS_GPIO_Port,Flash_CS_Pin,GPIO_PIN_SET);
}
//���ﱣ֤��Խ�磬�����Ǵ�0��ַ��ʼд��
//����addr��Ҫ��4096����
void SpiFlashWrite(uint8_t *pbuf, uint32_t addr, uint16_t len)
{
	SpiFlashEraseSector(0);//
	uint16_t i;

	SpiFlashWriteEnable();    /* дʹ�� */

	HAL_GPIO_WritePin(Flash_CS_GPIO_Port,Flash_CS_Pin,GPIO_PIN_RESET);
	SpiReadWriteByte(FLASH_PageProgram);    /* ����дҳ���� */
	SpiFlashSendAddr(addr);                /* ���͵�ַ */

	for(i=0;i<len;i++)
	{
		SpiReadWriteByte(pbuf[i]);          /* ѭ��д�� */
	}

	HAL_GPIO_WritePin(Flash_CS_GPIO_Port,Flash_CS_Pin,GPIO_PIN_SET);
	SpiFlashWaitBusy();       /* �ȴ�д����� */
}
//����ѹ��ƫ��ֵ
void SpiFlashSetOffset(void)
{
	uint8_t buf[8];
	uint8_t index;
	uint8_t *p = (uint8_t*)&offsetPre1;
	for(index = 0;index < 4;index ++)
		buf[index] = p[index];
	p = (uint8_t *)&offsetPre2;
	for(index = 0;index < 4;index ++)
		buf[4+index] = p[index];
	SpiFlashWrite(buf,0,8);
}
//��ȡѹ��ƫ��ֵ
void SpiFlashReadOffset(void)
{
	uint8_t buf[8];
	SpiFlashRead(buf,0,8);
	offsetPre1 = *(float*)buf;
	offsetPre2 = *(float*)&buf[4];
	if(fabs(offsetPre1) > 15)
		offsetPre1 = 0;
}

uint8_t SpiFlashReadSR(uint8_t regno)
{
    uint8_t byte = 0, command = 0;

    switch (regno)
    {
        case 1:
            command = FLASH_ReadStatusReg1;  /* ��״̬�Ĵ���1ָ�� */
            break;

        case 2:
            command = FLASH_ReadStatusReg2;  /* ��״̬�Ĵ���2ָ�� */
            break;

        case 3:
            command = FLASH_ReadStatusReg3;  /* ��״̬�Ĵ���3ָ�� */
            break;

        default:
            command = FLASH_ReadStatusReg1;
            break;
    }

    HAL_GPIO_WritePin(Flash_CS_GPIO_Port,Flash_CS_Pin,GPIO_PIN_RESET);
    SpiReadWriteByte(command);      /* ���Ͷ��Ĵ������� */
    byte = SpiReadWriteByte(0Xff);  /* ��ȡһ���ֽ� */
    HAL_GPIO_WritePin(Flash_CS_GPIO_Port,Flash_CS_Pin,GPIO_PIN_SET);
    
    return byte;
}
//дʹ��
void SpiFlashWriteEnable(void)
{
    HAL_GPIO_WritePin(Flash_CS_GPIO_Port,Flash_CS_Pin,GPIO_PIN_RESET);
    SpiReadWriteByte(FLASH_WriteEnable);   /* ����дʹ�� */
    HAL_GPIO_WritePin(Flash_CS_GPIO_Port,Flash_CS_Pin,GPIO_PIN_SET);
}
void SpiFlashEraseSector(uint32_t saddr)
{
    saddr *= 4096;
    SpiFlashWriteEnable();        /* дʹ�� */
    SpiFlashWaitBusy();           /* �ȴ����� */

    HAL_GPIO_WritePin(Flash_CS_GPIO_Port,Flash_CS_Pin,GPIO_PIN_RESET);
    SpiReadWriteByte(FLASH_SectorErase);    /* ����дҳ���� */
    SpiFlashSendAddr(saddr);   /* ���͵�ַ */
    HAL_GPIO_WritePin(Flash_CS_GPIO_Port,Flash_CS_Pin,GPIO_PIN_SET);
    SpiFlashWaitBusy();           /* �ȴ������������ */
}
/* USER CODE END 1 */
