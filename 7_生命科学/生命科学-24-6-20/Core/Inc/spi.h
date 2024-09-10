/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_SPI2_Init(void);

/* USER CODE BEGIN Prototypes */
	
	/* FLASHÐ¾Æ¬ÁÐ±í */
#define W25Q80      0XEF13          /* W25Q80   Ð¾Æ¬ID */
#define W25Q16      0XEF14          /* W25Q16   Ð¾Æ¬ID */
#define W25Q32      0XEF15          /* W25Q32   Ð¾Æ¬ID */
#define W25Q64      0XEF16          /* W25Q64   Ð¾Æ¬ID */
#define W25Q128     0XEF17          /* W25Q128  Ð¾Æ¬ID */
#define W25Q256     0XEF18          /* W25Q256  Ð¾Æ¬ID */
#define BY25Q64     0X6816          /* BY25Q64  Ð¾Æ¬ID */
#define BY25Q128    0X6817          /* BY25Q128 Ð¾Æ¬ID */
#define NM25Q64     0X5216          /* NM25Q64  Ð¾Æ¬ID */
#define NM25Q128    0X5217          /* NM25Q128 Ð¾Æ¬ID */

/* Ö¸Áî±í */
#define FLASH_WriteEnable           0x06 
#define FLASH_WriteDisable          0x04 
#define FLASH_ReadStatusReg1        0x05 
#define FLASH_ReadStatusReg2        0x35 
#define FLASH_ReadStatusReg3        0x15 
#define FLASH_WriteStatusReg1       0x01 
#define FLASH_WriteStatusReg2       0x31 
#define FLASH_WriteStatusReg3       0x11 
#define FLASH_ReadData              0x03 
#define FLASH_FastReadData          0x0B 
#define FLASH_FastReadDual          0x3B 
#define FLASH_FastReadQuad          0xEB  
#define FLASH_PageProgram           0x02 
#define FLASH_PageProgramQuad       0x32 
#define FLASH_BlockErase            0xD8 
#define FLASH_SectorErase           0x20 
#define FLASH_ChipErase             0xC7 
#define FLASH_PowerDown             0xB9 
#define FLASH_ReleasePowerDown      0xAB 
#define FLASH_DeviceID              0xAB 
#define FLASH_ManufactDeviceID      0x90 
#define FLASH_JedecDeviceID         0x9F 
#define FLASH_Enable4ByteAddr       0xB7
#define FLASH_Exit4ByteAddr         0xE9
#define FLASH_SetReadParam          0xC0 
#define FLASH_EnterQPIMode          0x38
#define FLASH_ExitQPIMode           0xFF

/* ÆÕÍ¨º¯Êý */
void SpiFlashInit(void);                   /* ³õÊ¼»¯25QXX */
uint16_t SpiFlashReadID(void);            /* ¶ÁÈ¡FLASH ID */
void SpiFlashWriteEnable(void);           /* Ð´Ê¹ÄÜ */
uint8_t SpiFlashReadSR(uint8_t regno);    /* ¶ÁÈ¡×´Ì¬¼Ä´æÆ÷ */
void SpiFlashWriteSR(uint8_t regno,uint8_t sr);   /* Ð´×´Ì¬¼Ä´æÆ÷ */

void SpiFlashEraseChip(void);             /* ÕûÆ¬²Á³ý */
void SpiFlashEraseSector(uint32_t addr); /* ÉÈÇø²Á³ý */
void SpiFlashRead(uint8_t *pbuf, uint32_t addr, uint16_t len);     /* ¶ÁÈ¡flash */
void SpiFlashWrite(uint8_t *pbuf, uint32_t addr, uint16_t len);    /* Ð´Èëflash */
void SpiFlashSetOffset(void);
//¶ÁÈ¡Ñ¹Á¦Æ«ÒÆÖµ
void SpiFlashReadOffset(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

