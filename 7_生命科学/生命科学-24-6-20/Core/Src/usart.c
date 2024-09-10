/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "rtc.h"
#include <math.h>
//接收300个字节
#define FLOW_MAX_RECIVE_LENGTH 60
#define IBP_MAX_RECEIVE_LENGTH 1000
static uint8_t txBufDebug[USART_MAX_DATA_PER_FRAME];//至调试串口
static uint8_t txBufHMI[200];//至大彩串口屏
static uint8_t txBufFlow[20];//至超声波流量计
static uint8_t txBufBat[20];//至库仑计
USART_DATA_SEND dataDebug;//串口发送数据缓存
USART_SEND_QUEUE queDebug;//串口发送队列

USART_DATA_SEND dataHMI;//串口发送数据缓存
USART_SEND_QUEUE queHMI;//串口发送队列
//调试串口上一帧发送完成
static uint8_t completeFlagDebug = 0;
//串口屏上一帧发送完成
static uint8_t completeFlagHMI = 0;
//流量气泡传感器上一帧发送完成
static uint8_t completeFlagFlow = 0;
//电池上一帧发送完成
static uint8_t completeFlagBat = 0;
//压力传感器接收的数据
static uint8_t ibpRxBuf[IBP_MAX_RECEIVE_LENGTH];
uint8_t flowRxBuf[FLOW_MAX_RECIVE_LENGTH];//超声波流量计
uint8_t hmiRxBuf[HMI_MEX_RECEIVE_LENGTH];//收到的数据
uint8_t batRxBuf[FLOW_MAX_RECIVE_LENGTH];//电池收到的数据
static uint8_t flowRxCom = 0;//
static uint8_t batRxLen = 0;
static uint8_t batRxCom = 0;//电池收到的数据完成
//ibp握手数据包
static uint8_t handShake[10] = {0xfa,0x0a,0x07,DATA_TYPE_DC,0x01,0x01,0x00,0x00,0x00,0x14};
IBP_BUFFER ibpBuffer;//串口收到的数据缓冲队列
//IBP命令队列
static uint8_t ibpCmdBuf[20] = {0xfa,0x00,0x07};
//读到的压力数据
IBP_PAR sysIbpPar;
//float ibpRead[4];
IBP_OP_QUE ibpCmdQue;
FLOW_TYPE sysFlowPar;//读取到的流量传感器参数
//HMI屏幕命令
HMI_FRAME hmiCmd;
HMI_PAR sysHmiPar;

extern TIM_HandleTypeDef htim7;
uint8_t setFlag = 0;

PSW_DATA pswInput;//


/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}
/* UART5 init function */
void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_ODD;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	//初始化循环缓冲区
	ibpBuffer.length = 0;
	ibpBuffer.readIndex = 0;
	ibpBuffer.writeIndex = 0;
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,ibpRxBuf,IBP_MAX_RECEIVE_LENGTH);
  ibpCmdQue.length = 0;
	ibpCmdQue.readIndex = 0;
	ibpCmdQue.writeIndex = 0;
  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
	hmiCmd.readIndex = 0;
	hmiCmd.writeIndex = 0;
	HAL_UARTEx_ReceiveToIdle_IT(&huart3,hmiRxBuf,HMI_MEX_RECEIVE_LENGTH);
	sysHmiPar.statusFrame = 6;//从6开始，最大7
	sysHmiPar.setSpeed = 0;//默认开机速度0，最大9000
	sysHmiPar.setTemp = 37.0;//开机默认是37.0℃
	sysHmiPar.fastEnable = 0;//默认不快速调节
  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PA0-WKUP     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */
    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* UART5 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspInit 1 */

  /* USER CODE END UART5_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
//  else if(uartHandle->Instance==USART2)
//  {
//  /* USER CODE BEGIN USART2_MspInit 0 */

//  /* USER CODE END USART2_MspInit 0 */
//    /* USART2 clock enable */
//    __HAL_RCC_USART2_CLK_ENABLE();

//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    /**USART2 GPIO Configuration
//    PA2     ------> USART2_TX
//    PA3     ------> USART2_RX
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//    /* USART2 interrupt Init */
//    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(USART2_IRQn);
//  /* USER CODE BEGIN USART2_MspInit 1 */

//  /* USER CODE END USART2_MspInit 1 */
//  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PA0-WKUP     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

    /* UART4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    /* UART5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
//  else if(uartHandle->Instance==USART2)
//  {
//  /* USER CODE BEGIN USART2_MspDeInit 0 */

//  /* USER CODE END USART2_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_USART2_CLK_DISABLE();

//    /**USART2 GPIO Configuration
//    PA2     ------> USART2_TX
//    PA3     ------> USART2_RX
//    */
//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

//    /* USART2 interrupt Deinit */
//    HAL_NVIC_DisableIRQ(USART2_IRQn);
//  /* USER CODE BEGIN USART2_MspDeInit 1 */

//  /* USER CODE END USART2_MspDeInit 1 */
//  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


//串口接收事件,一般是空闲中断
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	//串口2DMA接收IDLE中断
	uint16_t index = 0,tmp;
	//uint8_t txBuf[IBP_MAX_RECEIVE_LENGTH];
	if(USART1 == huart->Instance)
	{
		tmp = ibpBuffer.writeIndex;
		for(; index < Size;index ++)//放至接收数据队列
		{
			ibpBuffer.buf[tmp++] = ibpRxBuf[index];
			if(tmp == IBP_MAX_REC_BUFFER)
				tmp = 0;
		}
		ibpBuffer.writeIndex = tmp;
		ibpBuffer.length += Size;
		//继续接收
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,ibpRxBuf,IBP_MAX_RECEIVE_LENGTH);
		TIM6->CNT = 0;
	}
	else if(UART5 == huart->Instance)
	{
		flowRxCom = 0xff;
	}
	else if(USART3 == huart->Instance)
	{
		tmp = hmiCmd.writeIndex;
		for(index = 0;index < Size;index ++)
			hmiCmd.cmd[tmp].buf[index] = hmiRxBuf[index];
		hmiCmd.cmd[tmp].length = Size;
		tmp ++;
		if(tmp == HMI_MAX_CMD)
			tmp = 0;
		hmiCmd.writeIndex = tmp;//更新队列
		HAL_UARTEx_ReceiveToIdle_IT(&huart3,hmiRxBuf,HMI_MEX_RECEIVE_LENGTH);
	}
//	else if(USART2 == huart->Instance)//串口2，电池容量
//	{
//		batRxCom = 0xff;
//		batRxLen = Size;
//	}
//	sprintf((char*)(txBuf),"rec Data %d\r\n",Size);
//	UsartSetTxBuf(txBuf,strlen((char*)(txBuf)));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	uint16_t value;
	//用于重启串口2DMA接收，只要串口2有接收到数据，就不会跑到这里
	if(htim->Instance == TIM6)
	{
		if(huart1.ErrorCode != HAL_UART_ERROR_NONE)
		{
			value = USART1->SR;
			value = USART1->DR;
			huart1.ErrorCode = HAL_UART_ERROR_NONE;
			HAL_DMA_Abort(huart1.hdmarx);
			ibpBuffer.length = 0;
			ibpBuffer.writeIndex = 0;
			ibpBuffer.readIndex = 0;
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1,ibpRxBuf,IBP_MAX_RECEIVE_LENGTH);
		}
	}
	value = value;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	//发生错误导致中断
	if(huart->Instance == USART1)
	{
		//重启接收
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,ibpRxBuf,IBP_MAX_RECEIVE_LENGTH);
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART4)
	{
		completeFlagDebug = 0;
	}
//	else if(huart->Instance == USART2)
//	{
//		completeFlagBat = 0;
//		HAL_GPIO_WritePin(_4852En_GPIO_Port,_4852En_Pin,GPIO_PIN_RESET);//接收
//		HAL_UARTEx_ReceiveToIdle_IT(&huart2,batRxBuf,FLOW_MAX_RECIVE_LENGTH);
//	}
	else if(huart->Instance == USART3)
	{
		completeFlagHMI = 0;
	}
	else if(huart->Instance == UART5)
	{
		completeFlagFlow = 0;
		HAL_GPIO_WritePin(_485EN_GPIO_Port,_485EN_Pin,GPIO_PIN_RESET);//接收
		HAL_UARTEx_ReceiveToIdle_IT(&huart5,flowRxBuf,FLOW_MAX_RECIVE_LENGTH);
	}
}
//发送串口数据,真正意义上的数据发送
void UsartSendData(uint8_t indexUart)
{
	uint8_t index;
	USART_SEND_QUEUE *que;
	USART_DATA_SEND *data;
	uint8_t *buf;
	if(indexUart == INDEX_DEBUG)
	{
		if(completeFlagDebug)//还没有发送完
			return;
		que = &queDebug;
		data = &dataDebug;
		buf = txBufDebug;
	}
	else
	{
		if(completeFlagHMI)//还没有发送完
			return;
		que = &queHMI;
		data = &dataHMI;
		buf = txBufHMI;
	}
	if(que->readIndex != que->writeIndex)//还有数据没发送完成
	{
		for(index = 0;index < que->bufQueue[que->readIndex].length;index ++)
			buf[index] = data->datSend[(index + que->bufQueue[que->readIndex].readAddr) % USART_MAX_DATA_SEND_LENGTH];
		que->readIndex ++;
		que->readIndex %= USART_SEND_QUEUE_MAX_COUNT;
		if(indexUart == INDEX_DEBUG)
		{
			completeFlagDebug = 1;
			HAL_UART_Transmit_IT(&huart4,buf,index);
		}
		else
		{
			completeFlagHMI = 1;
			HAL_UART_Transmit_IT(&huart3,buf,index);
		}
	}
}
//将要发送的数据填入数组，按顺序等待发送
void UsartSetTxBuf(uint8_t *txBuf,uint8_t length,uint8_t indexUart)
{
	uint8_t index = 0;
	USART_SEND_QUEUE *que;
	USART_DATA_SEND *data;
	if(indexUart == INDEX_DEBUG)
	{
		que = &queDebug;
		data = &dataDebug;
	}
	else
	{
		que = &queHMI;
		data = &dataHMI;
	}
	que->bufQueue[que->writeIndex].readAddr = data->nowIndex;//记录发送位置
	for(index = 0;index < length;index ++)//将数据填入发送缓冲区
		data->datSend[(que->bufQueue[que->writeIndex].readAddr + index) % USART_MAX_DATA_SEND_LENGTH] = txBuf[index];
	que->bufQueue[que->writeIndex].length = index;
	data->nowIndex = (data->nowIndex + index) % USART_MAX_DATA_SEND_LENGTH;
	que->writeIndex = (que->writeIndex + 1) % USART_SEND_QUEUE_MAX_COUNT;
}
//IBP数据累加和校验

uint8_t IBPChkSum(uint8_t *buf)
{
	uint8_t chkSum;
	uint8_t length;
	chkSum = buf[1];
	length = chkSum - 1;
	uint8_t index = 2;
	for(;index < length;index ++)
		chkSum += buf[index];
	return chkSum;
}
//ibp数据解析,2ms一次 
void IBPDataAnalysis(void)
{
	//完整数据帧
	IBP_DATA_FRAME *frame;
	uint8_t debugBuff[100];//调试串口数组
	uint8_t buf[100];//存放接收到的ibp数据
	uint16_t index = ibpBuffer.readIndex;
	uint16_t tmpLength = 0,tmp;
	if(GET_RING_LENGTH(ibpBuffer) < 10)//数据没有一个最小帧
		return;
	//找帧头
	while(ibpBuffer.buf[index] != 0xfa)
	{
		index ++;
		if(index == ibpBuffer.writeIndex)
			return;
		if(index == IBP_MAX_REC_BUFFER)
			index = 0;
		if(GET_RING_LENGTH(ibpBuffer) < 10)//没有最小帧长度
		{
			ibpBuffer.readIndex = index;//去掉非帧头部分
			return;
		}
	}
	if(GET_RING_LENGTH(ibpBuffer) < frame->length)
		return;
	//更新缓冲队列读取地址,readindex = index + length,避免数组越界
	ibpBuffer.readIndex = (index + ibpBuffer.buf[index + 1]) % IBP_MAX_REC_BUFFER;
	//获取命令长度
	tmpLength = ibpBuffer.buf[(index + 1) % IBP_MAX_REC_BUFFER];
	if(tmpLength > 100)//避免数组溢出
		tmpLength = 100;
	//命令拷贝至数组
	for(tmp = 0;tmp < tmpLength;tmp ++)
	{
		buf[tmp] = ibpBuffer.buf[index];
		index ++;
		if(index == IBP_MAX_REC_BUFFER)
			index = 0;
	}
	frame = (IBP_DATA_FRAME *)buf;
	ibpBuffer.length = GET_RING_LENGTH(ibpBuffer);
	if(IBPChkSum(buf) != buf[buf[1] - 1])//ptr[ptr[1] - 1])
	{
		sprintf((char*)debugBuff,"校验失败\r\n");
		UsartSetTxBuf(debugBuff,strlen((char*)debugBuff),INDEX_DEBUG);
		return;
	}
	switch(frame->dataID)//ID就是CMD
	{
		case 0x81://握手
		{
			if(frame->dataType == DATA_TYPE_DD)//请求握手
			{
				sprintf((char*)debugBuff,"上电握手请求\r\n");
				UsartSetTxBuf(debugBuff,strlen((char*)debugBuff),INDEX_DEBUG);
				if(sysIbpPar.ibpStatus == IBP_HAND_SHAKE)
				{
					HAL_UART_Transmit_DMA(&huart1,handShake,10);
				}
			}
			break;
		}
		//模块信息
		case 0x82:
		{
			sprintf((char*)debugBuff,"软件版本V%d.%d.%d\r\n",
																frame->data[0],
																frame->data[1],
																frame->data[2]);
			UsartSetTxBuf(debugBuff,strlen((char*)debugBuff),INDEX_DEBUG);
			sprintf((char*)debugBuff,"算法版本V%d.%d.%d\r\n",
																frame->data[3],
																frame->data[4],
																frame->data[5]);
			UsartSetTxBuf(debugBuff,strlen((char*)debugBuff),INDEX_DEBUG);
			sprintf((char*)debugBuff,"协议版本V%d.%d.%d\r\n",
																frame->data[6],
																frame->data[7],
																frame->data[8]);
			if(frame->data[9] != 0)
			{
				sprintf((char*)debugBuff,"硬件自检失败0x%02x\r\n",frame->data[9]);
				UsartSetTxBuf(debugBuff,strlen((char*)debugBuff),INDEX_DEBUG);
			}
			//
			IBP_QUE_DEL;
			break;
		}
		case 0x80://通用命令ack
		{
			switch(frame->dataSN)
			{
				//没有应答,协议上说会应答
				case 0x0001://握手应答
				{
					sysIbpPar.ibpStatus = IBP_NORMAL;
					sprintf((char*)debugBuff,"握手应答\r\n");
					UsartSetTxBuf(debugBuff,strlen((char*)debugBuff),INDEX_DEBUG);
					break;//
				}
				default:break;
			}
			break;//
		}
		case 0x84://周期IBP数据包 128Hz
		{
			for(index = 0;index < 4;index ++)
			{
				//借用tmpLength变量做存储
				tmpLength = frame->data[1 + 2 * index];//先获取高八位
				tmpLength <<= 8;
				tmpLength += frame->data[2 * index];
				//放大10倍，再加上100为上传值
				sysIbpPar.ibpValue[index] = tmpLength;
				sysIbpPar.ibpValue[index] = sysIbpPar.ibpValue[index] / 10 - 100;
			}
			break;
		}
		case 0x85://IBP计算结果1Hz
		{
//			sprintf((char*)debugBuff,"IBP计算结果\r\n");
//			UsartSetTxBuf(debugBuff,strlen((char*)debugBuff));
			break;
		}
		case 0x94://周期数据1Hz，报告导联状态和硬件信息
		{
//			sprintf((char*)debugBuff,"导联硬件状态:0x%02x\r\n",frame->data[0]);
//			UsartSetTxBuf(debugBuff,strlen((char*)debugBuff));
			sysIbpPar.onOffLine = frame->data[0];
			break;//
		}
		default:
		{		
			sprintf((char*)debugBuff,"其他数据:0x%x\r\n",frame->dataID);
			UsartSetTxBuf(debugBuff,strlen((char*)debugBuff),INDEX_DEBUG);
			break;
		}
	}
}

//50ms一次IBP命令
void IBPCmdSend(void)
{
	uint8_t index;
	//上位机序列号码
	static uint32_t frameCount = 0;//后续可以把序列号加入命令的结构体
																//以便接收判断
	//队列中没有命令
	if(ibpCmdQue.length == 0)
		return;
	index = ibpCmdQue.readIndex;
	switch(ibpCmdQue.opCmd[index].status)
	{
		//等待发送 
		case OP_STATUS_FREE:
		{
			switch(ibpCmdQue.opCmd[index].op)
			{
				//握手
				case IBP_OP_HAND_SHAKE:
				{
					//长度
					ibpCmdBuf[1] = 0x0a;
					//数据包ID
					ibpCmdBuf[3] = DATA_TYPE_DC;
					break;
				}
				case IBP_OP_INFO: //查询模块信息
				{
					//长度
					ibpCmdBuf[1] = 0x0a;
					//数据包ID
					ibpCmdBuf[3] = DATA_TYPE_DR;
					break;
				}
				case IBP_OP_ADJUST_SET_ZERO:  //校0，校准
				{
					//长度
					ibpCmdBuf[1] = 0x0e;
					//数据包ID
					ibpCmdBuf[3] = DATA_TYPE_DC;
					//参数
					ibpCmdBuf[9] = ibpCmdQue.opCmd[index].cmdPar[0];
					ibpCmdBuf[10] = ibpCmdQue.opCmd[index].cmdPar[1];
					ibpCmdBuf[11] = ibpCmdQue.opCmd[index].cmdPar[2];
					ibpCmdBuf[12] = ibpCmdQue.opCmd[index].cmdPar[3];
					break;
				}
				case IBP_OP_SET_HUMAN: //设置病人类型
				{
					//长度
					ibpCmdBuf[1] = 0x0b;
					//数据包ID
					ibpCmdBuf[3] = DATA_TYPE_DC;
					//参数
					ibpCmdBuf[9] = ibpCmdQue.opCmd[index].cmdPar[0];
					break;
				}
				case IBP_OP_SET_TIME ://IBP平均时间设置
				{
					//长度
					ibpCmdBuf[1] = 0x0c;
					//数据包ID
					ibpCmdBuf[3] = DATA_TYPE_DC;
					//参数
					ibpCmdBuf[9] = ibpCmdQue.opCmd[index].cmdPar[0];
					ibpCmdBuf[10] = ibpCmdQue.opCmd[index].cmdPar[1];
					break;
				}
				case IBP_OP_ADJUST_INFO ://校0/校准信息查询
				{
					//长度
					ibpCmdBuf[1] = 0x0c;
					//数据包ID
					ibpCmdBuf[3] = DATA_TYPE_DR;
					//参数
					ibpCmdBuf[9] = ibpCmdQue.opCmd[index].cmdPar[0];
					ibpCmdBuf[10] = ibpCmdQue.opCmd[index].cmdPar[1];
					break;
				}
				case IBP_OP_SET_FILTER ://IBP滤波设置
				{
					//长度
					ibpCmdBuf[1] = 0x0c;
					//数据包ID
					ibpCmdBuf[3] = DATA_TYPE_DC;
					//参数
					ibpCmdBuf[9] = ibpCmdQue.opCmd[index].cmdPar[0];
					ibpCmdBuf[10] = ibpCmdQue.opCmd[index].cmdPar[1];
					break;
				}
				case IBP_OP_SET_MARK ://IBP表明设置
				{
					//长度
					ibpCmdBuf[1] = 0x0c;
					//数据包ID
					ibpCmdBuf[3] = DATA_TYPE_DC;
					//参数
					ibpCmdBuf[9] = ibpCmdQue.opCmd[index].cmdPar[0];
					ibpCmdBuf[10] = ibpCmdQue.opCmd[index].cmdPar[1];
					break;
				}
				case IBP_OP_SET_ADJUST_TIME: //校零校准时间设置
				{
					//长度
					ibpCmdBuf[1] = 0x11;
					//数据包ID
					ibpCmdBuf[3] = DATA_TYPE_DC;
					//参数
					ibpCmdBuf[9] = ibpCmdQue.opCmd[index].cmdPar[0];
					ibpCmdBuf[10] = ibpCmdQue.opCmd[index].cmdPar[1];
					ibpCmdBuf[11] = ibpCmdQue.opCmd[index].cmdPar[2];
					ibpCmdBuf[12] = ibpCmdQue.opCmd[index].cmdPar[3];
					ibpCmdBuf[13] = ibpCmdQue.opCmd[index].cmdPar[4];
					ibpCmdBuf[14] = ibpCmdQue.opCmd[index].cmdPar[5];
					ibpCmdBuf[15] = ibpCmdQue.opCmd[index].cmdPar[6];
					break;
				}
				case IBP_OP_IAP ://在线升级,也不管
				//未识别命令直接忽略
				default:
				{
					index += 1;
					if(index == OP_MAX)
						index = 0;
					ibpCmdQue.readIndex = index;
					ibpCmdQue.length --;
					return;
				}
			}
			//数据包ID
			ibpCmdBuf[4] = ibpCmdQue.opCmd[index].op;
			ibpCmdBuf[5] = frameCount;
			ibpCmdBuf[6] = frameCount >> 8;
			ibpCmdBuf[7] = frameCount >> 16;
			ibpCmdBuf[8] = frameCount >> 24;
			//添加校验
			ibpCmdBuf[ibpCmdBuf[1] - 1] = IBPChkSum(ibpCmdBuf);
			if(HAL_UART_Transmit_DMA(&huart1,ibpCmdBuf,ibpCmdBuf[1]) == HAL_OK)
			{
				//发送成功
				frameCount ++;
				ibpCmdQue.opCmd[index].timer = 0;
				ibpCmdQue.opCmd[index].status = OP_STATUS_WAIT_ACK;
			}
			break;
		}
		//已经发送，等待Ack
		case OP_STATUS_WAIT_ACK:
		{
			//超过2s没有回应，则重发
			if(++ibpCmdQue.opCmd[index].timer > 40)
			{
				ibpCmdQue.opCmd[index].status = OP_STATUS_FREE;
			}
			break;
		}
		default : break;
	}
}
//添加命令
uint8_t IBPCmdAdd(IBP_OP op,uint8_t *par)
{
	IBP_CMD *cmd;
	uint8_t index = 0;
	if(ibpCmdQue.length == OP_MAX)
		return 0;
	cmd = (IBP_CMD*)&ibpCmdQue.opCmd[ibpCmdQue.writeIndex];
	ibpCmdQue.writeIndex ++;
	ibpCmdQue.length ++;
	if(ibpCmdQue.writeIndex == OP_MAX)
		ibpCmdQue.writeIndex = 0;
	cmd->op = op;
	for(index = 0;index < 7;index ++)
		cmd->cmdPar[index] = par[index];
	return 1;
}
//ModBusCRC校验码
uint16_t MakeCrcModbus(uint8_t *str,uint8_t length)
{
	uint16_t crc = 0xffff;
	uint16_t tmp = 0;
	uint8_t i,j;
	for(j = 0 ;j < length ;j++)
	{
		tmp = 0;
		crc ^= str[j];
		for(i = 0;i<8;i++)
		{	
			tmp = (crc&0x01);
			crc >>= 1;	
			if(tmp == 0)
				continue;
			crc ^= 0xA001;		
		}
	}
	return crc;
}

//超声波流量计CRC
uint8_t MakeCrcFlow(uint8_t *buf)
{
	int index,count;
	uint8_t a,b;
	const uint8_t crcSerial = 0xD4;
	b = buf[0];
	count = (buf[1] << 8) + buf[2];
	for(index = 1;index <= (count - 2);index ++)
	{
			a = b ^ buf[index];
			if(a & 0x80)
					b = (a << 1) ^ crcSerial;
			else
					b = (a << 1);
	}
	return (b & 0x3f);
}
//超声波流量传感器获取ID
void FlowGetId(void)
{
	txBufFlow[0] = 0xf1;
	txBufFlow[1] = 0x00;
	txBufFlow[2] = 0x05;
	txBufFlow[3] = 0x25;
	txBufFlow[4] = MakeCrcFlow(txBufFlow);
	HAL_GPIO_WritePin(_485EN_GPIO_Port,_485EN_Pin,GPIO_PIN_SET);
	completeFlagFlow = 1;
	HAL_UART_Transmit_IT(&huart5,txBufFlow,5);
}
//获取测量值
void FlowGetMeasure(void)
{
	txBufFlow[0] = 0xf1;
	//长度
	txBufFlow[1] = 0x00;
	txBufFlow[2] = 14;
	//获取测量值
	txBufFlow[3] = 0x3f;
	txBufFlow[4] = 0x0b;
	txBufFlow[5] = 0x01;
	txBufFlow[6] = 0x04;
	//通道1ul/s
	txBufFlow[7] = 0x81;
	//通道1 ml/min
	txBufFlow[8] = 0x82;
	//通道2ul/s
	txBufFlow[9] = 0x83;
	//通道2 ml/min
	txBufFlow[10] = 0x84;
	//获取温度
	txBufFlow[11] = 0x88;
	//最近一次水泡直径
	txBufFlow[12] = 0x2a;
	txBufFlow[13] = MakeCrcFlow(txBufFlow);
	HAL_GPIO_WritePin(_485EN_GPIO_Port,_485EN_Pin,GPIO_PIN_SET);
	completeFlagFlow = 1;
	HAL_UART_Transmit_IT(&huart5,txBufFlow,14);
}

void BatteryDealt(void)
{
	//20ms执行一次
	uint16_t crc;
	if(batRxCom == 0xff)
	{
		batRxCom = 0;
		crc = batRxBuf[batRxLen - 1];
		crc = (crc << 8) + batRxBuf[batRxLen - 2];
		if(crc == MakeCrcModbus(batRxBuf,batRxLen - 2))
		{
			sysHmiPar.batPercent = batRxBuf[4];
		}
	}
}
void FlowDealt(void)
{
	uint8_t debugBuff[100];
	uint8_t length;
	int serialNum;
	//20ms执行一次
	if(flowRxCom == 0xff)
	{
		length = flowRxBuf[1];
		length = (length << 8) + flowRxBuf[2];
		if(flowRxBuf[length - 1] == MakeCrcFlow(flowRxBuf))
		{
			switch(flowRxBuf[3])
			{
				case 0x25:
				{
					serialNum = flowRxBuf[15];
					serialNum = (serialNum << 8) + flowRxBuf[14];
					serialNum = (serialNum << 8) + flowRxBuf[13];
					serialNum = (serialNum << 8) + flowRxBuf[12];
					sprintf((char*)debugBuff,"传感器系列号:%d\r\n",serialNum);
					break;
				}
				case 0x3f://测量值
				{
					sysFlowPar.ulCh1 = *(float*)&flowRxBuf[8];
					sysFlowPar.mlCh1 = *(float*)&flowRxBuf[13];
					sysFlowPar.ulCh2 = *(float*)&flowRxBuf[18];
					sysFlowPar.mlCh2 = *(float*)&flowRxBuf[23];
					sysFlowPar.temp = *(float*)&flowRxBuf[28];
					sysFlowPar.bubble = flowRxBuf[33];
					sysFlowPar.bubble /= 10;
					sprintf((char*)debugBuff,"流量:%.1fml/min,温度%.1f℃,气泡%.2fmm\r\n",sysFlowPar.mlCh1,
								 sysFlowPar.temp,sysFlowPar.bubble);
					//显示大于0.02的气泡，大于24说明是错误，不显示
					if(sysFlowPar.bubble > 0.02 && sysFlowPar.bubble < 24.0)
					{
						if(!sysFlowPar.counter)//如果数字为0，说明还没显示出气泡状态
						{
							//切换到有气泡那一帧
							ChangeIconFrame(SCREEN_HOME_PAGE,HOME_ICON_BUBBLE,0);
							//显示气泡直径控件
							SetCtrlVisable(SCREEN_HOME_PAGE,HOME_TXT_BUBBLE,1);
							sysFlowPar.counter = BUBBLE_TRIP_SECOND;
						}
						else
							sysFlowPar.counter = BUBBLE_TRIP_SECOND;
						FlowSetBuble();//显示气泡直径
					}
					break;
				}
				default:break;
			}
		}
		else
		{
			sprintf((char*)debugBuff,"校验失败\r\n");
		}
		UsartSetTxBuf(debugBuff,strlen((char*)debugBuff),INDEX_DEBUG);
		flowRxCom = 0;
	}
}
//流量计获取信息
//100ms一次
void FLowGetData(void)
{
	//上一次的处理完了,并且现在没有数据在发送
	if(flowRxCom == 0 && completeFlagFlow != 1)
	{
		FlowGetMeasure();//获取ID信息
	}
}

//获取电池电量百分比
//蓝色是485A 绿色 485B
//靠近库伦计开始看，从左往右 A B Gnd
//void BatteryGetData(void)
//{
//	uint16_t crc;
//	//上一次的处理完了,并且现在没有数据在发送
//	if(batRxCom == 0 && completeFlagBat != 1)
//	{
//		txBufBat[0] = 0x01;
//		txBufBat[1] = 0x04;
//		txBufBat[2] = 0x00;
//		txBufBat[3] = 0x00;
//		txBufBat[4] = 0x00;
//		txBufBat[5] = 0x01;
//		crc = MakeCrcModbus(txBufBat,6);
//		txBufBat[6] = crc;
//		txBufBat[7] = crc >> 8;
//		HAL_GPIO_WritePin(_4852En_GPIO_Port,_4852En_Pin,GPIO_PIN_SET);
//		completeFlagBat = 1;
//		HAL_UART_Transmit_IT(&huart2,txBufBat,8);
//	}
//}

//更改图标内容
void ChangeIconFrame(uint16_t screen,uint16_t id,uint8_t value)
{
	uint8_t txBuf[20];
	txBuf[0] = 0xee;
	txBuf[1] = 0xb1;
	txBuf[2] = 0x23;
	txBuf[3] = screen >> 8;
	txBuf[4] = screen;
	txBuf[5] = id >> 8;
	txBuf[6] = id;
	txBuf[7] = value;
	txBuf[8] = 0xff;
	txBuf[9] = 0xfc;
	txBuf[10] = 0xff;
	txBuf[11] = 0xff;
	UsartSetTxBuf(txBuf,12,INDEX_HMI);
}
//显示隐藏控件
void SetCtrlVisable(uint16_t screen,uint16_t id,uint8_t vis)
{
	uint8_t txBuf[12];
	txBuf[0] = 0xee;
	txBuf[1] = 0xb1;
	txBuf[2] = 0x03;
	txBuf[3] = screen >> 8;
	txBuf[4] = screen;
	txBuf[5] = id >> 8;
	txBuf[6] = id;
	txBuf[7] = vis;
	txBuf[8] = 0xff;
	txBuf[9] = 0xfc;
	txBuf[10] = 0xff;
	txBuf[11] = 0xff;
	UsartSetTxBuf(txBuf,12,INDEX_HMI);
}


//增加历史记录控件，2个曲线
//
void AddHistorySampleDateFloatTwice(uint16_t screen,uint16_t id,uint8_t *val1,uint8_t *val2)
{
	uint8_t txBuf[19];
	txBuf[0] = 0xee;
	txBuf[1] = 0xb1;
	txBuf[2] = 0x60;
	txBuf[3] = screen >> 8;
	txBuf[4] = screen;
	txBuf[5] = id >> 8;
	txBuf[6] = id;
	txBuf[7] = val1[3];
	txBuf[8] = val1[2];
	txBuf[9] = val1[1];
	txBuf[10] = val1[0];
	txBuf[11] = val2[3];
	txBuf[12] = val2[2];
	txBuf[13] = val2[1];
	txBuf[14] = val2[0];
	txBuf[15] = 0xff;
	txBuf[16] = 0xfc;
	txBuf[17] = 0xff;
	txBuf[18] = 0xff;
	UsartSetTxBuf(txBuf,19,INDEX_HMI);
}
//增加历史记录控件点，浮点数
//EE B1 60 00 00 00 01 41 A0 00 00 FF FC FF FF 
void AddHistorySampleDateFloat(uint16_t screen,uint16_t id,uint8_t *val)
{
	uint8_t txBuf[15];
	txBuf[0] = 0xee;
	txBuf[1] = 0xb1;
	txBuf[2] = 0x60;
	txBuf[3] = screen >> 8;
	txBuf[4] = screen;
	txBuf[5] = id >> 8;
	txBuf[6] = id;
	txBuf[7] = val[3];
	txBuf[8] = val[2];
	txBuf[9] = val[1];
	txBuf[10] = val[0];
	txBuf[11] = 0xff;
	txBuf[12] = 0xfc;
	txBuf[13] = 0xff;
	txBuf[14] = 0xff;
	UsartSetTxBuf(txBuf,15,INDEX_HMI);
}
//EE B1 10 00 00 00 01 FF FC FF FF 
//清除文本内容
void ClearTxtContent(uint16_t screen,uint8_t control)
{
	uint8_t txBuf[11];
	txBuf[0] = 0xee;
	txBuf[1] = 0xb1;
	txBuf[2] = 0x10;
	txBuf[3] = screen >> 8;
	txBuf[4] = screen;
	txBuf[5] = control >> 8;
	txBuf[6] = control;
	txBuf[7] = 0xff;
	txBuf[8] = 0xfc;
	txBuf[9] = 0xff;
	txBuf[10] = 0xff;
	UsartSetTxBuf(txBuf,11,INDEX_HMI);
}
//EE B1 00 00 02 FF FC FF FF 切换画面
void ChangeScreen(uint16_t screenId)
{
	uint8_t txBuf[9];
	txBuf[0] = 0xee;
	txBuf[1] = 0xb1;
	txBuf[2] = 0x00;
	txBuf[3] = screenId >> 8;
	txBuf[4] = screenId;
	txBuf[5] = 0xff;
	txBuf[6] = 0xfc;
	txBuf[7] = 0xff;
	txBuf[8] = 0xff;
	UsartSetTxBuf(txBuf,9,INDEX_HMI);
}
//更改曲线显示范围
//EE B1 64 00 00 00 01 00 00 00 64 00 00 00 00 FF FC FF FF 
void ChangeCurveRange(uint16_t screen,uint16_t id,int32_t min,int32_t max)
{
	uint8_t txBuf[19];
	txBuf[0] = 0xee;
	txBuf[1] = 0xb1;
	txBuf[2] = 0x64;
	txBuf[3] = screen >> 8;
	txBuf[4] = screen;
	txBuf[5] = id >> 8;
	txBuf[6] = id;
	txBuf[7] = max >> 24;
	txBuf[8] = max >> 16;
	txBuf[9] = max >> 8;
	txBuf[10] = max;
	txBuf[11] = min >> 24;
	txBuf[12] = min >> 16;
	txBuf[13] = min >> 8;
	txBuf[14] = min;
	txBuf[15] = 0xff;
	txBuf[16] = 0xfc;
	txBuf[17] = 0xff;
	txBuf[18] = 0xff;
	UsartSetTxBuf(txBuf,19,INDEX_HMI);
}
//更改触摸屏的文本值，数值类型
//isfloat代表小数点后的位数,为0表示显示的是整型
void ChangeTxtValue(uint16_t screen,uint16_t id,uint8_t *value,uint8_t isFloat)
{
	uint8_t txBuf[50];
	uint8_t tmp[10];
	float fVal;
	uint8_t len;
	uint8_t index;
	int iVal;
	txBuf[0] = 0xee;
	txBuf[1] = 0xb1;
	txBuf[2] = 0x10;
	txBuf[3] = screen >> 8;
	txBuf[4] = screen;
	txBuf[5] = id >> 8;
	txBuf[6] = id;
	if(isFloat)//浮点数
	{
		fVal = *(float*)value;
		if(isFloat == 1)//后1位
		{
			sprintf((char*)tmp,"%.1f",fVal);
		}
		else//小数点后两位
		{
			sprintf((char*)tmp,"%.2f",fVal);
		}
	}
	else
	{
		iVal = *(int*)value;
		sprintf((char*)tmp,"%d",iVal);
	}
	len = strlen((char*)tmp);
	for(index = 0;index < len;index++)
		txBuf[7 + index] = tmp[index];
	len = index + 7;
	txBuf[len++] = 0xff;
	txBuf[len++] = 0xfc;
	txBuf[len++] = 0xff;
	txBuf[len++] = 0xff;
	UsartSetTxBuf(txBuf,len,INDEX_HMI);
}
//更改触摸屏的文本值，非数值类型
void ChangeTxt(uint16_t screen,uint16_t id,uint8_t *txt,uint8_t len)
{
	uint8_t txBuf[20];
	uint8_t index = 0;
	uint8_t tmp;
	txBuf[0] = 0xee;
	txBuf[1] = 0xb1;
	txBuf[2] = 0x10;
	txBuf[3] = screen >> 8;
	txBuf[4] = screen;
	txBuf[5] = id >> 8;
	txBuf[6] = id;
	tmp = 7;
	for(;index < len;index ++)
	{
		txBuf[tmp++] = txt[index++];
	}
	txBuf[tmp++] = 0xff;
	txBuf[tmp++] = 0xfc;
	txBuf[tmp++] = 0xff;
	txBuf[tmp++] = 0xff;
	UsartSetTxBuf(txBuf,tmp,INDEX_HMI);
}
//设置选择控制选中项
void ChangeSelectorIndex(uint16_t screenId,uint16_t controlId,uint8_t index)
{
	uint8_t txBuf[12];
	txBuf[0] = 0xee;
	txBuf[1] = 0xb1;
	txBuf[2] = 0x10;
	txBuf[3] = screenId >> 8;
	txBuf[4] = screenId;
	txBuf[5] = controlId >> 8;
	txBuf[6] = controlId;
	txBuf[7] = index;
	txBuf[8] = 0xff;
	txBuf[9] = 0xfc;
	txBuf[10] = 0xff;
	txBuf[11] = 0xff;
	UsartSetTxBuf(txBuf,12,INDEX_HMI);
}
//获取RTC时间
void GetRTCFromHMI(void)
{
	uint8_t txBuf[6];
	txBuf[0] = 0xee;
	txBuf[1] = 0x82;
	txBuf[2] = 0xff;
	txBuf[3] = 0xfc;
	txBuf[4] = 0xff;
	txBuf[5] = 0xff;
	UsartSetTxBuf(txBuf,6,INDEX_HMI);
}
uint8_t BIN2BCD(uint8_t number)
{
	uint32_t bcdhigh = 0U;

  while (number >= 10U)
  {
    bcdhigh++;
    number -= 10U;
  }

  return ((uint8_t)(bcdhigh << 4U) | number);
}
void SetRTCToHmi(void)
{
	uint8_t txBuf[13];
	txBuf[0] = 0xee;
	txBuf[1] = 0x81;
	//BCD码格式
	txBuf[2] = BIN2BCD(sysHmiPar.hmiTime.Seconds);//秒
	txBuf[3] = BIN2BCD(sysHmiPar.hmiTime.Minutes);//分
	txBuf[4] = BIN2BCD(sysHmiPar.hmiTime.Hours);//时
	txBuf[5] = BIN2BCD(sysHmiPar.hmiDate.Date);//日
	txBuf[6] = BIN2BCD(sysHmiPar.hmiDate.WeekDay);//星期
	txBuf[7] = BIN2BCD(sysHmiPar.hmiDate.Month);//月
	txBuf[8] = BIN2BCD(sysHmiPar.hmiDate.Year);//年
	txBuf[9] = 0xff;
	txBuf[10] = 0xfc;
	txBuf[11] = 0xff;
	txBuf[12] = 0xff;
	UsartSetTxBuf(txBuf,13,INDEX_HMI);
}

//处理串口屏的数据
void HmiCmdDealt(void)
{
	uint8_t cmdType;// = cmdBuf->cmdType;//获取命令类型
	uint8_t ctrlMsg;// = cmdBuf->ctrlMsg;//获取消息类型
	uint8_t controlType;// = cmdBuf->controlType;
	uint16_t screenId;// = PTR2U16(&cmdBuf->secreenId);//获取页面ID
	uint16_t controlId;// = PTR2U16(&cmdBuf->controlId);//获取控件ID
	uint8_t btnVal;
	uint8_t *date;
	//uint8_t value[4];//倘若是收到文本类消息，则用于保存数值
	uint8_t tmp,len;
	PCTRL_MSG cmd;
	uint8_t tail1,tail2,tail3,tail4;
	static RTC_DateTypeDef dateSet;
	static RTC_TimeTypeDef timSet;
	if(hmiCmd.readIndex != hmiCmd.writeIndex)
	{
		tmp = hmiCmd.readIndex;
		len = hmiCmd.cmd[tmp].length;
		tail1 = hmiCmd.cmd[tmp].buf[len - 4];
		tail2 = hmiCmd.cmd[tmp].buf[len - 3];
		tail3 = hmiCmd.cmd[tmp].buf[len - 2];
		tail4 = hmiCmd.cmd[tmp].buf[len - 1];
		if(hmiCmd.cmd[tmp].buf[0] == 0xee && 
			tail1 == 0xff && tail2 == 0xfc && tail3 == 0xff && tail4 == 0xff)
		{
			cmd = (PCTRL_MSG)hmiCmd.cmd[tmp].buf;
			cmdType = cmd->cmdType;
			ctrlMsg = cmd->ctrlMsg;
			controlType = cmd->controlType;
			screenId = PTR2U16(&cmd->secreenId);;
			controlId = PTR2U16(&cmd->controlId);
			if(cmdType == NOTIFY_RTC)//读取到RTC时钟
			{
				//将读取到的时间记录起来
				date = hmiCmd.cmd[tmp].buf;
				sysHmiPar.hmiDate.Year = date[2];
				sysHmiPar.hmiDate.Month = date[3];
				sysHmiPar.hmiDate.WeekDay = date[4];
				sysHmiPar.hmiDate.Date = date[5];
				sysHmiPar.hmiTime.Hours = date[6];
				sysHmiPar.hmiTime.Minutes = date[7];
				sysHmiPar.hmiTime.Seconds = date[8];
				RTCSetDateAndTime();
			}
			else
			{
				switch(ctrlMsg)
				{
					//控件信息
					case MessageControl:
					{
						//控件
						if(cmdType == NOTIFY_CONTROL)
						{
							switch(controlType)
							{
								//文本控件
								case kCtrlText:
								{
									break;//
								}
								//按键控件
								case kCtrlButton:
								{
									btnVal = cmd->param[1];
									//主页的按键
									if(screenId == SCREEN_HOME_PAGE)
									{
										//根据ID来判断哪个按键
										switch(controlId)
										{
											//刷新压力值
											case HOME_BTN_REFRESH:
											{
												if(btnVal == 0)
												{
													offsetPre1 = sysIbpPar.ibpValue[2];
													offsetPre2 = sysIbpPar.ibpValue[3];
													setFlag = 0xff;
												}
												break;//
											}
											//器官状态的按键
											case HOME_BTN_STATUS:
											{
												//按键松开
												if(btnVal == 0x00)
												{
													sysHmiPar.statusFrame ++;
													if(sysHmiPar.statusFrame == 8)
													{
														sysHmiPar.statusFrame = 4;
													}
													ChangeIconFrame(SCREEN_HOME_PAGE,HOME_ICON_STATUS,
																					sysHmiPar.statusFrame);
												}
												break;
											}
											case HOME_BTN_SPEED_ADD://设置速度增加
											{
												if(btnVal == 0x00)//松开的
												{
													//先关闭定时器
													TIM7->CNT = 0;
													HAL_TIM_Base_Stop(&htim7);
													//还没起开始快速调节
													if(sysHmiPar.fastEnable == 0)
													{
														//只变化1
														ChangeSetSpeed(1,1);
													}
													else
													{
														//关闭快速调节
														sysHmiPar.fastEnable = 0;
													}
												}
												else//按下的
												{
													HAL_TIM_Base_Start(&htim7);
													sysHmiPar.curSetPar = CUR_SET_PAR_ADD_SPEED;
												}
												break;
											}
											case HOME_BTN_SPEED_DEC://设置速度减少
											{
												if(btnVal == 0x00)//松开的
												{
													//先关闭定时器
													TIM7->CNT = 0;
													HAL_TIM_Base_Stop(&htim7);
													//还没起开始快速调节
													if(sysHmiPar.fastEnable == 0)
													{
														//只变化1
														ChangeSetSpeed(0,1);
													}
													else
													{
														//关闭快速调节
														sysHmiPar.fastEnable = 0;
													}
												}
												else//按下的
												{
													HAL_TIM_Base_Start(&htim7);
													sysHmiPar.curSetPar = CUR_SET_PAR_DEC_SPEED;
												}
												break;
											}
											case HOME_BTN_TEMP_ADD://设置温度增加
											{
												if(btnVal == 0x00)//松开的
												{
													//先关闭定时器
													TIM7->CNT = 0;
													HAL_TIM_Base_Stop(&htim7);
													//还没起开始快速调节
													if(sysHmiPar.fastEnable == 0)
													{
														//只变化0.1
														ChangeSetTemp(1,0.1);
													}
													else
													{
														//关闭快速调节
														sysHmiPar.fastEnable = 0;
													}
												}
												else//按下的
												{
													HAL_TIM_Base_Start(&htim7);
													sysHmiPar.curSetPar = CUR_SET_PAR_ADD_TEMP;
												}
												break;
											}
											case HOME_BTN_TEMP_DEC://设置温度减少
											{
												if(btnVal == 0x00)//松开的
												{
													//先关闭定时器
													TIM7->CNT = 0;
													HAL_TIM_Base_Stop(&htim7);
													//还没起开始快速调节
													if(sysHmiPar.fastEnable == 0)
													{
														//只变化0.1
														ChangeSetTemp(0,0.1);
													}
													else
													{
														//关闭快速调节
														sysHmiPar.fastEnable = 0;
													}
												}
												else//按下的
												{
													HAL_TIM_Base_Start(&htim7);
													sysHmiPar.curSetPar = CUR_SET_PAR_DEC_TEMP;
												}
												break;
											}
											default:break;
										}
									}
									else if(screenId == SCREEN_SET_DATE)//设置日期
									{
										//设置日期
										if(controlId == DATE_BTN_SET && btnVal == 0x00)
										{
											RtcSetDate(dateSet);//设置时间
											SetRTCToHmi();
										}
									}
									else if(screenId == SCREEN_SET_TIME)
									{
										//设置时间
										if(controlId == TIME_BTN_SET && btnVal == 0x00)
										{
											RTCSetTime(timSet);//设置日期
											SetRTCToHmi();
										}
									}
									else if(screenId == SCREEN_PSW)//设置密码页面
									{
										//if(btnVal == 0)
										//{
											switch(controlId)//根据不同按键来判断是啥
											{
												case PSW_BTN_0://0
												{
													pswInput.pswBuf[pswInput.inputIndex++] = 0;
													break;
												}
												case PSW_BTN_1://1
												{
													pswInput.pswBuf[pswInput.inputIndex++] = 1;
													break;
												}
												case PSW_BTN_2://2
												{
													pswInput.pswBuf[pswInput.inputIndex++] = 2;
													break;
												}
												case PSW_BTN_3://3
												{
													pswInput.pswBuf[pswInput.inputIndex++] = 3;
													break;
												}
												case PSW_BTN_4://4
												{
													pswInput.pswBuf[pswInput.inputIndex++] = 4;
													break;
												}
												case PSW_BTN_5://5
												{
													pswInput.pswBuf[pswInput.inputIndex++] = 5;
													break;
												}
												case PSW_BTN_6://6
												{
													pswInput.pswBuf[pswInput.inputIndex++] = 6;
													break;
												}
												case PSW_BTN_7://7
												{
													pswInput.pswBuf[pswInput.inputIndex++] = 7;
													break;
												}
												case PSW_BTN_8://8
												{
													pswInput.pswBuf[pswInput.inputIndex++] = 8;
													break;
												}
												case PSW_BTN_9://9
												{
													pswInput.pswBuf[pswInput.inputIndex++] = 9;
													break;
												}
												case PSW_BTN_DEL://删除
												{
													if(pswInput.inputIndex != 0)
														pswInput.inputIndex --;
													break;
												}
												case PSW_BTN_BACK:
												{
													ClearTxtContent(SCREEN_PSW,PSW_TXT_PSW);
													SetCtrlVisable(SCREEN_PSW,PSW_PNG_ERR,0);//隐藏密码错误控件
													break;
												}
												default:break;
											}
											ChangePSWTxt();//更改输入框显示
										//}
									}
									break;
								}
								//选择控件
								case kCtrlSelector:
								{
									switch(controlId)
									{
										case DATE_SELECTOR_DAY://日、秒
										{
											if(screenId == SCREEN_SET_DATE)
											{
												dateSet.Date = cmd->param[0] + 1;
											}
											else
											{
												timSet.Seconds = cmd->param[0];
											}
											break;
										}
										case DATE_SELECTOR_MON://月、分
										{
											if(screenId == SCREEN_SET_DATE)
											{
												dateSet.Month = cmd->param[0] + 1;
											}
											else
											{
												timSet.Minutes = cmd->param[0] + 1;
											}
											break;
										}
										case DATE_SELECTOR_YEAR://年、时
										{
											if(screenId == SCREEN_SET_DATE)
											{
												dateSet.Year = cmd->param[0] + 23;
											}
											else
											{
												timSet.Hours = cmd->param[0];
											}
											break;
										}
									}
									break;
								}
								default:break;
							}
						}
						break;
					}
					//页面信息
					case MessageScreen:
					{
						sysHmiPar.curScreen = screenId;
						switch(screenId)
						{
							case SCREEN_PSW:
							case SCREEN_SET_DATE://设置日期，这里打算将现有的时间、日期更新到滑动控件那里
							{
								dateSet.Year = sysHmiPar.hmiDate.Year;
								dateSet.Month = sysHmiPar.hmiDate.Month;
								dateSet.Date = sysHmiPar.hmiDate.Date;
								ChangeSelectorIndex(SCREEN_SET_DATE,DATE_SELECTOR_YEAR,dateSet.Year - 23);
								ChangeSelectorIndex(SCREEN_SET_DATE,DATE_SELECTOR_MON,dateSet.Month - 1);
								ChangeSelectorIndex(SCREEN_SET_DATE,DATE_SELECTOR_DAY,dateSet.Date - 1);
								pswInput.inputIndex = 0;
								break;
							}
							case SCREEN_SET_TIME://设置时间，这里打算将现有的时间、日期更新到滑动控件那里
							{
								timSet.Hours = sysHmiPar.hmiTime.Hours;
								timSet.Minutes = sysHmiPar.hmiTime.Minutes;
								timSet.Seconds = sysHmiPar.hmiTime.Seconds;
								ChangeSelectorIndex(SCREEN_SET_TIME,TIME_SELECTOR_HOUR,timSet.Hours);
								ChangeSelectorIndex(SCREEN_SET_TIME,TIME_SELECTOR_MIN,timSet.Minutes - 1);
								ChangeSelectorIndex(SCREEN_SET_TIME,TIME_SELECTOR_SEC,timSet.Seconds);
								break;
							}
							default:break;
						}
						break;
					}
					default:break;
				}
			}
		}
		else//不是一个正确的数据帧
		{
			
		}
		tmp ++;
		if(tmp == HMI_MAX_CMD)
			tmp = 0;
		hmiCmd.readIndex = tmp;
	}
}

//更改显示的设置温度值,flag=0减
void ChangeSetTemp(uint8_t flag,float value)
{
	//加
	if(flag)
	{
		sysHmiPar.setTemp += value;
		if(sysHmiPar.setTemp > 40.0)
			sysHmiPar.setTemp = 40.0;
	}
	else//减
	{
		sysHmiPar.setTemp -= value;
		if(sysHmiPar.setTemp < 30.0)
			sysHmiPar.setTemp = 30.0;
	}
	ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_TEMP_SET,(uint8_t*)&sysHmiPar.setTemp,1);
}

//更改显示的设置速度值,flag=0减
void ChangeSetSpeed(uint8_t flag,uint8_t value)
{
	//加
	if(flag)
	{
		sysHmiPar.setSpeed += value;
		if(sysHmiPar.setSpeed > MOTOR_SPEED_MAX)
			sysHmiPar.setSpeed = MOTOR_SPEED_MAX;
	}
	else//减
	{
		sysHmiPar.setSpeed -= value;
		if(sysHmiPar.setSpeed < 0)
			sysHmiPar.setSpeed = 0;
	}
	ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_SET_SPEED,(uint8_t*)&sysHmiPar.setSpeed,0);
}
//快速调整参数检测,50ms一次
void HmiAdjustDected(void)
{
	if(htim7.State != HAL_TIM_STATE_READY)
	{
		//已经出发快速调节
		if(sysHmiPar.fastEnable)
		{
			if(TIM7->CNT > 100)
			{
				TIM7->CNT = 0;
				switch(sysHmiPar.curSetPar)
				{
					case CUR_SET_PAR_ADD_SPEED://增加速度
					{
						ChangeSetSpeed(1,10);
						break;
					}
					case CUR_SET_PAR_ADD_TEMP://增加温度
					{
						ChangeSetTemp(1,0.5);
						break;
					}
					case CUR_SET_PAR_DEC_SPEED://减少速度
					{
						ChangeSetSpeed(0,10);
						break;
					}
					case CUR_SET_PAR_DEC_TEMP://减少温度
					{
						ChangeSetTemp(0,0.5);
						break;
					}
					default:break;
				}
			}
		}
		else//还没触发快速调节
		{
			if(TIM7->CNT > 2000)
			{
				sysHmiPar.fastEnable = 0xff;//触发调节
				TIM7->CNT = 0;
			}
		}
	}
}
//持续工作时间，1s执行一次
void HmiWorkTimerRefresh(void)
{
	uint8_t txBuf[30];
	uint8_t len;
	sysHmiPar.secondWork++;
	if(sysHmiPar.secondWork > 60)
	{
		sysHmiPar.minWork ++;
		sysHmiPar.secondWork = 0;
		if(sysHmiPar.minWork > 60)
		{
			sysHmiPar.hourWork++;
			sysHmiPar.minWork = 0;
		}
	}
	txBuf[0] = 0xee;
	txBuf[1] = 0xb1;
	txBuf[2] = 0x10;
	txBuf[3] = SCREEN_HOME_PAGE >> 8;
	txBuf[4] = SCREEN_HOME_PAGE;
	txBuf[5] = HOME_TXT_TIME >> 8;
	txBuf[6] = HOME_TXT_TIME;
	len = 7;
	if(sysHmiPar.hourWork > 99)
	{
		txBuf[len++] = sysHmiPar.hourWork / 100 + 0x30;
	}
	txBuf[len++] = sysHmiPar.hourWork % 100 / 10 + 0x30;
	txBuf[len++] = sysHmiPar.hourWork % 10 + 0x30;
	txBuf[len++] = ':';
	txBuf[len++] = sysHmiPar.minWork / 10 + 0x30;
	txBuf[len++] = sysHmiPar.minWork % 10 + 0x30;
	txBuf[len++] = ':';
	txBuf[len++] = sysHmiPar.secondWork / 10 + 0x30;
	txBuf[len++] = sysHmiPar.secondWork % 10 + 0x30;
	txBuf[len++] = 0xff;
	txBuf[len++] = 0xfc;
	txBuf[len++] = 0xff;
	txBuf[len++] = 0xff;
	UsartSetTxBuf(txBuf,len,INDEX_HMI);
}
//显示气泡直径
void FlowSetBuble(void)
{
	uint8_t txBuf[30];
	uint8_t len;
	txBuf[0] = 0xee;
	txBuf[1] = 0xb1;
	txBuf[2] = 0x10;
	txBuf[3] = SCREEN_HOME_PAGE >> 8;
	txBuf[4] = SCREEN_HOME_PAGE;
	txBuf[5] = HOME_TXT_BUBBLE >> 8;
	txBuf[6] = HOME_TXT_BUBBLE;
	txBuf[7] = 'D';
	sprintf((char*)&txBuf[7],"%.2f",sysFlowPar.bubble);
	len = strlen((char*)&txBuf[7]);
	len += 8;
	txBuf[len++] = 0xff;
	txBuf[len++] = 0xfc;
	txBuf[len++] = 0xff;
	txBuf[len++] = 0xff;
	UsartSetTxBuf(txBuf,len,INDEX_HMI);
}
//气泡显示时间定时器,1S跑一次
void BubbleTimer(void)
{
	//旧方法，没有闪烁，没有声音提示,1S 执行一次
	/*
	//数字不为0说明时间还没到
	if(sysFlowPar.counter)
	{
		if(--sysFlowPar.counter == 0)//时间到了，切换回正常状态
		{
			ChangeIconFrame(SCREEN_HOME_PAGE,HOME_ICON_BUBBLE,1);
			SetCtrlVisable(SCREEN_HOME_PAGE,HOME_TXT_BUBBLE,0);//隐藏直径控件
		}
	}
	*///旧方法
	//新方法，带闪烁，带声音提示，500ms执行一次
	static uint8_t counter = 0;
	if(sysFlowPar.counter)
	{
		counter ++;
		if(counter % 2 == 0)//1S到了
		{
			//SetCtrlVisable(SCREEN_HOME_PAGE,HOME_ICON_BUBBLE,1);//隐藏图标
			SetCtrlVisable(SCREEN_HOME_PAGE,HOME_TXT_BUBBLE,1);//隐藏直径
			if(rgbLed != MODE_FLASH_RED)
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
			if(--sysFlowPar.counter == 0)
			{
				counter = 0;
				ChangeIconFrame(SCREEN_HOME_PAGE,HOME_ICON_BUBBLE,1);
				SetCtrlVisable(SCREEN_HOME_PAGE,HOME_TXT_BUBBLE,0);//隐藏直径控件
			}
		}
		else
		{
			//SetCtrlVisable(SCREEN_HOME_PAGE,HOME_ICON_BUBBLE,0);//隐藏图标
			SetCtrlVisable(SCREEN_HOME_PAGE,HOME_TXT_BUBBLE,0);//隐藏直径
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
		}
	}
}

void BatteryUpDate(void)
{
	uint8_t conIndex;
	uint8_t txBuf[30];
	uint8_t len;
	if(sysHmiPar.curScreen == SCREEN_PSW)//避免误改
		return;
	//1秒更新一次
	if(sysHmiPar.batPercent < 10)
		conIndex = 5;
	else if(sysHmiPar.batPercent < 25)
		conIndex = 4;
	else if(sysHmiPar.batPercent < 50)
		conIndex = 3;
	else if(sysHmiPar.batPercent < 75)
		conIndex = 2;
	else if(sysHmiPar.batPercent < 99)
		conIndex = 1;
	else 
		conIndex = 0;
	ChangeIconFrame(sysHmiPar.curScreen,HOME_ICON_BATTERY,conIndex);
	txBuf[0] = 0xee;
	txBuf[1] = 0xb1;
	txBuf[2] = 0x10;
	txBuf[3] = sysHmiPar.curScreen >> 8;
	txBuf[4] = sysHmiPar.curScreen;
	txBuf[5] = HOME_TXT_BATTERY >> 8;
	txBuf[6] = HOME_TXT_BATTERY;
	sprintf((char*)&txBuf[7],"%d",sysHmiPar.batPercent);
	len = strlen((char*)&txBuf[7]);
	len += 7;
	txBuf[len++ ] = '%';
	txBuf[len++] = 0xff;
	txBuf[len++] = 0xfc;
	txBuf[len++] = 0xff;
	txBuf[len++] = 0xff;
	UsartSetTxBuf(txBuf,len,INDEX_HMI);
}

#define ADC_SPEED_TAP 600 //300档
//由于选了电位器，这里就分档位了
void AdcSetSpeed(void)
{
	static float  tap;
	float adcTap = 4000.0 / ADC_SPEED_TAP;
	float speedTap = MOTOR_SPEED_MAX * 1.0 / ADC_SPEED_TAP;
	float tapTmp = 0;
	tapTmp = sysAdcValue[4] / adcTap;// * speedIndex;
	if(fabs(tap - tapTmp) > 0.9)
	{
		tap = tapTmp;
		sysHmiPar.setSpeed = tap * speedTap;
		if(sysAdcValue[4] < 50)
			sysHmiPar.setSpeed = 0;
		if(sysHmiPar.setSpeed > MOTOR_SPEED_MAX)
			sysHmiPar.setSpeed = MOTOR_SPEED_MAX;
	}
}
//修改输入密码框
void ChangePSWTxt(void)
{
	//uint8_t txBuf[20];
	uint8_t index;
	//uint8_t tmp;
	uint8_t pswTrue = 1;
	//txBuf[0] = 0xee;
	//txBuf[1] = 0xb1;
	//txBuf[2] = 0x10;
	//txBuf[3] = SCREEN_PSW >> 8;
	//txBuf[4] = SCREEN_PSW;
	//txBuf[5] = PSW_TXT_PSW >> 8;
	//txBuf[6] = PSW_TXT_PSW;
	//tmp = pswInput.inputIndex;
	if(pswInput.inputIndex == 1)
		SetCtrlVisable(SCREEN_PSW,PSW_PNG_ERR,0);//隐藏密码错误图标
	//for(index = 0;index < tmp;index ++)
	//#if PSW_DISPLAY_TRUE //明码
		//txBuf[7 + index] = pswInput.pswBuf[index] + 0x30;
 //	#else //密码保活
		//txBuf[7 + index] = '*';
	//#endif
	//index += 7;
	//txBuf[index ++] = 0xff;
	//txBuf[index ++] = 0xfc;
	//txBuf[index ++] = 0xff;
	//txBuf[index ++] = 0xff;
	//UsartSetTxBuf(txBuf,index,INDEX_HMI);
	if(pswInput.inputIndex == PSW_BUF_LENGTH)//够密码长度了
	{
		for(index = 0;index < PSW_BUF_LENGTH;index++)
		{
			if(pswInput.pswBuf[index] != 0)
			{
				pswTrue = 0;//密码错误
				break;
			}
		}
		pswInput.inputIndex = 0;
		if(pswTrue != 1)//密码错误
		{
			SetCtrlVisable(SCREEN_PSW,PSW_PNG_ERR,1);
			ClearTxtContent(SCREEN_PSW,PSW_TXT_PSW);
		}
		else
		{
			pswInput.inputIndex = 0;
			SetCtrlVisable(SCREEN_PSW,PSW_PNG_ERR,0);
			ClearTxtContent(SCREEN_PSW,PSW_TXT_PSW);
			ChangeScreen(SCREEN_SET_DATE);
			sysHmiPar.curScreen = SCREEN_SET_DATE;
		}
	}
}

//更改屏幕曲线显示范围
void SetScreenCurveRange(void)
{
	static uint8_t counter = 0;
	switch(counter)
	{
		case 0:
		{
			//动脉压
			ChangeCurveRange(SCREEN_CURVE,CURVE_CURVE_DONG,0,120);
			counter = 1;
			break;
		}
		case 1:
		{
			//静脉压
			ChangeCurveRange(SCREEN_CURVE,CURVE_CURVE_JING,0,120);
			counter = 2;
			break;
		}
		case 2:
		{
			//阻力指数
			ChangeCurveRange(SCREEN_CURVE,CURVE_CURVE_RES,0,2);
			counter = 3;
			break;
		}
		case 3:
		{
			ChangeCurveRange(SCREEN_CURVE,CURVE_CURVE_FLOW,-2500,10000);
			counter = 4;
			break;
		}
		default:break;
	}
}

/* USER CODE END 1 */
