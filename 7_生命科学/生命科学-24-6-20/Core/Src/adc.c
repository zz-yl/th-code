/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */
uint16_t sysAdcValue[ADC_AVERAGE_CHANNELS];//系统ADC数据
#if USE_SHIFT 
static ADC_DATA_GET adcValue[ADC_AVERAGE_CHANNELS];//ADC数据采集数组
#endif
static const uint32_t sysAdcChannel[ADC_AVERAGE_CHANNELS] = {
															 ADC_CHANNEL_10,//第1路PT100
															 ADC_CHANNEL_11,//第2路PT100
															 ADC_CHANNEL_12,//第3路PT100
															 ADC_CHANNEL_13,//第4路PT100
															 ADC_CHANNEL_5,//调速电位器电压
															 ADC_CHANNEL_15//电机转速
															};
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
	#if USE_SHIFT
	uint8_t index = 0;
	#endif
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
	//将所有通道的采样时间全部设置好
  //第1路PT100
  ADC1->SMPR1 |= ADC_SMPR1(ADC_SAMPLETIME_144CYCLES, ADC_CHANNEL_10);
  //第2路PT100
  ADC1->SMPR1 |= ADC_SMPR1(ADC_SAMPLETIME_144CYCLES, ADC_CHANNEL_11);
  //第3路PT100
  ADC1->SMPR1 |= ADC_SMPR1(ADC_SAMPLETIME_144CYCLES, ADC_CHANNEL_12);
  //第4路PT100
  ADC1->SMPR1 |= ADC_SMPR1(ADC_SAMPLETIME_144CYCLES, ADC_CHANNEL_13);
  //调速电位器
  ADC1->SMPR2 |= ADC_SMPR2(ADC_SAMPLETIME_144CYCLES, ADC_CHANNEL_5);
  //电机转速
  ADC1->SMPR1 |= ADC_SMPR1(ADC_SAMPLETIME_144CYCLES, ADC_CHANNEL_15);
  HAL_ADC_Stop(&hadc1);
  //将所有数值清零
	#if USE_SHIFT
  for(index = 0;index < ADC_AVERAGE_CHANNELS;index ++)
  {
		adcValue[index].oldPosition = 0;
		adcValue[index].totalSum = 0;
		adcValue[index].valueAfterFilter = 0;
  }
	#endif
  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PC2     ------> ADC1_IN12
    PC3     ------> ADC1_IN13
    PA5     ------> ADC1_IN5
    PC5     ------> ADC1_IN15
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PC2     ------> ADC1_IN12
    PC3     ------> ADC1_IN13
    PA5     ------> ADC1_IN5
    PC5     ------> ADC1_IN15
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/*
	ADC采样滤波
	*TmpData:用于装滤波后的ADC值
	NewValue:新采样的值
	返回值: 滤波后的值
*/
#if USE_SHIFT 
uint16_t AdcFilterValue(ADC_DATA_GET *tmpData,uint16_t newValue)
{
	tmpData->totalSum += newValue;
	tmpData->totalSum -= tmpData->getValues[tmpData->oldPosition];
	tmpData->getValues[tmpData->oldPosition ++] = newValue;
	tmpData->oldPosition %= ADC_AVERAGE_TIMES;
	return (tmpData->totalSum >> ADC_AVERAGE_SHIFT_BIT);
}
#endif
#if !USE_SHIFT
//AD转化
#define FILTER  0.01
uint16_t filter(uint16_t data,uint16_t old)
{
	data = data * FILTER + (1.0 - FILTER) * old;
	return data;
}
#endif

void AdcConvert(void)
{
	#if !USE_SHIFT
	static uint16_t oldValue[ADC_AVERAGE_CHANNELS];
	#endif
	static uint8_t nowChannel = 0;
	static uint16_t reStartTime = 0;//避免ADC卡死
	uint16_t result;
	uint32_t state = ADC1->SR;
	if((state & ADC_FLAG_EOC))//采集完毕
	{
		reStartTime = 0;
		result = HAL_ADC_GetValue(&hadc1);
		#if !USE_SHIFT
		sysAdcValue[nowChannel] = filter(result,oldValue[nowChannel]);
		oldValue[nowChannel] = sysAdcValue[nowChannel];
		#endif
		#if USE_SHIFT 
		sysAdcValue[nowChannel] = AdcFilterValue(&adcValue[nowChannel],result);
		#endif
		nowChannel ++;
		nowChannel %= ADC_AVERAGE_CHANNELS;
		ADC1->SQR3 = sysAdcChannel[nowChannel];
		ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
	}
	else//TODO
	{
		if(++reStartTime > 1000)//多次进入但是ADC未采集完毕，则重新开始
		{
			HAL_ADC_Stop(&hadc1);
			ADC1->SQR3 = sysAdcChannel[nowChannel];
			ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
			HAL_ADC_Start(&hadc1);
		}
	}
}
#define MAX_SAMPLE 10
uint16_t valueAfter[ADC_AVERAGE_CHANNELS];
static uint16_t valueAll[ADC_AVERAGE_CHANNELS][MAX_SAMPLE];
//在滑差滤波的基础上再增加一级平均值滤波，去掉最大最小值
void ADCFilter(void)
{
	uint16_t min,max;//记录最大最小值
	uint8_t index = 0;
	uint8_t tmp;
	static uint8_t counter = 0;
	static unsigned long sum[ADC_AVERAGE_CHANNELS] = {0};
	for(;index < ADC_AVERAGE_CHANNELS;index++)
	{
		valueAll[index][counter] = sysAdcValue[index];
		sum[index] += sysAdcValue[index];
	}
	counter++;
	if(counter == MAX_SAMPLE)//10次采样求一次平均
	{
		counter = 0;
		for(index = 0;index < ADC_AVERAGE_CHANNELS;index++)
		{
			min = valueAll[index][0];
			max = valueAll[index][0];
			for(tmp = 1;tmp < MAX_SAMPLE;tmp++)
			{
				if(min > valueAll[index][tmp])//取最小值
					min = valueAll[index][tmp];
				if(max < valueAll[index][tmp])//取最大值
					max = valueAll[index][tmp];
			}
			sum[index] = sum[index] - max - min;//减掉最大值最小求平均
			valueAfter[index] = sum[index] / 8;
			sum[index] = 0;
		}
	}	
}
/* USER CODE END 1 */
