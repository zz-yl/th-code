/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "usart.h"
extern TIM_HandleTypeDef htim4;
uint8_t em1Status = 0x00,em0Status = 0x00;//0x00表示松开
RGB_MODE rgbLed = MODE_FLASH_GREEN; //rgb三色灯闪烁状态
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, sysLed_Pin|_485EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, _4852En_Pin|ledRed_Pin|ledBlue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ledGreen_Pin|Flash_CS_Pin|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = sysLed_Pin|_485EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = _4852En_Pin|ledRed_Pin|ledBlue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PB4 */
  GPIO_InitStruct.Pin = ledGreen_Pin|Flash_CS_Pin|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = EM1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EM1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = EM0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EM0_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
void GpioChk(void)
{
	static uint8_t em1Pre,em1Re,em0Pre,em0Re;
//	static 
	uint8_t changeFlag = 0;//0表示没有状态改变
	if(HAL_GPIO_ReadPin(EM1_GPIO_Port,EM1_Pin))//松开
	{
		em1Pre = 0;
		if(em1Status == 0xff)
		{
			if(++em1Re > 2)
			{
				em1Re = 0;
				em1Status = 0x00;
				changeFlag = 0xff;
			}
		}
	}
	else
	{
		em1Re = 0;
		if(em1Status == 0x00)
		{
			if(++em1Pre > 2)
			{
				em1Pre = 0;
				em1Status = 0xff;
				changeFlag = 0xff;
			}
		}
	}
	
	if(HAL_GPIO_ReadPin(EM0_GPIO_Port,EM0_Pin))//松开
	{
		em0Pre = 0;
		if(em0Status == 0xff)
		{
			if(++em0Re > 2)
			{
				em0Re = 0;
				em0Status = 0x00;
				changeFlag = 0xff;
			}
		}
	}
	else
	{
		em0Re = 0;
		if(em0Status == 0x00)
		{
			if(++em0Pre > 2)
			{
				em0Pre = 0;
				em0Status = 0xff;
				changeFlag = 0xff;
			}
		}
	}
	//急停状态有改变
	if(changeFlag)
	{
		//有一个急停按下
		if(em0Status || em1Status)
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
			sysHmiPar.statusFrame -= 4;
			ChangeIconFrame(SCREEN_HOME_PAGE,HOME_ICON_STATUS,sysHmiPar.statusFrame);
			rgbLed = MODE_FLASH_RED;
			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);
			//HAL_TIM_Base_Stop(&htim4);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
			sysHmiPar.statusFrame += 4;
			ChangeIconFrame(SCREEN_HOME_PAGE,HOME_ICON_STATUS,sysHmiPar.statusFrame);
			rgbLed = MODE_FLASH_GREEN;
			//HAL_TIM_Base_Start(&htim4);
			HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
		}
	}
}
//RGB灯,100ms一次
void RGBLedDealt(void)
{
	static uint8_t counter = 0;
	//200ms处理一次
	if(++counter >= 2)
	{
		counter = 0;
		switch(rgbLed)
		{
			case MODE_FLASH_BLUE://闪烁蓝色
			{
				HAL_GPIO_WritePin(ledGreen_GPIO_Port,ledGreen_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ledRed_GPIO_Port,ledRed_Pin,GPIO_PIN_RESET);
				HAL_GPIO_TogglePin(ledBlue_GPIO_Port,ledBlue_Pin);
				break;
			}
			case MODE_FLASH_GREEN://闪烁绿色
			{
				HAL_GPIO_WritePin(ledBlue_GPIO_Port,ledBlue_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ledRed_GPIO_Port,ledRed_Pin,GPIO_PIN_RESET);
				HAL_GPIO_TogglePin(ledGreen_GPIO_Port,ledGreen_Pin);
				break;
			}
			case MODE_FLASH_RED://闪烁红色
			{
				HAL_GPIO_WritePin(ledGreen_GPIO_Port,ledGreen_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ledBlue_GPIO_Port,ledBlue_Pin,GPIO_PIN_RESET);
				HAL_GPIO_TogglePin(ledRed_GPIO_Port,ledRed_Pin);
				break;
			}
			default:break;
		}
	}
}


/* USER CODE END 2 */
