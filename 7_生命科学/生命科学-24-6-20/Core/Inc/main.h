/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define sysLed_Pin GPIO_PIN_13
#define sysLed_GPIO_Port GPIOC
#define _4852En_Pin GPIO_PIN_4
#define _4852En_GPIO_Port GPIOA
#define ledRed_Pin GPIO_PIN_6
#define ledRed_GPIO_Port GPIOA
#define ledBlue_Pin GPIO_PIN_7
#define ledBlue_GPIO_Port GPIOA
#define ledGreen_Pin GPIO_PIN_0
#define ledGreen_GPIO_Port GPIOB
#define Flash_CS_Pin GPIO_PIN_12
#define Flash_CS_GPIO_Port GPIOB
#define EM1_Pin GPIO_PIN_15
#define EM1_GPIO_Port GPIOA
#define EM0_Pin GPIO_PIN_10
#define EM0_GPIO_Port GPIOC
#define _485EN_Pin GPIO_PIN_11
#define _485EN_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define  _ENABLE_IWDG 0 //调试的时候不使能看门狗
//ADC相关
#define USE_SHIFT 1//不使用滑差滤波
#if USE_SHIFT 
#define ADC_AVERAGE_TIMES 64 // ADC采集数据次数
#define ADC_AVERAGE_SHIFT_BIT 6 // 快速求平均值位数
typedef struct
{
	volatile uint32_t totalSum;// 总数
	volatile uint8_t oldPosition;// 替换的位置
	volatile uint16_t getValues[ADC_AVERAGE_TIMES];// ADC获取数值数组 
	volatile uint16_t valueAfterFilter;// 滤波过后的值
}ADC_DATA_GET;//ADC数据采集
#endif
#define ADC_AVERAGE_CHANNELS 6 // ADC转换通道数 -- 6个

#define IBP_MAX_REC_BUFFER 1000 //1000个字节缓冲区
typedef struct 
{
	uint8_t buf[IBP_MAX_REC_BUFFER];//接收缓冲区
	uint16_t writeIndex;//写地址
	uint16_t readIndex;//读地址
	uint16_t length;//缓冲区长度
}IBP_BUFFER;

#define GET_RING_LENGTH(ringBuf) (((ringBuf.writeIndex) >= (ringBuf.readIndex)) ? \
																 ((ringBuf.writeIndex) - (ringBuf.readIndex)) \
																: ((ringBuf.writeIndex) + (IBP_MAX_REC_BUFFER - (ringBuf.readIndex))))
#define USART_MAX_DATA_SEND_LENGTH 1500// 1000个数据发送缓冲区
#define USART_MAX_DATA_PER_FRAME 100//发送数据最大30个字节
#define USART_SEND_QUEUE_MAX_COUNT 50// 最大数据缓存帧
typedef struct
{
	uint8_t datSend[USART_MAX_DATA_SEND_LENGTH];//发送数据缓冲区
	uint16_t nowIndex;//当前写入位置
}USART_DATA_SEND;//串口发送缓冲
typedef struct
{
	uint16_t readAddr;//获取数据其实地址
	uint8_t length;//需要发送的数据长度
}USART_SEND_DATA_FRAME;//发送数据信息

typedef struct
{
	USART_SEND_DATA_FRAME bufQueue[USART_SEND_QUEUE_MAX_COUNT];//发送缓冲队列
	uint8_t readIndex;//出队位置
	uint8_t writeIndex;//入队位置
}USART_SEND_QUEUE;//串口发送队列
#define INDEX_HMI 0
#define INDEX_DEBUG 1
typedef struct
{
	uint8_t flag2Ms;// 2ms时间戳
	uint8_t flag5Ms;// 5ms时间戳
	uint8_t flag10Ms;// 10ms时间戳
	uint8_t flag20Ms;// 20ms时间戳
	uint8_t flag50Ms;// 50ms时间戳
	uint8_t flag100Ms;// 100ms时间戳
	uint8_t flag500Ms;//500ms时间戳
	uint8_t flag1S;// 1s时间戳
}TIM_STAMP_FLAG;//任务运行时间戳
typedef enum
{
	IBP_HAND_SHAKE,//上电默认状态，握手
	IBP_NORMAL,//正常工作状态
}IBP_STATUS;
typedef struct 
{
	float ibpValue[4];
	float ibpReal[4];
	float ibpRes[4];//阻力指数
	uint8_t onOffLine;
	IBP_STATUS ibpStatus;
}IBP_PAR;

#define BUBBLE_TRIP_SECOND 3 //检测到气泡则提示停留3秒钟的时间
typedef struct
{
	float ulCh1;//通道1流量ul/s
	float mlCh1;//通道1流量,mL/min
	float ulCh2;//通道2流量,ul/s
	float mlCh2;//通道2流量,mL/min
	float temp;//温度
	float bubble;//气泡直径
	uint8_t counter;//用于标记显示多久
}FLOW_TYPE;
#define CUR_SET_PAR_ADD_TEMP  0
#define CUR_SET_PAR_ADD_SPEED 1
#define CUR_SET_PAR_DEC_TEMP  2
#define CUR_SET_PAR_DEC_SPEED 3
typedef struct
{
	uint8_t statusFrame;//当前第几帧
	int setSpeed;//设置的速度
	uint8_t curSetPar;//当前在设置的时间还是温度
	float setTemp;//当前设置的温度
	uint8_t fastEnable;//快速调节 
	RTC_DateTypeDef hmiDate;
	RTC_TimeTypeDef hmiTime;
	uint8_t hourWork;//持续工作时间
	uint8_t minWork;//持续工作时间
	uint8_t secondWork;//持续工作时间
	int curSpeed;//当前速度
	uint16_t curScreen;//当前屏幕
	uint8_t batPercent;//电量百分比
}HMI_PAR;

#define MOTOR_SPEED_MAX 3000  //5000


typedef enum
{
	MODE_FLASH_RED,//闪红色
	MODE_FLASH_BLUE,//闪蓝色
	MODE_FLASH_GREEN,//闪绿色
}RGB_MODE;


extern TIM_STAMP_FLAG sysTimeFlag;
//extern IBP_STATUS ibpStatus;
extern IBP_BUFFER ibpBuffer;
extern IBP_PAR sysIbpPar;
extern uint16_t sysAdcValue[ADC_AVERAGE_CHANNELS];//系统ADC数据
extern FLOW_TYPE sysFlowPar;
extern HMI_PAR sysHmiPar;//系统HMI屏幕的参数
extern RGB_MODE rgbLed;//RGB三色灯
extern float offsetPre1,offsetPre2;
extern uint8_t setFlag;//是否写入flash
extern uint8_t em1Status,em0Status;//0x00表示松开
//extern float ibpRead[4];
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
