/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "motor.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//IBP_STATUS ibpStatus;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float temp[4];//4路温度探针
extern uint16_t valueAfter[ADC_AVERAGE_CHANNELS];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t txBuf[100];
	uint8_t index;
	float ri;//阻力指数
	uint16_t flashID = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
//	HAL_Delay(1000);
//	HAL_Delay(1000);
//	HAL_Delay(1000);
//	HAL_Delay(1000);
//	HAL_Delay(1000);
//	HAL_Delay(1000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_IWDG_Init();
//  MX_USART2_UART_Init();
    motor_init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	sprintf((char*)txBuf,"Bulid at %s,%s\r\n",__DATE__,__TIME__);
	UsartSetTxBuf(txBuf,strlen((char*)txBuf),INDEX_DEBUG);
	sysIbpPar.ibpStatus = IBP_HAND_SHAKE;
	ADC1->SQR3 = ADC_CHANNEL_10;//第一组ADC转换
  HAL_ADC_Start(&hadc1);
	TIM4->CCR2 = 0;
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	//获取两次时间
	GetRTCFromHMI();
	//GetRTCFromHMI();
	sysHmiPar.batPercent = 100;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	flashID = SpiFlashReadID();
	sprintf((char*)txBuf,"detected spi flash,ID=0x%x\r\n",flashID);
	UsartSetTxBuf(txBuf,strlen((char*)txBuf),INDEX_DEBUG);
	SpiFlashReadOffset();
	ClearTxtContent(SCREEN_PSW,PSW_TXT_PSW);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		#if USE_SHIFT
		AdcConvert();//ADC转换
		#endif
		if(sysTimeFlag.flag2Ms)//2ms任务
	  {
			//解析IBP血压模块数据
			IBPDataAnalysis();
		  sysTimeFlag.flag2Ms = 0;
	  }
	  if(sysTimeFlag.flag5Ms)//5ms任务
	  {
			#if !USE_SHIFT
			AdcConvert();//ADC转换
			#endif
		  sysTimeFlag.flag5Ms = 0;
	  }
	  if(sysTimeFlag.flag10Ms)//10ms任务
	  {
			//调试串口数据
			UsartSendData(INDEX_DEBUG);
			GpioChk();//主要是急停开关
			HmiCmdDealt();//触摸屏接收命令解析
          motor_run();
		  sysTimeFlag.flag10Ms = 0;
	  }
	  if(sysTimeFlag.flag20Ms)//20ms任务
	  {
			//超声波流量计数据解析
			FlowDealt();
			BatteryDealt();
			UsartSendData(INDEX_HMI);//串口屏数据
		  sysTimeFlag.flag20Ms = 0;
	  }
	  if(sysTimeFlag.flag50Ms)//50ms任务
	  {
			#if _ENABLE_IWDG
			//50ms喂狗一次
			__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
			#endif
			IBPCmdSend();//IBP血压模块操作命令
			HmiAdjustDected();//快速调节判断
          
            motor_data.cmd_speed = (float)sysHmiPar.setSpeed / 10;
            ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_SET_SPEED,(uint8_t*)&sysHmiPar.setSpeed,0);
            sysHmiPar.curSpeed = motor_data.fb_speed * 10;
            ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_SPEED_CUR,(uint8_t*)&sysHmiPar.curSpeed,0);
          
		  sysTimeFlag.flag50Ms = 0;
	  }
	  if(sysTimeFlag.flag100Ms)//100ms任务
	  {
		  sysTimeFlag.flag100Ms = 0;
			ADCFilter();
			FLowGetData();//获取气泡流量传感器数据
			HAL_GPIO_TogglePin(sysLed_GPIO_Port,sysLed_Pin);//闪烁LED
			sysIbpPar.ibpReal[2] = fabs(sysIbpPar.ibpValue[2] - offsetPre1);
			sysIbpPar.ibpReal[3] = fabs(sysIbpPar.ibpValue[3] - offsetPre2);
			//更改显示屏的动静脉压力显示值
			ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_PRE_1,(uint8_t*)&sysIbpPar.ibpReal[2],2);
			ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_PRE_2,(uint8_t*)&sysIbpPar.ibpReal[3],2);
			RGBLedDealt();//RGB三色灯状态改变
//			AdcSetSpeed();//检查调速电位器
	  }
	  if(sysTimeFlag.flag500Ms)//500ms任务
	  {
		  sysTimeFlag.flag500Ms = 0;
			//更新输出速度
//			TIM4->CCR2 = (sysHmiPar.setSpeed / 9000.0 * 1000);
//			ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_SET_SPEED,(uint8_t*)&sysHmiPar.setSpeed,0);
			//更新一次流量
			ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_FLOW1,(uint8_t*)&sysFlowPar.mlCh1,2);
			//更新阻力指数
			if(sysFlowPar.mlCh1 != 0)
				ri = sysIbpPar.ibpReal[2] / sysFlowPar.mlCh1;
			else 
				ri = 0;
			sysIbpPar.ibpRes[2] = ri;// * 20;
			ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_RES_1,(uint8_t*)&ri,2);
			if(sysFlowPar.mlCh1 != 0)
				ri = sysIbpPar.ibpReal[3] / sysFlowPar.mlCh1;
			sysIbpPar.ibpRes[3] = ri;// * 20;
			ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_RES_2,(uint8_t*)&ri,2);
//			BatteryGetData();
			//气泡显示定时器
			BubbleTimer();
	  }
	  if(sysTimeFlag.flag1S)//1s任务
	  {
		  sysTimeFlag.flag1S = 0;
			//更新一次曲线范围
			SetScreenCurveRange();
			//更新一次系统时间
			RtcGetDateTime();
			//1000毫秒打印一次读取值
			sprintf((char*)txBuf,"IBP1: %.2f,IBP2: %.2f,IBP3: %.2f,IBP4: %.2f\r\n",
							sysIbpPar.ibpValue[0],sysIbpPar.ibpValue[1],sysIbpPar.ibpValue[2],sysIbpPar.ibpValue[3]);
			UsartSetTxBuf(txBuf,strlen((char*)(txBuf)),INDEX_DEBUG);
			sprintf((char*)txBuf,"导联硬件状态:0x%02x\r\n",sysIbpPar.onOffLine);
			UsartSetTxBuf(txBuf,strlen((char*)txBuf),INDEX_DEBUG);
			#if IBP_CMD_TEST
			IBPTestCmd();
			#endif
			for(index = 0;index < 4;index ++)
			{
				temp[index] = (216868 + 76.7299 * valueAfter[index]) / 
												 (2891.5662 - 0.03837 * valueAfter[index]);
				//温度 =  (r - 100) / 0.385
				temp[index] = (temp[index] - 100) / 0.385;
			}
			//更新血液温度
			ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_TEMP_BLOOD,(uint8_t*)&temp[0],1);
			//更新采样得到的速度值
//			sysHmiPar.curSpeed = sysAdcValue[5] / 4096.0 * 9000;
//			if(sysHmiPar.setSpeed == 0 || em0Status == 0xff || em1Status == 0xff)
//				sysHmiPar.curSpeed = 0;
//			ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_SPEED_CUR,(uint8_t*)&sysHmiPar.curSpeed,0);
			HmiWorkTimerRefresh();//更新持续灌注时间
			//更新电量图标
			BatteryUpDate();
			
			//历史记录曲线更新.动脉
			AddHistorySampleDateFloat(SCREEN_CURVE,CURVE_CURVE_DONG,
																(uint8_t*)&sysIbpPar.ibpReal[3]);
			//添加一个点到静脉
			AddHistorySampleDateFloat(SCREEN_CURVE,CURVE_CURVE_JING,
																(uint8_t*)&sysIbpPar.ibpReal[2]);
			//添加一个点到流量曲线
			AddHistorySampleDateFloat(SCREEN_CURVE,CURVE_CURVE_FLOW,
																(uint8_t*)&sysFlowPar.mlCh1);
			//曲线添加一个点，阻力指数
			AddHistorySampleDateFloatTwice(SCREEN_CURVE,CURVE_CURVE_RES,
																(uint8_t*)&sysIbpPar.ibpRes[2],	
																(uint8_t*)&sysIbpPar.ibpRes[3]);			
			if(setFlag)
			{
				setFlag = 0;
				//将压力传感器偏移量写入FLASH
				SpiFlashSetOffset();
			}
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
