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
float temp[4];//4·�¶�̽��
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
	float ri;//����ָ��
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
	ADC1->SQR3 = ADC_CHANNEL_10;//��һ��ADCת��
  HAL_ADC_Start(&hadc1);
	TIM4->CCR2 = 0;
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	//��ȡ����ʱ��
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
		AdcConvert();//ADCת��
		#endif
		if(sysTimeFlag.flag2Ms)//2ms����
	  {
			//����IBPѪѹģ������
			IBPDataAnalysis();
		  sysTimeFlag.flag2Ms = 0;
	  }
	  if(sysTimeFlag.flag5Ms)//5ms����
	  {
			#if !USE_SHIFT
			AdcConvert();//ADCת��
			#endif
		  sysTimeFlag.flag5Ms = 0;
	  }
	  if(sysTimeFlag.flag10Ms)//10ms����
	  {
			//���Դ�������
			UsartSendData(INDEX_DEBUG);
			GpioChk();//��Ҫ�Ǽ�ͣ����
			HmiCmdDealt();//�����������������
          motor_run();
		  sysTimeFlag.flag10Ms = 0;
	  }
	  if(sysTimeFlag.flag20Ms)//20ms����
	  {
			//���������������ݽ���
			FlowDealt();
			BatteryDealt();
			UsartSendData(INDEX_HMI);//����������
		  sysTimeFlag.flag20Ms = 0;
	  }
	  if(sysTimeFlag.flag50Ms)//50ms����
	  {
			#if _ENABLE_IWDG
			//50msι��һ��
			__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
			#endif
			IBPCmdSend();//IBPѪѹģ���������
			HmiAdjustDected();//���ٵ����ж�
          
            motor_data.cmd_speed = (float)sysHmiPar.setSpeed / 10;
            ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_SET_SPEED,(uint8_t*)&sysHmiPar.setSpeed,0);
            sysHmiPar.curSpeed = motor_data.fb_speed * 10;
            ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_SPEED_CUR,(uint8_t*)&sysHmiPar.curSpeed,0);
          
		  sysTimeFlag.flag50Ms = 0;
	  }
	  if(sysTimeFlag.flag100Ms)//100ms����
	  {
		  sysTimeFlag.flag100Ms = 0;
			ADCFilter();
			FLowGetData();//��ȡ������������������
			HAL_GPIO_TogglePin(sysLed_GPIO_Port,sysLed_Pin);//��˸LED
			sysIbpPar.ibpReal[2] = fabs(sysIbpPar.ibpValue[2] - offsetPre1);
			sysIbpPar.ibpReal[3] = fabs(sysIbpPar.ibpValue[3] - offsetPre2);
			//������ʾ���Ķ�����ѹ����ʾֵ
			ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_PRE_1,(uint8_t*)&sysIbpPar.ibpReal[2],2);
			ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_PRE_2,(uint8_t*)&sysIbpPar.ibpReal[3],2);
			RGBLedDealt();//RGB��ɫ��״̬�ı�
//			AdcSetSpeed();//�����ٵ�λ��
	  }
	  if(sysTimeFlag.flag500Ms)//500ms����
	  {
		  sysTimeFlag.flag500Ms = 0;
			//��������ٶ�
//			TIM4->CCR2 = (sysHmiPar.setSpeed / 9000.0 * 1000);
//			ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_SET_SPEED,(uint8_t*)&sysHmiPar.setSpeed,0);
			//����һ������
			ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_FLOW1,(uint8_t*)&sysFlowPar.mlCh1,2);
			//��������ָ��
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
			//������ʾ��ʱ��
			BubbleTimer();
	  }
	  if(sysTimeFlag.flag1S)//1s����
	  {
		  sysTimeFlag.flag1S = 0;
			//����һ�����߷�Χ
			SetScreenCurveRange();
			//����һ��ϵͳʱ��
			RtcGetDateTime();
			//1000�����ӡһ�ζ�ȡֵ
			sprintf((char*)txBuf,"IBP1: %.2f,IBP2: %.2f,IBP3: %.2f,IBP4: %.2f\r\n",
							sysIbpPar.ibpValue[0],sysIbpPar.ibpValue[1],sysIbpPar.ibpValue[2],sysIbpPar.ibpValue[3]);
			UsartSetTxBuf(txBuf,strlen((char*)(txBuf)),INDEX_DEBUG);
			sprintf((char*)txBuf,"����Ӳ��״̬:0x%02x\r\n",sysIbpPar.onOffLine);
			UsartSetTxBuf(txBuf,strlen((char*)txBuf),INDEX_DEBUG);
			#if IBP_CMD_TEST
			IBPTestCmd();
			#endif
			for(index = 0;index < 4;index ++)
			{
				temp[index] = (216868 + 76.7299 * valueAfter[index]) / 
												 (2891.5662 - 0.03837 * valueAfter[index]);
				//�¶� =  (r - 100) / 0.385
				temp[index] = (temp[index] - 100) / 0.385;
			}
			//����ѪҺ�¶�
			ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_TEMP_BLOOD,(uint8_t*)&temp[0],1);
			//���²����õ����ٶ�ֵ
//			sysHmiPar.curSpeed = sysAdcValue[5] / 4096.0 * 9000;
//			if(sysHmiPar.setSpeed == 0 || em0Status == 0xff || em1Status == 0xff)
//				sysHmiPar.curSpeed = 0;
//			ChangeTxtValue(SCREEN_HOME_PAGE,HOME_TXT_SPEED_CUR,(uint8_t*)&sysHmiPar.curSpeed,0);
			HmiWorkTimerRefresh();//���³�����עʱ��
			//���µ���ͼ��
			BatteryUpDate();
			
			//��ʷ��¼���߸���.����
			AddHistorySampleDateFloat(SCREEN_CURVE,CURVE_CURVE_DONG,
																(uint8_t*)&sysIbpPar.ibpReal[3]);
			//���һ���㵽����
			AddHistorySampleDateFloat(SCREEN_CURVE,CURVE_CURVE_JING,
																(uint8_t*)&sysIbpPar.ibpReal[2]);
			//���һ���㵽��������
			AddHistorySampleDateFloat(SCREEN_CURVE,CURVE_CURVE_FLOW,
																(uint8_t*)&sysFlowPar.mlCh1);
			//�������һ���㣬����ָ��
			AddHistorySampleDateFloatTwice(SCREEN_CURVE,CURVE_CURVE_RES,
																(uint8_t*)&sysIbpPar.ibpRes[2],	
																(uint8_t*)&sysIbpPar.ibpRes[3]);			
			if(setFlag)
			{
				setFlag = 0;
				//��ѹ��������ƫ����д��FLASH
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
