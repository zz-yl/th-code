/**@file   bsp_sys.c
* @brief   ϵͳ����
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_sys.h"

/* ��ӹ���ͷ�ļ� (FreeRTOS ��Ҫ�õ�) */
#include "FreeRTOS.h"
#include "task.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/


/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

//#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
//#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
//#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

//#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
//#define TRCENA          0x01000000

//struct __FILE { int handle; /* Add whatever is needed */ };
//FILE __stdout;
//FILE __stdin;

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/


/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

#if (__ARMCC_VERSION >= 6010050)            /* ʹ��AC6������ʱ */
__asm(".global __use_no_semihosting\n\t");  /* ������ʹ�ð�����ģʽ */
__asm(".global __ARM_use_no_argv \n\t");    /* AC6����Ҫ����main����Ϊ�޲�����ʽ�����򲿷����̿��ܳ��ְ�����ģʽ */

#else
/* ʹ��AC5������ʱ, Ҫ�����ﶨ��__FILE �� ��ʹ�ð�����ģʽ */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* ��ʹ�ð�����ģʽ��������Ҫ�ض���_ttywrch\_sys_exit\_sys_command_string����,��ͬʱ����AC6��AC5ģʽ */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* ����_sys_exit()�Ա���ʹ�ð�����ģʽ */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}
/* FILE �� stdio.h���涨�� */
FILE __stdout;

/* �ض���fputc����, printf�������ջ�ͨ������fputc����ַ��������� */
//int fputc(int ch, FILE *f)
//{
//    while ((USART1->ISR & 0X40) == 0);               /* �ȴ���һ���ַ�������� */

//    USART1->RDR = (uint8_t)ch;                       /* ��Ҫ���͵��ַ� ch д�뵽DR�Ĵ��� */
//    return ch;
//}

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
/**
 * @brief       ʱ�����ú���
 */
uint8_t system_clock_config(void)
{
    RCC_OscInitTypeDef rcc_osc_init_struct = {0};
    RCC_ClkInitTypeDef rcc_clk_init_struct = {0};

    /* ʹ��HSE����ѡ��HSE��ΪPLLʱ��Դ������PLL1������USBʱ�� */
    rcc_osc_init_struct.OscillatorType = RCC_OSCILLATORTYPE_HSE;/* ʱ��ԴΪHSE */
    rcc_osc_init_struct.HSEState = RCC_HSE_ON;                  /* ��HSE */
    rcc_osc_init_struct.PLL.PLLState = RCC_PLL_ON;              /* ��PLL */
    rcc_osc_init_struct.PLL.PLLSource = RCC_PLLSOURCE_HSE;      /* PLLʱ��Դѡ��HSE */
    rcc_osc_init_struct.PLL.PLLM = 1;
    rcc_osc_init_struct.PLL.PLLN = 20;
    rcc_osc_init_struct.PLL.PLLP = RCC_PLLP_DIV7;
    rcc_osc_init_struct.PLL.PLLQ = RCC_PLLQ_DIV2;
    rcc_osc_init_struct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&rcc_osc_init_struct) != HAL_OK)
    {
//        Error_Handler();
        return 1;
    }
    /* ѡ��PLL��Ϊϵͳʱ��Դ��������HCLK,PCLK1��PCLK2*/
    rcc_clk_init_struct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

    rcc_clk_init_struct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  /* ����ϵͳʱ��ʱ��ԴΪPLL */
    rcc_clk_init_struct.AHBCLKDivider = RCC_SYSCLK_DIV1;         /* AHB��Ƶϵ�� */
    rcc_clk_init_struct.APB1CLKDivider = RCC_HCLK_DIV1;          /* APB1��Ƶϵ�� */
    rcc_clk_init_struct.APB2CLKDivider = RCC_HCLK_DIV1;          /* APB2��Ƶϵ�� */
    if (HAL_RCC_ClockConfig(&rcc_clk_init_struct, FLASH_LATENCY_4) != HAL_OK)  /* ͬʱ����FLASH��ʱ����Ϊ5WS��Ҳ����6��CPU���� */
    {
//        Error_Handler();
        return 1;
    }

    /** Configure the main internal regulator output voltage
    */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
//        Error_Handler();
        return 1;
    }
    return 0;
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief       ��������ʾ�����ʱ��˺����������������ļ���������
 * @param       file��ָ��Դ�ļ�
 * @param       line��ָ�����ļ��е�����
 * @retval      ��
 */
void assert_failed(uint8_t *file, uint32_t line)
{ 
    while (1)
    {
    }
}
#endif

extern void xPortSysTickHandler(void);

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) /* OS��ʼ����,��ִ�������ĵ��ȴ��� */
    {
        xPortSysTickHandler();
    }
}
