/**@file   bsp_sys.c
* @brief   系统配置
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_sys.h"

/* 添加公共头文件 (FreeRTOS 需要用到) */
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

#if (__ARMCC_VERSION >= 6010050)            /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");  /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");    /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}
/* FILE 在 stdio.h里面定义 */
FILE __stdout;

/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
//int fputc(int ch, FILE *f)
//{
//    while ((USART1->ISR & 0X40) == 0);               /* 等待上一个字符发送完成 */

//    USART1->RDR = (uint8_t)ch;                       /* 将要发送的字符 ch 写入到DR寄存器 */
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
 * @brief       时钟设置函数
 */
uint8_t system_clock_config(void)
{
    RCC_OscInitTypeDef rcc_osc_init_struct = {0};
    RCC_ClkInitTypeDef rcc_clk_init_struct = {0};

    /* 使能HSE，并选择HSE作为PLL时钟源，配置PLL1，开启USB时钟 */
    rcc_osc_init_struct.OscillatorType = RCC_OSCILLATORTYPE_HSE;/* 时钟源为HSE */
    rcc_osc_init_struct.HSEState = RCC_HSE_ON;                  /* 打开HSE */
    rcc_osc_init_struct.PLL.PLLState = RCC_PLL_ON;              /* 打开PLL */
    rcc_osc_init_struct.PLL.PLLSource = RCC_PLLSOURCE_HSE;      /* PLL时钟源选择HSE */
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
    /* 选中PLL作为系统时钟源并且配置HCLK,PCLK1和PCLK2*/
    rcc_clk_init_struct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

    rcc_clk_init_struct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  /* 设置系统时钟时钟源为PLL */
    rcc_clk_init_struct.AHBCLKDivider = RCC_SYSCLK_DIV1;         /* AHB分频系数 */
    rcc_clk_init_struct.APB1CLKDivider = RCC_HCLK_DIV1;          /* APB1分频系数 */
    rcc_clk_init_struct.APB2CLKDivider = RCC_HCLK_DIV1;          /* APB2分频系数 */
    if (HAL_RCC_ClockConfig(&rcc_clk_init_struct, FLASH_LATENCY_4) != HAL_OK)  /* 同时设置FLASH延时周期为5WS，也就是6个CPU周期 */
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
 * @brief       当编译提示出错的时候此函数用来报告错误的文件和所在行
 * @param       file：指向源文件
 * @param       line：指向在文件中的行数
 * @retval      无
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
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) /* OS开始跑了,才执行正常的调度处理 */
    {
        xPortSysTickHandler();
    }
}
