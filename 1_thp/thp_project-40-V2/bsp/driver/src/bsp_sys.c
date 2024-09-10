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

///* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
//int fputc(int ch, FILE *f)
//{
//    if (DEMCR & TRCENA) 
//    {
//        while (ITM_Port32(0) == 0);
//        ITM_Port8(0) = ch;
//    }
//    return ch;
//}
/**
 * @brief  使能STM32H7的L1-Cache, 同时开启D cache的强制透写
 * @param  无
 * @retval 无
 */
void sys_cache_enable(void)
{
    SCB_EnableICache();     /* 使能I-Cache,函数在core_cm7.h里面定义 */
    SCB_EnableDCache();     /* 使能D-Cache,函数在core_cm7.h里面定义 */
    SCB->CACR |= 1 << 2;    /* 强制D-Cache透写,如不开启透写,实际使用中可能遇到各种问题 */
}

/**
 * @brief       时钟设置函数
 * @param       plln: PLL1倍频系数(PLL倍频), 取值范围: 4~512.
 * @param       pllm: PLL1预分频系数(进PLL之前的分频), 取值范围: 2~63.
 * @param       pllp: PLL1的p分频系数(PLL之后的分频), 分频后作为系统时钟, 取值范围: 2~128.(且必须是2的倍数)
 * @param       pllq: PLL1的q分频系数(PLL之后的分频), 取值范围: 1~128.
 * @note
 *
 *              Fvco: VCO频率
 *              Fsys: 系统时钟频率, 也是PLL1的p分频输出时钟频率
 *              Fq:   PLL1的q分频输出时钟频率
 *              Fs:   PLL输入时钟频率, 可以是HSI, CSI, HSE等.
 *              Fvco = Fs * (plln / pllm);
 *              Fsys = Fvco / pllp = Fs * (plln / (pllm * pllp));
 *              Fq   = Fvco / pllq = Fs * (plln / (pllm * pllq));
 *
 *              外部晶振为25M的时候, 推荐值: plln = 160, pllm = 5, pllp = 2, pllq = 4.
 *              得到:Fvco = 25 * (160 / 5) = 800Mhz
 *                   Fsys = pll1_p_ck = 800 / 2 = 400Mhz
 *                   Fq   = pll1_q_ck = 800 / 4 = 200Mhz
 *
 *              H743默认需要配置的频率如下:
 *              CPU频率(rcc_c_ck) = sys_d1cpre_ck = 400Mhz
 *              rcc_aclk = rcc_hclk3 = 200Mhz
 *              AHB1/2/3/4(rcc_hclk1/2/3/4) = 200Mhz
 *              APB1/2/3/4(rcc_pclk1/2/3/4) = 100Mhz
 *              pll2_p_ck = (25 / 25) * 440 / 2) = 220Mhz
 *              pll2_r_ck = FMC时钟频率 = ((25 / 25) * 440 / 2) = 220Mhz
 *
 * @retval      错误代码: 0, 成功; 1, 错误;
 */
uint8_t system_clock_config(void)
{
    RCC_ClkInitTypeDef rcc_clk_init_struct = {0};
    RCC_OscInitTypeDef rcc_osc_init_struct = {0};
    HAL_StatusTypeDef ret = HAL_OK;

    MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);                         /*使能供电配置更新 */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  /* VOS = 1, Scale1, 1.2V内核电压,FLASH访问可以得到最高性能 */
    while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY);     /* 等待电压稳定 */

    /* 使能HSE，并选择HSE作为PLL时钟源，配置PLL1，开启USB时钟 */
    rcc_osc_init_struct.OscillatorType = RCC_OSCILLATORTYPE_HSE;// | RCC_OSCILLATORTYPE_HSI48;
    rcc_osc_init_struct.HSEState = RCC_HSE_BYPASS;//RCC_HSE_ON;
    rcc_osc_init_struct.HSIState = RCC_HSI_OFF;
    rcc_osc_init_struct.CSIState = RCC_CSI_OFF;
    rcc_osc_init_struct.HSI48State = RCC_HSI48_OFF;//RCC_HSI48_ON;
    rcc_osc_init_struct.PLL.PLLState = RCC_PLL_ON;
    rcc_osc_init_struct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    rcc_osc_init_struct.PLL.PLLN = 192;
    rcc_osc_init_struct.PLL.PLLM = 5;
    rcc_osc_init_struct.PLL.PLLP = 2;
    rcc_osc_init_struct.PLL.PLLQ = 4;
    rcc_osc_init_struct.PLL.PLLR = 2;
    rcc_osc_init_struct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    rcc_osc_init_struct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    rcc_osc_init_struct.PLL.PLLFRACN = 0;
    ret = HAL_RCC_OscConfig(&rcc_osc_init_struct);
    if (ret != HAL_OK)
    {
        return 1;
    }

    /*
     *  选择PLL的输出作为系统时钟
     *  配置RCC_CLOCKTYPE_SYSCLK系统时钟,400M
     *  配置RCC_CLOCKTYPE_HCLK 时钟,200Mhz,对应AHB1，AHB2，AHB3和AHB4总线
     *  配置RCC_CLOCKTYPE_PCLK1时钟,100Mhz,对应APB1总线
     *  配置RCC_CLOCKTYPE_PCLK2时钟,100Mhz,对应APB2总线
     *  配置RCC_CLOCKTYPE_D1PCLK1时钟,100Mhz,对应APB3总线
     *  配置RCC_CLOCKTYPE_D3PCLK1时钟,100Mhz,对应APB4总线
     */
    rcc_clk_init_struct.ClockType = (RCC_CLOCKTYPE_SYSCLK \
                                    | RCC_CLOCKTYPE_HCLK \
                                    | RCC_CLOCKTYPE_PCLK1 \
                                    | RCC_CLOCKTYPE_PCLK2 \
                                    | RCC_CLOCKTYPE_D1PCLK1 \
                                    | RCC_CLOCKTYPE_D3PCLK1);

    rcc_clk_init_struct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clk_init_struct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    rcc_clk_init_struct.AHBCLKDivider = RCC_HCLK_DIV2;
    rcc_clk_init_struct.APB1CLKDivider = RCC_APB1_DIV2; 
    rcc_clk_init_struct.APB2CLKDivider = RCC_APB2_DIV2; 
    rcc_clk_init_struct.APB3CLKDivider = RCC_APB3_DIV2;  
    rcc_clk_init_struct.APB4CLKDivider = RCC_APB4_DIV2; 
    ret = HAL_RCC_ClockConfig(&rcc_clk_init_struct, FLASH_LATENCY_2);
    if (ret != HAL_OK)
    {
        return 1;
    }

    HAL_PWREx_EnableUSBVoltageDetector();   /* 使能USB电压电平检测器 */
    __HAL_RCC_CSI_ENABLE() ;                /* 使能CSI时钟 */
    __HAL_RCC_SYSCFG_CLK_ENABLE() ;         /* 使能SYSCFG时钟 */
    HAL_EnableCompensationCell();           /* 使能IO补偿单元 */
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
