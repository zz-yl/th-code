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

///* �ض���fputc����, printf�������ջ�ͨ������fputc����ַ��������� */
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
 * @brief  ʹ��STM32H7��L1-Cache, ͬʱ����D cache��ǿ��͸д
 * @param  ��
 * @retval ��
 */
void sys_cache_enable(void)
{
    SCB_EnableICache();     /* ʹ��I-Cache,������core_cm7.h���涨�� */
    SCB_EnableDCache();     /* ʹ��D-Cache,������core_cm7.h���涨�� */
    SCB->CACR |= 1 << 2;    /* ǿ��D-Cache͸д,�粻����͸д,ʵ��ʹ���п��������������� */
}

/**
 * @brief       ʱ�����ú���
 * @param       plln: PLL1��Ƶϵ��(PLL��Ƶ), ȡֵ��Χ: 4~512.
 * @param       pllm: PLL1Ԥ��Ƶϵ��(��PLL֮ǰ�ķ�Ƶ), ȡֵ��Χ: 2~63.
 * @param       pllp: PLL1��p��Ƶϵ��(PLL֮��ķ�Ƶ), ��Ƶ����Ϊϵͳʱ��, ȡֵ��Χ: 2~128.(�ұ�����2�ı���)
 * @param       pllq: PLL1��q��Ƶϵ��(PLL֮��ķ�Ƶ), ȡֵ��Χ: 1~128.
 * @note
 *
 *              Fvco: VCOƵ��
 *              Fsys: ϵͳʱ��Ƶ��, Ҳ��PLL1��p��Ƶ���ʱ��Ƶ��
 *              Fq:   PLL1��q��Ƶ���ʱ��Ƶ��
 *              Fs:   PLL����ʱ��Ƶ��, ������HSI, CSI, HSE��.
 *              Fvco = Fs * (plln / pllm);
 *              Fsys = Fvco / pllp = Fs * (plln / (pllm * pllp));
 *              Fq   = Fvco / pllq = Fs * (plln / (pllm * pllq));
 *
 *              �ⲿ����Ϊ25M��ʱ��, �Ƽ�ֵ: plln = 160, pllm = 5, pllp = 2, pllq = 4.
 *              �õ�:Fvco = 25 * (160 / 5) = 800Mhz
 *                   Fsys = pll1_p_ck = 800 / 2 = 400Mhz
 *                   Fq   = pll1_q_ck = 800 / 4 = 200Mhz
 *
 *              H743Ĭ����Ҫ���õ�Ƶ������:
 *              CPUƵ��(rcc_c_ck) = sys_d1cpre_ck = 400Mhz
 *              rcc_aclk = rcc_hclk3 = 200Mhz
 *              AHB1/2/3/4(rcc_hclk1/2/3/4) = 200Mhz
 *              APB1/2/3/4(rcc_pclk1/2/3/4) = 100Mhz
 *              pll2_p_ck = (25 / 25) * 440 / 2) = 220Mhz
 *              pll2_r_ck = FMCʱ��Ƶ�� = ((25 / 25) * 440 / 2) = 220Mhz
 *
 * @retval      �������: 0, �ɹ�; 1, ����;
 */
uint8_t system_clock_config(void)
{
    RCC_ClkInitTypeDef rcc_clk_init_struct = {0};
    RCC_OscInitTypeDef rcc_osc_init_struct = {0};
    HAL_StatusTypeDef ret = HAL_OK;

    MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);                         /*ʹ�ܹ������ø��� */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  /* VOS = 1, Scale1, 1.2V�ں˵�ѹ,FLASH���ʿ��Եõ�������� */
    while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY);     /* �ȴ���ѹ�ȶ� */

    /* ʹ��HSE����ѡ��HSE��ΪPLLʱ��Դ������PLL1������USBʱ�� */
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
     *  ѡ��PLL�������Ϊϵͳʱ��
     *  ����RCC_CLOCKTYPE_SYSCLKϵͳʱ��,400M
     *  ����RCC_CLOCKTYPE_HCLK ʱ��,200Mhz,��ӦAHB1��AHB2��AHB3��AHB4����
     *  ����RCC_CLOCKTYPE_PCLK1ʱ��,100Mhz,��ӦAPB1����
     *  ����RCC_CLOCKTYPE_PCLK2ʱ��,100Mhz,��ӦAPB2����
     *  ����RCC_CLOCKTYPE_D1PCLK1ʱ��,100Mhz,��ӦAPB3����
     *  ����RCC_CLOCKTYPE_D3PCLK1ʱ��,100Mhz,��ӦAPB4����
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

    HAL_PWREx_EnableUSBVoltageDetector();   /* ʹ��USB��ѹ��ƽ����� */
    __HAL_RCC_CSI_ENABLE() ;                /* ʹ��CSIʱ�� */
    __HAL_RCC_SYSCFG_CLK_ENABLE() ;         /* ʹ��SYSCFGʱ�� */
    HAL_EnableCompensationCell();           /* ʹ��IO������Ԫ */
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
