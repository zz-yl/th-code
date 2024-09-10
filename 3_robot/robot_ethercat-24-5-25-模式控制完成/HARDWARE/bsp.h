/*
*********************************************************************************************************
*
*	模块名称 : BSP模块
*	文件名称 : bsp.h
*	说    明 : 这是底层驱动模块所有的h文件的汇总文件。 应用程序只需 #include bsp.h 即可，
*			  不需要#include 每个模块的 h 文件
*
*********************************************************************************************************
*/

#ifndef _BSP_H_
#define _BSP_H

#define STM32_V5
//#define STM32_X4

/* 定义 BSP 版本号 */
#define __STM32F1_BSP_VERSION		"1.1"

/* 开关全局中断的宏 */
#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */

/* 这个宏仅用于调试阶段排错 */
//#define BSP_Printf		printf
//#define BSP_Printf(...)

#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_it.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
 
#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif


/* 提供给其他C文件调用的函数 */
void bsp_Init(void);
/* 供外部调用的函数声明 */
void bsp_InitLed(void);
void bsp_LedOn(uint8_t _no);
void bsp_LedOff(uint8_t _no);
void bsp_LedToggle(uint8_t _no);
uint8_t bsp_IsLedOn(uint8_t _no);
uint8_t ST_A1(void);
uint8_t ST_A2(void);
uint8_t Get_Syn_cmd(void);
#endif

/***************************** (END OF FILE) *********************************/
