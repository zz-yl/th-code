/*
*********************************************************************************************************
*
*	ģ������ : BSPģ��
*	�ļ����� : bsp.h
*	˵    �� : ���ǵײ�����ģ�����е�h�ļ��Ļ����ļ��� Ӧ�ó���ֻ�� #include bsp.h ���ɣ�
*			  ����Ҫ#include ÿ��ģ��� h �ļ�
*
*********************************************************************************************************
*/

#ifndef _BSP_H_
#define _BSP_H

#define STM32_V5
//#define STM32_X4

/* ���� BSP �汾�� */
#define __STM32F1_BSP_VERSION		"1.1"

/* ����ȫ���жϵĺ� */
#define ENABLE_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
#define DISABLE_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */

/* ���������ڵ��Խ׶��Ŵ� */
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


/* �ṩ������C�ļ����õĺ��� */
void bsp_Init(void);
/* ���ⲿ���õĺ������� */
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
