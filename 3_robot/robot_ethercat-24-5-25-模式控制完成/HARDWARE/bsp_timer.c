/*
*********************************************************************************************************
*
*	ģ������ : ��ʱ��ģ��
*	�ļ����� : bsp_timer.c
*	��    �� : V1.0
*	˵    �� : ����systick��ʱ����Ϊϵͳ�δ�ʱ����ȱʡ��ʱ����Ϊ100us��
*
*
*
*********************************************************************************************************
*/

#include "bsp_timer.h"
extern __IO uint32_t g_iRunTime;
/*
*********************************************************************************************************
*	�� �� ��: bsp_InitTimer
*	����˵��: ����systick�жϣ�����ʼ�������ʱ������
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitTimer(void)
{
	SysTick_Config(SystemCoreClock /1000000);
}
uint32_t bsp_GetTickCount(void)
{
	g_iRunTime=TIM_GetCounter(TIM2);
   return g_iRunTime;
}


void TIM2_Int_Init(u32 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///ʹ��TIM3ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��3
}
/******************************************************************************
***************************************************************************************
** ��������: Delay_us 
** ��������: 
** ��    ��: ��ʱ΢��
** �� �� ֵ: None       
****************************************************************************************/
void Delay_us(uint32_t time)
{    
   uint32_t i=0; 
   while(time--)
   {
      i=45;   
      while(i--);    
   }
}
 /***************************************************************************************
** ��������: Delay_ms
** ��������: 
** ��    ��: time
** �� �� ֵ: None       
****************************************************************************************/
void Delay_ms(uint32_t time)
{
   uint32_t i=0;
   while(time--)
   {
      i=48353; 
      while(i--);    
   }
}
void delay(__IO uint32_t nCount)
{
    for (; nCount != 0; nCount--);
}