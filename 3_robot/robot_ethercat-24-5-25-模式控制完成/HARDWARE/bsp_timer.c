/*
*********************************************************************************************************
*
*	模块名称 : 定时器模块
*	文件名称 : bsp_timer.c
*	版    本 : V1.0
*	说    明 : 配置systick定时器作为系统滴答定时器。缺省定时周期为100us。
*
*
*
*********************************************************************************************************
*/

#include "bsp_timer.h"
extern __IO uint32_t g_iRunTime;
/*
*********************************************************************************************************
*	函 数 名: bsp_InitTimer
*	功能说明: 配置systick中断，并初始化软件定时器变量
*	形    参:  无
*	返 回 值: 无
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
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//初始化TIM3
	TIM_Cmd(TIM2,ENABLE); //使能定时器3
}
/******************************************************************************
***************************************************************************************
** 函数名称: Delay_us 
** 功能描述: 
** 参    数: 延时微秒
** 返 回 值: None       
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
** 函数名称: Delay_ms
** 功能描述: 
** 参    数: time
** 返 回 值: None       
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