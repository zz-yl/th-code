#include "sys.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "LAN8720.h"
#include "usmart.h"
#include "sram.h"
#include "rtc.h"
#include "beep.h"
#include "adc.h"
#include "bsp_timer.h"
#include "temperature.h"
#include "sram.h"
#include "malloc.h"
#include <stdio.h>
#include <string.h>
//ALIENTEK 探索者STM32F407开发板 实验55
//LWIP网络通信综合实验-库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com  
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK

/*ALIENTEK为LWIP学习专门编写手册《ALIENTEK STM32F4 LWIP使用教程.pdf》，详细说明请参考手册。*/

extern void Adc_Temperate_Init(void);	//声明内部温度传感器初始化函数
u16 test;
u16 ded=0;
u32 watch,watch1,watch2;
u8 mydata[60] = {0x01, 0x01, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								 0x00, 0x00, 0x88, 0xA4, 0x0E, 0x10, 0x08, 0x40, 0x00, 0x00,
								 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

u8 MAC_Send(u8 *data, u16 length);
								 
int main(void)
{
	u8 t;
	u8 key;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	uart_init(115200);   	//串口波特率设置
	usmart_dev.init(84); 	//初始化USMART
//	LED_Init();  			//LED初始化
	bsp_InitTimer();	/* 初始化系统滴答定时器 */
//	KEY_Init();  			//按键初始化
//	FSMC_SRAM_Init();		//初始化外部SRAM  
//	BEEP_Init();			//蜂鸣器初始化
//	My_RTC_Init();  		//RTC初始化
//	Adc_Init();  			//ADC初始化 
//	Adc_Temperate_Init(); 	//内部温度传感器初始化
  LAN8720_Init();
	MAC_Init(); 
	while(1)
	{
		MAC_Send(mydata,60);
   if(ETH_CheckFrameReceived()) 	//检测是否收到数据包
	  { 
	   lwip_pkt_handle();
    }  
		ded++;
		watch=bsp_GetTickCount();
		Delay_ms(1000);
	}
}

u8 MAC_Send(u8 *data, u16 length)
{
	u8 res;
 		memcpy((u8*)(DMATxDescToSet->Buffer1Addr), (u8*)data, length);
		/* Prepare transmit descriptors to give to DMA*/
	  ETH_Prepare_Transmit_Descriptors(length);
	  return ETH_SUCCESS;
}







