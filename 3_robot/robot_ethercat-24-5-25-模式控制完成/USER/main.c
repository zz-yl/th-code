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
//ALIENTEK ̽����STM32F407������ ʵ��55
//LWIP����ͨ���ۺ�ʵ��-�⺯���汾
//����֧�֣�www.openedv.com
//�Ա����̣�http://eboard.taobao.com  
//������������ӿƼ����޹�˾  
//���ߣ�����ԭ�� @ALIENTEK

/*ALIENTEKΪLWIPѧϰר�ű�д�ֲᡶALIENTEK STM32F4 LWIPʹ�ý̳�.pdf������ϸ˵����ο��ֲᡣ*/

extern void Adc_Temperate_Init(void);	//�����ڲ��¶ȴ�������ʼ������
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

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	uart_init(115200);   	//���ڲ���������
	usmart_dev.init(84); 	//��ʼ��USMART
//	LED_Init();  			//LED��ʼ��
	bsp_InitTimer();	/* ��ʼ��ϵͳ�δ�ʱ�� */
//	KEY_Init();  			//������ʼ��
//	FSMC_SRAM_Init();		//��ʼ���ⲿSRAM  
//	BEEP_Init();			//��������ʼ��
//	My_RTC_Init();  		//RTC��ʼ��
//	Adc_Init();  			//ADC��ʼ�� 
//	Adc_Temperate_Init(); 	//�ڲ��¶ȴ�������ʼ��
  LAN8720_Init();
	MAC_Init(); 
	while(1)
	{
		MAC_Send(mydata,60);
   if(ETH_CheckFrameReceived()) 	//����Ƿ��յ����ݰ�
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







