/**@file   bsp_delay.c
* @brief   ��ʱ/�δ�ʱ����ʼ��
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_delay.h"
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#if SYSTEM_SUPPORT_OS  //���SYSTEM_SUPPORT_OS������,˵��Ҫ֧��OS��(������UCOS)
#define delay_osrunning		OSRunning			//OS�Ƿ����б��,0,������;1,������
#define delay_ostickspersec	OSCfg_TickRate_Hz	//OSʱ�ӽ���,��ÿ����ȴ���
#define delay_osintnesting 	OSIntNestingCtr		//�ж�Ƕ�׼���,���ж�Ƕ�״���
#endif
/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

static uint8_t  fac_us = 0;    //us��ʱ������			   
static uint16_t fac_ms = 0;    //ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

#if SYSTEM_SUPPORT_OS
/**
* @brief  us����ʱʱ,�ر��������(��ֹ���us���ӳ�)
* @attention 
*/
void delay_osschedlock(void)
{
    OS_ERR err;
    OSSchedLock(&err);							//UCOSIII�ķ�ʽ,��ֹ���ȣ���ֹ���us��ʱ
}

//us����ʱʱ,�ָ��������
void delay_osschedunlock(void)
{
    OS_ERR err; 
    OSSchedUnlock(&err);						//UCOSIII�ķ�ʽ,�ָ�����
}

//����OS�Դ�����ʱ������ʱ
//ticks:��ʱ�Ľ�����
void delay_ostimedly(u32 ticks)
{
    OS_ERR err; 
    OSTimeDly(ticks,OS_OPT_TIME_PERIODIC,&err);	//UCOSIII��ʱ��������ģʽ
}
 
//systick�жϷ�����,ʹ��ucosʱ�õ�
void SysTick_Handler(void)
{	
    if(delay_osrunning==1)						//OS��ʼ����,��ִ�������ĵ��ȴ���
    {
        OSIntEnter();							//�����ж�
        OSTimeTick();       					//����ucos��ʱ�ӷ������               
        OSIntExit();       	 					//���������л����ж�
    }
}
#endif

//��ʼ���ӳٺ���
//��ʹ��ucos��ʱ��,�˺������ʼ��ucos��ʱ�ӽ���
//SYSTICK��ʱ�ӹ̶�ΪAHBʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��Ƶ��
void DelayInit(void)
{
#if SYSTEM_SUPPORT_OS  						    //�����Ҫ֧��OS.
    uint32_t reload;
#endif
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);  //SYSTICKʹ���ⲿʱ��Դ
    fac_us = SystemCoreClock/1000000/8;           //�����Ƿ�ʹ��OS,fac_us����Ҫʹ��
#if SYSTEM_SUPPORT_OS                           //�����Ҫ֧��OS.
    reload = SystemCoreClock/1000000*(1000000/delay_ostickspersec)/8;  //����delay_ostickspersec�趨���ʱ��
                                                //reloadΪ24λ�Ĵ���,���ֵ:16777216,��72M��,Լ��1.86s����	
    fac_ms = 1000/delay_ostickspersec;            //����OS������ʱ�����ٵ�λ	   

    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;    //����SYSTICK�ж�
    SysTick->LOAD = reload;                       //ÿ1/OS_TICKS_PER_SEC���ж�һ��
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;     //����SYSTICK    

#else
    fac_ms=(u16)fac_us*1000;					//��OS��,����ÿ��ms��Ҫ��systickʱ����   
#endif
}								    

#if SYSTEM_SUPPORT_OS  							//�����Ҫ֧��OS.
//��ʱnus
//nusΪҪ��ʱ��us��.		    								   
//nus:0~204522252(���ֵ��2^32/fac_us@fac_us=21)
void DelayUs(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told,tnow,tcnt = 0;
    uint32_t reload=SysTick->LOAD;              //LOAD��ֵ
    ticks=nus*fac_us; 							//��Ҫ�Ľ�����
    delay_osschedlock();						//��ֹOS���ȣ���ֹ���us��ʱ
    told=SysTick->VAL;        					//�ս���ʱ�ļ�����ֵ
    while(1)
    {
        tnow=SysTick->VAL;
        if(tnow!=told)
        {
            if(tnow<told)tcnt+=told-tnow;		//����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
            else tcnt+=reload-tnow+told;
            told=tnow;
            if(tcnt>=ticks)break;				//ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
        }
    };
    delay_osschedunlock();						//�ָ�OS����								    
}
//��ʱnms
//nms:Ҫ��ʱ��ms��
//nms:0~65535
void DelayMs(uint16_t nms)
{	
    if(delay_osrunning&&delay_osintnesting==0)	//���OS�Ѿ�������,���Ҳ������ж�����(�ж����治���������)	    
    {		 
        if(nms>=fac_ms)							//��ʱ��ʱ�����OS������ʱ������ 
        { 
            delay_ostimedly(nms/fac_ms);		//OS��ʱ
        }
        nms%=fac_ms;							//OS�Ѿ��޷��ṩ��ôС����ʱ��,������ͨ��ʽ��ʱ    
    }
    DelayUs((uint32_t)(nms*1000));              //��ͨ��ʽ��ʱ
}
#else //����ucosʱ
//��ʱnus
//nusΪҪ��ʱ��us��.		    								   
//ע��:nus��ֵ,��Ҫ����798915us(���ֵ��2^24/fac_us@fac_us=21)
void DelayUs(uint32_t nus)
{		
    uint32_t temp;
    SysTick->LOAD=nus*fac_us; 					//ʱ�����	  		 
    SysTick->VAL=0x00;        					//��ռ�����
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;     //��ʼ����
    do
    {
        temp=SysTick->CTRL;
    }while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
    SysTick->VAL =0X00;      					//��ռ�����	 
}
//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
//��168M������,nms<=798ms 
void DelayXMs(uint16_t nms)
{
    uint32_t temp;
    SysTick->LOAD=(u32)nms*fac_ms;				//ʱ�����(SysTick->LOADΪ24bit)
    SysTick->VAL =0x00;							//��ռ�����
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����
    do
    {
        temp=SysTick->CTRL;
    }while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
    SysTick->VAL =0X00;       					//��ռ�����
} 
//��ʱnms 
//nms:0~65535
void DelayMs(uint16_t nms)
{	 	 
	uint8_t repeat=nms/540;						//������540,�ǿ��ǵ�ĳЩ�ͻ����ܳ�Ƶʹ��,
											//���糬Ƶ��248M��ʱ��,delay_xms���ֻ����ʱ541ms������
	uint16_t remain=nms%540;

	while(repeat)
	{
		DelayXMs(540);
		repeat--;
	}
	if(remain)DelayXMs(remain);
} 
#endif 
