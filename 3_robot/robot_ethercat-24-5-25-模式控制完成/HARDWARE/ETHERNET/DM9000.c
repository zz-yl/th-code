/*
*********************************************************************************************************
*
*	ģ������ : DM9000AEP �ײ�����ģ��(For STM32F4XX�� uip)
*	�ļ����� : dm9k_uip.c
*	��    �� : V1.1
*	˵    �� : ����Ӳ���ײ�������������ļ���ÿ��c�ļ����� #include "bsp.h" ���������е���������ģ�顣
*			   bsp = Borad surport packet �弶֧�ְ�
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2013-03-01  armfly   ��ʽ����
*		V1.1    2013-06-20  armfly   �淶ע�ͣ���ӱ�Ҫ˵��
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/
#include "stm32f4xx.h"
#include "bsp_timer.h"
#include "DM9000.h"
#include "bsp.h"

//#define DM9000A_FLOW_CONTROL
//#define DM9000A_UPTO_100M
//#define Fifo_Point_Check
//#define Point_Error_Reset
#define Fix_Note_Address
#define FifoPointCheck

/* DM9000A ���պ������ú� */
#define Rx_Int_enable
#define Max_Int_Count			1
#define Max_Ethernet_Lenth		1536
#define Broadcast_Jump
#define Max_Broadcast_Lenth		500

/* DM9000A ���ͺ������ú� */
#define Max_Send_Pack			2

#define NET_BASE_ADDR		0x68400000
#define NET_REG_ADDR		(*((volatile uint16_t *) NET_BASE_ADDR))
#define NET_REG_DATA		(*((volatile uint16_t *) (NET_BASE_ADDR + 0x00080000)))

#define ETH_ADDR_LEN			6

/* ���������� MAC ��ַ */
static unsigned char DEF_MAC_ADDR[ETH_ADDR_LEN] = {0x01, 0x02, 0x01, 0x02, 0x01, 0x02};
uint8_t SendPackOk = 0;
uint8_t s_FSMC_Init_Ok = 0;	/* ����ָʾFSMC�Ƿ��ʼ�� */

u16  receiveLen_DM9000;
u8   receiveBuffer_DM9000[200];
u16  receiveLen;
u8   receiveBuffer[200];

//#define printk(...)
//#define printk printf

static void DM9K_CtrlLinesConfig(void);
static void DM9K_FSMCConfig(void);
void DM9000_Initnic(void);

/*
*********************************************************************************************************
*	�� �� ��: DM9000_Init
*	����˵��: uIP �ӿں���,��ʼ������.  uIP �ӿں���.
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void DM9000_Init(void)
{

			DM9K_CtrlLinesConfig();
			DM9K_FSMCConfig();
	    s_FSMC_Init_Ok = 1;

	DM9000_Initnic();			/* ����DM9000 */
}

/*
*********************************************************************************************************
*	�� �� ��: DM9K_CtrlLinesConfig
*	����˵��: ����DM9000AE���ƿ��ߣ�FSMC�ܽ�����Ϊ���ù��� (For SMT32F4)
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void DM9K_CtrlLinesConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
	/* ʹ��FSMCʱ�� */
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

	/* ʹ�� GPIOʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOG, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	/* ���� PD.00(D2), PD.01(D3), PD.04(NOE), PD.05(NWE), PD.08(D13), PD.09(D14),
	 PD.10(D15), PD.14(D0), PD.15(D1) Ϊ����������� */

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
	                            GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 |
	                            GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* ���� PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
	 PE.14(D11), PE.15(D12) Ϊ����������� */

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5 , GPIO_AF_FSMC);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
	                            GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
	                            GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* ���� PD.13(A18 (RS))  Ϊ����������� */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* ���� PG10 (CS)) Ϊ����������� */
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource10, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

//	/* PA15 ��DM9000_INT�ж������(������δʹ��) */
//  /* Connect EXTI Line to INT Pin */
//  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource15);
	/*dm9000��λӲ����*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 		//PC9 ������� 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	DM9000_RST;								//DM9000Ӳ����λ
	Delay_ms(1);
	DM9000_SET; 							//DM9000Ӳ����λ����
	Delay_ms(200);								//һ��Ҫ�������ʱ����DM9000׼��������
	
//  /* Configure EXTI line */
//  EXTI_InitStructure.EXTI_Line = EXTI_Line15;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure);

//  /* Enable and set the EXTI interrupt to priority 1*/
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
}

/*
*********************************************************************************************************
*	�� �� ��: DM9K_FSMCConfig
*	����˵��: ����FSMC���ڷ���ʱ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void DM9K_FSMCConfig(void)
{
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  p;

	/*-- FSMC Configuration ------------------------------------------------------*/
	/*----------------------- SRAM Bank 3 ----------------------------------------*/
	/*-- FSMC Configuration ------------------------------------------------------*/
	p.FSMC_AddressSetupTime = 8;		/* ����Ϊ2�����; 3���� */
	p.FSMC_AddressHoldTime = 0;
	p.FSMC_DataSetupTime = 10;			/* ����Ϊ1����2���� */
	p.FSMC_BusTurnAroundDuration = 0;
	p.FSMC_CLKDivision = 0;
	p.FSMC_DataLatency = 0;
	p.FSMC_AccessMode = FSMC_AccessMode_A;
 
//	p.FSMC_AddressSetupTime = 0;		/* ����Ϊ2�����; 3���� */
//	p.FSMC_AddressHoldTime = 0;
//	p.FSMC_DataSetupTime = 3;			/* ����Ϊ1����2���� */
//	p.FSMC_BusTurnAroundDuration = 0x00;
//	p.FSMC_CLKDivision = 0x00;
//	p.FSMC_DataLatency = 0x00;
//	p.FSMC_AccessMode = FSMC_AccessMode_A;
//	
	
	
	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM3;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;	// FSMC_MemoryType_PSRAM;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

	/*!< Enable FSMC Bank1_SRAM3 Bank */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM3, ENABLE);
	
	
	
//	    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
//    FSMC_NORSRAMTimingInitTypeDef  FSMC_NORSRAMTimingInitStructure;
//    FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 10;             /* ??????*/
//    FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 0;      
//    FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 10;                /* ??????*/
//    FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
//    FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0x00;
//    FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0x00;
//    FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;    /* FSMC ????*/

//    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM3;
//    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
//    FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
//    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
//    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
//    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
//    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
//    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
//    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
//    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
//    FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
//    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
//    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
//    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;
//    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 
//    /* ??FSMC???*/
//    FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 5;              /* ??????*/
//    FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 0;      
//    FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 5;                 /* ??????*/
//    FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
//    FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0x00;
//    FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0x00;
//    FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;    /* FSMC????*/
//    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMTimingInitStructure; 
//	  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

//	  /*!< Enable FSMC Bank1_SRAM3 Bank */
//	  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM3, ENABLE);		
}

/*
*********************************************************************************************************
*	�� �� ��: dm9k_ReadReg
*	����˵��: ����DM9000ָ���Ĵ�����ֵ
*	��    ��: reg �Ĵ�����ַ
*	�� �� ֵ: �Ĵ���ֵ
*********************************************************************************************************
*/
uint8_t dm9k_ReadReg(uint8_t reg)
{
	NET_REG_ADDR = reg;
	return (NET_REG_DATA);
}

/*
*********************************************************************************************************
*	�� �� ��: dm9k_WriteReg
*	����˵��: ����DM9000ָ���Ĵ�����ֵ
*	��    ��: reg ���Ĵ�����ַ
*			 writedata : д�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void dm9k_WriteReg(uint8_t reg, uint8_t writedata)
{
	NET_REG_ADDR = reg;
	NET_REG_DATA = writedata;
}

/*
*********************************************************************************************************
*	�� �� ��: dm9k_hash_table
*	����˵��: ���� DM9000A MAC �� �㲥 �� �ಥ �Ĵ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void dm9k_hash_table(void)
{
	uint8_t i;

//	/* ��MAC��ַ����uip */
//	for (i = 0; i < 6; i++)
//	{
//		uip_ethaddr.addr[i] = DEF_MAC_ADDR[i];
//	}

	/* ���� ���� MAC λ�ã������ MyHardware */
	for(i = 0; i < 6; i++)
	{
		dm9k_WriteReg(DM9000_REG_PAR + i, DEF_MAC_ADDR[i]);
	}

	/* ��� �����ಥ���� */
	for(i = 0; i < 8; i++)
	{
		dm9k_WriteReg(DM9000_REG_MAR + i, 0xFF);
	}

//	/* ���� �㲥�� ���� */
//	dm9k_WriteReg(DM9000_REG_MAR + 7, 0x80);
}
/*
*********************************************************************************************************
*	�� �� ��: dm9k_err_reset
*	����˵��: ����ָ�����ʱ�����λDM9000AE
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void dm9k_err_reset(void)
{
 	dm9k_WriteReg(DM9000_REG_NCR,0x03);	   //������:�����λDM9000 
	Delay_us(10); 	
	dm9k_WriteReg(DM9000_REG_NCR,0x03);	   //���岽:�ڶ��������λDM9000 
	Delay_us(10); 	

		/* �����Ǵ���������� */
	dm9k_WriteReg(DM9000_REG_IMR, DM9000_IMR_OFF); 			/* �����ڴ��Ի�ģʽ */
	dm9k_WriteReg(DM9000_REG_TCR2, DM9000_TCR2_SET);			/* ���� LED ��ʾģʽ1:ȫ˫��������˫���� */
	/* ���������Ѷ */
	dm9k_WriteReg(DM9000_REG_NSR, 0x2c);
	dm9k_WriteReg(DM9000_REG_TCR, 0x00);
	dm9k_WriteReg(DM9000_REG_ISR, 0x0f);
	dm9k_WriteReg(DM9000_REG_RCR, DM9000_RCR_SET);			/* ���� ���չ��� */
	dm9k_WriteReg(DM9000_REG_IMR, DM9000_IMR_OFF);			/* �ر� �ж�ģʽ */
	SendPackOk = 0;
}

/*
*********************************************************************************************************
*	�� �� ��: dm9k_reset
*	����˵��: �����λDM9000AE
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void dm9k_reset(void)
{
	
//��λDM9000,��λ����ο�<DM9000 Application Notes V1.22>�ֲ�29ҳ
	DM9000_RST;								//DM9000Ӳ����λ
	Delay_ms(20);
	DM9000_SET; 							//DM9000Ӳ����λ����
	Delay_ms(100);								//һ��Ҫ�������ʱ����DM9000׼��������
 	dm9k_WriteReg(DM9000_REG_GPCR,0x01);			//��һ��:����GPCR�Ĵ���(0X1E)��bit0Ϊ1 
	dm9k_WriteReg(DM9000_REG_GPR,0);				//�ڶ���:����GPR�Ĵ���(0X1F)��bit1Ϊ0��DM9000�ڲ���PHY�ϵ�
	Delay_ms(20);                          //��ʱ2ms���ϵȴ�PHY�ϵ�
 	dm9k_WriteReg(DM9000_REG_NCR,0x03);	   //������:�����λDM9000 
  do 
	{ 
		Delay_ms(1); 
	}while(dm9k_ReadReg(DM9000_REG_NCR)&1);		//�ȴ�DM9000��λ���
	dm9k_WriteReg(DM9000_REG_NCR,0x00);	   //���Ĳ�:�����λ��ɣ�������������ģʽ 
	dm9k_WriteReg(DM9000_REG_NCR,0x03);	   //���岽:�ڶ��������λDM9000 
	do 
	{
		Delay_ms(1);
	}while(dm9k_ReadReg(DM9000_REG_NCR)&1);		//�ȴ�DM9000��λ���
	dm9k_WriteReg(DM9000_REG_NCR,0x00);	   //������:�����λ��ɣ�������������ģʽ 	
		/* �����Ǵ���������� */
	dm9k_WriteReg(DM9000_REG_IMR, DM9000_IMR_OFF); 			/* �����ڴ��Ի�ģʽ */
	dm9k_WriteReg(DM9000_REG_TCR2, DM9000_TCR2_SET);			/* ���� LED ��ʾģʽ1:ȫ˫��������˫���� */
	/* ���������Ѷ */
	dm9k_WriteReg(DM9000_REG_NSR, 0x2c);
	dm9k_WriteReg(DM9000_REG_TCR, 0x00);
	dm9k_WriteReg(DM9000_REG_ISR, 0x0f);
	dm9k_WriteReg(DM9000_REG_RCR, DM9000_RCR_SET);			/* ���� ���չ��� */
	dm9k_WriteReg(DM9000_REG_IMR, DM9000_IMR_OFF);			/* �ر� �ж�ģʽ */
	SendPackOk = 0;
}

/*
*********************************************************************************************************
*	�� �� ��: dm9k_phy_write
*	����˵��: �����λDM9000AE
*	��    ��: phy_reg �� PHY�Ĵ�����ַ
*			  writedata �� д�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void dm9k_phy_write(uint8_t phy_reg, uint16_t writedata)
{
	/* ����д�� PHY �Ĵ�����λ�� */
	dm9k_WriteReg(DM9000_REG_EPAR, phy_reg | DM9000_PHY);

	/* ����д�� PHY �Ĵ�����ֵ */
	dm9k_WriteReg(DM9000_REG_EPDRH, ( writedata >> 8 ) & 0xff);
	dm9k_WriteReg(DM9000_REG_EPDRL, writedata & 0xff);

	dm9k_WriteReg(DM9000_REG_EPCR, 0x0a); 						/* ������д�� PHY �Ĵ��� */
	while(dm9k_ReadReg(DM9000_REG_EPCR) & 0x01);					/* ��Ѱ�Ƿ�ִ�н��� */
	Delay_ms(50);
//	dm9k_WriteReg(DM9000_REG_EPCR, 0x08); 						/* ���д������ */
	 dm9k_WriteReg(DM9000_REG_EPCR, 0x00); 	
}

/*
*********************************************************************************************************
*	�� �� ��: DM9000_Initnic
*	����˵��: ����DM9000AEоƬ����ʼ����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void DM9000_Initnic(void)
{
	
	dm9k_reset();									//��λDM9000
	
	dm9k_WriteReg(DM9000_REG_GPR, DM9000_PHY_OFF);			/* �ر� PHY ������ PHY ����*/
	dm9k_phy_write(0x00, 0x8000);					/* ���� PHY �ļĴ��� */
	dm9k_phy_write(0x04, 0x01e1);					/* ���� ����Ӧģʽ���ݱ� */
	//dm9k_phy_write(0x00, 0x1000);					/* ���� ��������ģʽ */
	/* ����ģʽ����
	  0x0000 : �̶�10M��˫��
	  0x0100 : �̶�10Mȫ˫��
	  0x2000 : �̶�100M��˫��
	  0x2100 : �̶�100Mȫ˫��
	  0x1000 : ����Ӧģʽ
	*/
	dm9k_phy_write(0x00, 0x1000);				/* ���� ��������ģʽ */
	Delay_ms(2000);
	dm9k_WriteReg(DM9000_REG_GPR, DM9000_PHY_ON);				/* ���� PHY ����, ���� PHY */
	Delay_ms(2000);
	dm9k_hash_table();								/* ���� DM9000A MAC �� �ಥ*/
	
}

/*
*********************************************************************************************************
*	�� �� ��: dm9k_receive_packet
*	����˵��: ����DM9000AEоƬ����ʼ����
*	��    ��: _uip_buf : ���ս����ŵĻ�����ָ��
*	�� �� ֵ: > 0 ��ʾ���յ����ݳ���, 0��ʾû������
*********************************************************************************************************
*/
uint16_t dm9k_receive_packet(void)
{
	uint16_t ReceiveData[1600];
	uint8_t  rx_int_count = 0;
	u32  rx_checkbyte;
	uint16_t rx_status, rx_length;
	uint8_t  jump_packet;
	uint16_t i;
	uint16_t calc_len;
	uint16_t calc_MRR;
	uint16_t temp;
	do
	{
		jump_packet = 0;								/* ����������� */
		dm9k_ReadReg(DM9000_REG_MRCMDX);							/* ��ȡ�ڴ����ݣ���ַ������ */
		/* �����ڴ�����λ�� */
		calc_MRR = (dm9k_ReadReg(DM9000_REG_MRRH) << 8) + dm9k_ReadReg(DM9000_REG_MRRL);
		rx_checkbyte = dm9k_ReadReg(DM9000_REG_MRCMDX);			/*  */
		if(rx_checkbyte == DM9000_PKT_RDY)				/* ȡ */
		{
			/* ��ȡ��������Ѷ �� ���� */
			NET_REG_ADDR = DM9000_REG_MRCMD;
			rx_status = NET_REG_DATA;
			rx_length = NET_REG_DATA;
			/* ���յ�����ϵͳ�ɳ��ܵķ�����˰����� */
			if(rx_length > Max_Ethernet_Lenth)
				jump_packet = 1;

#ifdef Broadcast_Jump
			/* ���յ��Ĺ㲥��ಥ�������ض����ȣ��˰����� */
			if(rx_status & 0x4000)
			{
				if(rx_length > Max_Broadcast_Lenth)
					jump_packet = 1;
			}
#endif
			/* ������һ������ָ��λ , �����ճ���Ϊ���������һ����ż�ֽڡ�*/
			/* ���ǳ��� 0x3fff ������ع��Ƶ� 0x0c00 ��ʼλ�� */
			calc_MRR += (rx_length + 4);
			if(rx_length & 0x01) calc_MRR++;
			if(calc_MRR > 0x3fff) calc_MRR -= 0x3400;

			if(jump_packet == 0x01)
			{
				/* ��ָ���Ƶ���һ�����İ�ͷλ�� */
				dm9k_WriteReg (DM9000_REG_MRRH, (calc_MRR >> 8) & 0xff);
				dm9k_WriteReg (DM9000_REG_MRRL, calc_MRR & 0xff );
				continue;
			}

			/* ��ʼ���ڴ�����ϰᵽ��ϵͳ�У�ÿ���ƶ�һ�� word */
			calc_len = (rx_length + 1) >> 1;
			for(i = 0 ; i < calc_len ; i++)
				ReceiveData[i] = NET_REG_DATA;

			/* �������ر��� TCP/IP �ϲ㣬����ȥ���� 4 BYTE �� CRC-32 ����� */
			receiveLen_DM9000 = rx_length - 4;
      memcpy((unsigned char*)receiveBuffer_DM9000,(uint8_t *)ReceiveData,receiveLen_DM9000);
			rx_int_count++;								/* �ۼ��հ����� */

#ifdef FifoPointCheck
			if(calc_MRR != ((dm9k_ReadReg(DM9000_REG_MRRH) << 8) + dm9k_ReadReg(DM9000_REG_MRRL)))
			{
#ifdef Point_Error_Reset
				dm9k_reset();								/* ����ָ��������� */
				return ReceiveLength;
#endif
				/*����ָ�������ָ���Ƶ���һ�����İ�ͷλ��  */
				dm9k_WriteReg(DM9000_REG_MRRH, (calc_MRR >> 8) & 0xff);
				dm9k_WriteReg(DM9000_REG_MRRL, calc_MRR & 0xff);
			}
#endif
			dm9k_WriteReg(DM9000_REG_MRRH, (calc_MRR >> 8) & 0xff);
			dm9k_WriteReg(DM9000_REG_MRRL, calc_MRR & 0xff);
			return receiveLen_DM9000;
		}
		else
		{
			if(rx_checkbyte == DM9000_PKT_NORDY)		/* δ�յ��� */
			{
				dm9k_WriteReg(DM9000_REG_ISR, 0x3f);				/*  */
			}
			else
			{
				dm9k_err_reset();								/* ����ָ��������� */
			}
			return (0);
		}
	}while(rx_int_count < Max_Int_Count);				/* �Ƿ񳬹������շ������ */
	return 0;
}

/*
*********************************************************************************************************
*	�� �� ��: dm9k_send_packet
*	����˵��: ����һ������
*	��    ��: p_char : �������ݻ�����
*	�� �� ֵ: length : ���ݳ���
*********************************************************************************************************
*/
void dm9k_send_packet(uint8_t *p_char, uint16_t length)
{
	uint16_t SendLength = length;
	uint16_t *SendData = (uint16_t *) p_char;
	uint16_t i;
	uint16_t calc_len;
	__IO uint16_t calc_MWR;
  uint32_t timer2;
  
	/* ��� DM9000A �Ƿ��ڴ����У����ǵȴ�ֱ�����ͽ��� */
	dm9k_WriteReg(DM9000_REG_IMR,DM9000_IMR_OFF);		//�ر������ж� 
//	if(SendPackOk == Max_Send_Pack)
//	{
//		timer2=bsp_GetTickCount();
//		while(dm9k_ReadReg(DM9000_REG_TCR) & DM9000_TCR_SET)
//		{
//			Delay_us (5);
//			if(bsp_GetTickCount()-timer2>=1000)
//				 break;
//		}
//		SendPackOk = 0;
//	}

//	SendPackOk++;										/* ���ô��ͼ��� */
	NET_REG_ADDR = DM9000_REG_MWCMD;
	/* ��ʼ��ϵͳ�����ϰᵽ���ڴ��У�ÿ���ƶ�һ�� word */
	calc_len = (SendLength + 1) >> 1;
	for(i = 0; i < calc_len; i++)
		NET_REG_DATA = SendData[i];
		
	dm9k_WriteReg(DM9000_REG_TXPLH, (SendLength >> 8) & 0xff);	/* ���ô��ͷ���ĳ��� */
	dm9k_WriteReg(DM9000_REG_TXPLL, SendLength & 0xff);
	dm9k_WriteReg(DM9000_REG_TCR, DM9000_TCR_SET);				/* ���д��� */
	timer2=bsp_GetTickCount();
	while(dm9k_ReadReg(DM9000_REG_TCR)&0x01)
	   {  
			if(bsp_GetTickCount()-timer2>=100)
				 break;
		 }
	dm9k_WriteReg(DM9000_REG_NSR, 0x2c);
	dm9k_WriteReg(DM9000_REG_IMR , DM9000_IMR_OFF);
//	dm9k_err_reset();
	return;
}
/*
*********************************************************************************************************
*	�� �� ��: dm9k_interrupt
*	����˵��: �жϴ����� (webserver����δʹ���ж�)
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void  dm9k_interrupt(void)
{
	uint8_t  save_reg;
	uint16_t isr_status;

	save_reg = NET_REG_ADDR;							/* �ݴ���ʹ�õ�λ�� */
	dm9k_WriteReg(DM9000_REG_IMR , DM9000_IMR_OFF);				/* �ر� DM9000A �ж� */
	isr_status = dm9k_ReadReg(DM9000_REG_ISR);					/* ȡ���жϲ���ֵ */

	if (isr_status & DM9000_RX_INTR)
	{ 					/* ����Ƿ�Ϊ�����ж� */
		//dm9k_receive_packet();							/* ִ�н��մ������ */
	}
	dm9k_WriteReg(DM9000_REG_IMR , DM9000_IMR_SET);				/* ���� DM9000A �ж� */
	NET_REG_ADDR = save_reg;							/* �ظ���ʹ�õ�λ�� */

}
/**
  * @brief  PHY�ⲿ�жϷ������
  * @param  ��
  * @retval ��
  */
//void EXTI15_10_IRQHandler(void)
//{
//	 EXTI_ClearITPendingBit(EXTI_Line15);
//	if(EXTI_GetITStatus(EXTI_Line15) != RESET)
//  {
//    dm9k_interrupt();
//    /* Clear interrupt pending bit */
//   
//  }
//}
/*******************************************************************************
*	������: dm9k_ReadID
*	��  ��: ��
*	��  ��: ��
*	��  ��: ��ȡоƬID
*/
uint32_t dm9k_ReadID(void)
{
	uint8_t vid1,vid2,pid1,pid2;

	if (s_FSMC_Init_Ok == 0)
	{
		DM9K_CtrlLinesConfig();
		DM9K_FSMCConfig();

		s_FSMC_Init_Ok = 1;
	}
	vid1 = dm9k_ReadReg(DM9000_REG_VID_L) & 0xFF;
	vid2 = dm9k_ReadReg(DM9000_REG_VID_H) & 0xFF;
	pid1 = dm9k_ReadReg(DM9000_REG_PID_L) & 0xFF;
	pid2 = dm9k_ReadReg(DM9000_REG_PID_H) & 0xFF;
	return (vid2 << 24) | (vid1 << 16) | (pid2 << 8) | pid1;
}

