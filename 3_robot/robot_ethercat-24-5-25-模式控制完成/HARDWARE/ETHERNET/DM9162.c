#include "DM9162.h"
#include "DM9000.h"
#include "stm32f4x7_eth.h"
#include "usart.h" 
#include "bsp_timer.h"
#include <string.h>
#include "bsp.h"

ETH_InitTypeDef ETH_InitStructure;
__IO uint32_t  EthStatus = 0;
u8 MyMacAddr[6] = {0x08, 0x00, 0x06, 0x00, 0x00, 0x09}; 


//static void ETHERNET_NVICConfiguration(void);
static void ETH_GPIO_Config_RMII(void);
void lwip_pkt_handle(void);	
//LAN8720��ʼ��
//����ֵ:0,�ɹ�;
//    ����,ʧ��
u8 DM9162_Init(void)
{
	 u8 rval=0;
    ETH_GPIO_Config_RMII(); 
//	ETHERNET_NVICConfiguration();
	  rval=ETH_MACDMA_Config();//�˴���ȴ�100ms���ȴ���̫��������ɣ�
	  /* Read PHY status register: Get Ethernet link status */
		if(ETH_ReadPHYRegister(DM9162_PHY_ADDRESS, PHY_SR) & 4)
		{
			EthStatus |= ETH_LINK_FLAG;
		}
//  /* Configure the PHY to generate an interrupt on change of link status */
//  Eth_Link_PHYITConfig(DM9162_PHY_ADDRESS);
//  /* Configure the EXTI for Ethernet link status. */
//    Eth_Link_EXTIConfig();
	  return !rval;					//ETH�Ĺ���Ϊ:0,ʧ��;1,�ɹ�;����Ҫȡ��һ�� 
}
//�õ�8720���ٶ�ģʽ
//����ֵ:
//001:10M��˫��
//101:10Mȫ˫��
//010:100M��˫��
//110:100Mȫ˫��
//����:����.
void MAC_Init(void)
{
    int i;

    /* initialize MAC address in ethernet MAC */
    ETH_MACAddressConfig(ETH_MAC_Address0, MyMacAddr);
    /* Initialize Tx Descriptors list: Chain Mode */
    ETH_DMATxDescChainInit(DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
    /* Initialize Rx Descriptors list: Chain Mode  */
    ETH_DMARxDescChainInit(DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);
 
    /* Enable the TCP, UDP and ICMP checksum insertion for the Tx frames */
    for(i = 0; i < ETH_TXBUFNB; i++) {
        ETH_DMATxDescChecksumInsertionConfig(&DMATxDscrTab[i], ETH_DMATxDesc_ChecksumTCPUDPICMPFull);
    }
    ETH_Start();
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//���²���ΪSTM32F407��������/�ӿں���.

//��ʼ��ETH MAC�㼰DMA����
//����ֵ:ETH_ERROR,����ʧ��(0)
//		ETH_SUCCESS,���ͳɹ�(1)

u8 ETH_MACDMA_Config(void)
{
	u8 rval;
	ETH_InitTypeDef ETH_InitStructure; 
	
	//ʹ����̫��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC | RCC_AHB1Periph_ETH_MAC_Tx |RCC_AHB1Periph_ETH_MAC_Rx, ENABLE);
                        
	ETH_DeInit();  								//AHB����������̫��
	ETH_SoftwareReset();  						//�����������
	while (ETH_GetSoftwareResetStatus() == SET);//�ȴ���������������
	
	ETH_StructInit(&ETH_InitStructure); 	 	//��ʼ������ΪĬ��ֵ  

	///����MAC�������� 
	ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;   			//������������Ӧ����
	ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;					//�رշ���
	ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable; 		//�ر��ش�����
	ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable; 	//�ر��Զ�ȥ��PDA/CRC���� 
	ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Enable;						//�رս������е�֡
	ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;//����������й㲥֡
	ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;			//�رջ��ģʽ�ĵ�ַ����  
	ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;//�����鲥��ַʹ��������ַ����   
	ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;	//�Ե�����ַʹ��������ַ���� 
	ETH_InitStructure.ETH_ChecksumOffload = ETH_ChecksumOffload_Enable; 			//����ipv4��TCP/UDP/ICMP��֡У���ж��   
	//������ʹ��֡У���ж�ع��ܵ�ʱ��һ��Ҫʹ�ܴ洢ת��ģʽ,�洢ת��ģʽ��Ҫ��֤����֡�洢��FIFO��,
	//����MAC�ܲ���/ʶ���֡У��ֵ,����У����ȷ��ʱ��DMA�Ϳ��Դ���֡,����Ͷ�������֡
	ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable; //��������TCP/IP����֡
	ETH_InitStructure.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;     //�����������ݵĴ洢ת��ģʽ    
	ETH_InitStructure.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;   //�����������ݵĴ洢ת��ģʽ  

	ETH_InitStructure.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Disable;     	//��ֹת������֡  
	ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Disable;	//��ת����С�ĺ�֡ 
	ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Enable;  		//�򿪴���ڶ�֡����
	ETH_InitStructure.ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;  	//����DMA����ĵ�ַ���빦��
	ETH_InitStructure.ETH_FixedBurst = ETH_FixedBurst_Enable;            			//�����̶�ͻ������    
	ETH_InitStructure.ETH_RxDMABurstLength = ETH_RxDMABurstLength_32Beat;     		//DMA���͵����ͻ������Ϊ32������   
	ETH_InitStructure.ETH_TxDMABurstLength = ETH_TxDMABurstLength_32Beat;			//DMA���յ����ͻ������Ϊ32������
	ETH_InitStructure.ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_2_1;

	rval=ETH_Init(&ETH_InitStructure,DM9162_PHY_ADDRESS);		//����ETH

	EthStatus=rval;
//	if(rval==ETH_SUCCESS)//���óɹ�
//	{
//		ETH_DMAITConfig(ETH_DMA_IT_NIS|ETH_DMA_IT_R,ENABLE);  	//ʹ����̫�������ж�	
//	}
	return rval;
}

//��̫���жϷ�����
//void ETH_IRQHandler(void)
//{
//	while(ETH_CheckFrameReceived()) 	//����Ƿ��յ����ݰ�
//	{ 
//	 lwip_pkt_handle();		
//	}
//	ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
//	ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
//}  
//����һ���������ݰ�
//����ֵ:�������ݰ�֡�ṹ��
FrameTypeDef ETH_Rx_Packet(void)
{ 
	uint32_t framelength = 0;
  FrameTypeDef frame = {0,0,0}; 
  
  /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
  framelength = ((DMARxDescToGet->Status & ETH_DMARxDesc_FL) >> ETH_DMARxDesc_FrameLengthShift) - 4;
  frame.length = framelength;
  
  /* Get the address of the buffer start address */ 
  /* Check if more than one segment in the frame */
  if (DMA_RX_FRAME_infos->Seg_Count >1)
  {
    frame.buffer =(DMA_RX_FRAME_infos->FS_Rx_Desc)->Buffer1Addr;
  }
  else 
  {
    frame.buffer = DMARxDescToGet->Buffer1Addr;
  }

  frame.descriptor = DMARxDescToGet;
  
  /* Update the ETHERNET DMA global Rx descriptor with next Rx descriptor */      
  /* Chained Mode */    
  /* Selects the next DMA Rx descriptor list for next buffer to read */ 
  DMARxDescToGet = (ETH_DMADESCTypeDef*) (DMARxDescToGet->Buffer2NextDescAddr);    
  
  /* Return Frame */
  return (frame);  
	
	
	
//	u32 framelength=0;
//	FrameTypeDef frame={0,0};   
//	//��鵱ǰ������,�Ƿ�����ETHERNET DMA(���õ�ʱ��)/CPU(��λ��ʱ��)
//	if((DMARxDescToGet->Status&ETH_DMARxDesc_OWN)!=(u32)RESET)
//	{	
//		frame.length=ETH_ERROR; 
//		if ((ETH->DMASR&ETH_DMASR_RBUS)!=(u32)RESET)  
//		{ 
//			ETH->DMASR = ETH_DMASR_RBUS;//���ETH DMA��RBUSλ  
//			ETH->DMARPDR=0;//�ָ�DMA����
//		}
//		return frame;//����,OWNλ��������
//	}  
//	if(((DMARxDescToGet->Status&ETH_DMARxDesc_ES)==(u32)RESET)&& 
//	((DMARxDescToGet->Status & ETH_DMARxDesc_LS)!=(u32)RESET)&&  
//	((DMARxDescToGet->Status & ETH_DMARxDesc_FS)!=(u32)RESET))  
//	{       
//		framelength=((DMARxDescToGet->Status&ETH_DMARxDesc_FL)>>ETH_DMARxDesc_FrameLengthShift)-4;//�õ����հ�֡����(������4�ֽ�CRC)
// 		frame.buffer = DMARxDescToGet->Buffer1Addr;//�õ����������ڵ�λ��
//	}else framelength=ETH_ERROR;//����  
//	frame.length=framelength; 
//	frame.descriptor=DMARxDescToGet;  
//  asdjsdk.length=framelength;
//	asdjsdk.descriptor=DMARxDescToGet;
//	//����ETH DMAȫ��Rx������Ϊ��һ��Rx������
//	//Ϊ��һ��buffer��ȡ������һ��DMA Rx������
//	DMARxDescToGet=(ETH_DMADESCTypeDef*)(DMARxDescToGet->Buffer2NextDescAddr); 
//  sdfdfsf=(ETH_DMADESCTypeDef*)(DMARxDescToGet->Buffer2NextDescAddr);
//	return frame;  
}
//����һ���������ݰ�
//FrameLength:���ݰ�����
//����ֵ:ETH_ERROR,����ʧ��(0)
//		ETH_SUCCESS,���ͳɹ�(1)
u8 ETH_Tx_Packet(u16 FrameLength)
{   
	//��鵱ǰ������,�Ƿ�����ETHERNET DMA(���õ�ʱ��)/CPU(��λ��ʱ��)
	if((DMATxDescToSet->Status&ETH_DMATxDesc_OWN)!=(u32)RESET)return ETH_ERROR;//����,OWNλ�������� 
 	DMATxDescToSet->ControlBufferSize=(FrameLength&ETH_DMATxDesc_TBS1);//����֡����,bits[12:0]
	DMATxDescToSet->Status|=ETH_DMATxDesc_LS|ETH_DMATxDesc_FS;//�������һ���͵�һ��λ����λ(1������������һ֡)
  	DMATxDescToSet->Status|=ETH_DMATxDesc_OWN;//����Tx��������OWNλ,buffer�ع�ETH DMA
	if((ETH->DMASR&ETH_DMASR_TBUS)!=(u32)RESET)//��Tx Buffer������λ(TBUS)�����õ�ʱ��,������.�ָ�����
	{ 
		ETH->DMASR=ETH_DMASR_TBUS;//����ETH DMA TBUSλ 
		ETH->DMATPDR=0;//�ָ�DMA����
	} 
	//����ETH DMAȫ��Tx������Ϊ��һ��Tx������
	//Ϊ��һ��buffer����������һ��DMA Tx������ 
	DMATxDescToSet=(ETH_DMADESCTypeDef*)(DMATxDescToSet->Buffer2NextDescAddr); 
	return ETH_SUCCESS;   
}
//�õ���ǰ��������Tx buffer��ַ
//����ֵ:Tx buffer��ַ
u32 ETH_GetCurrentTxBuffer(void)
{  
  return DMATxDescToSet->Buffer1Addr;//����Tx buffer��ַ  
}
void lwip_pkt_handle(void)
{
//		 struct pbuf *p, *q;
		u16 len;
		int l =0;
		FrameTypeDef frame;
		u8 *buffer;
		uint32_t i=0;
		__IO ETH_DMADESCTypeDef *DMARxNextDesc;
		
		
//		p = NULL;
		
		/* get received frame */
		frame = ETH_Get_Received_Frame();
		
		/* Obtain the size of the packet and put it into the "len" variable. */
		len = frame.length;
		buffer = (u8 *)frame.buffer;
		receiveLen=len;
	  memcpy((u8 *)receiveBuffer, (u8 *)buffer, len);
		/* We allocate a pbuf chain of pbufs from the Lwip buffer pool */
//		p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
		
		/* copy received frame to pbuf chain */
//		if (p != NULL)
//		{
//			for (q = p; q != NULL; q = q->next)
//			{
//				memcpy((u8_t*)q->payload, (u8_t*)&buffer[l], q->len);
//				l = l + q->len;
//			}    
//		}
		
		/* Release descriptors to DMA */
		/* Check if frame with multiple DMA buffer segments */
		if (DMA_RX_FRAME_infos->Seg_Count > 1)
		{
			DMARxNextDesc = DMA_RX_FRAME_infos->FS_Rx_Desc;
		}
		else
		{
			DMARxNextDesc = frame.descriptor;
		}
		
		/* Set Own bit in Rx descriptors: gives the buffers back to DMA */
		for (i=0; i<DMA_RX_FRAME_infos->Seg_Count; i++)
		{  
			DMARxNextDesc->Status = ETH_DMARxDesc_OWN;
			DMARxNextDesc = (ETH_DMADESCTypeDef *)(DMARxNextDesc->Buffer2NextDescAddr);
		}
		
		/* Clear Segment_Count */
		DMA_RX_FRAME_infos->Seg_Count =0;
		
		/* When Rx Buffer unavailable flag is set: clear it and resume reception */
		if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)  
		{
			/* Clear RBUS ETHERNET DMA flag */
			ETH->DMASR = ETH_DMASR_RBUS;
			/* Resume DMA reception */
			ETH->DMARPDR = 0;
		}
		
//	  uint32_t i=0;
//    uint32_t *p32;
//    FrameTypeDef frame;
//	__IO ETH_DMADESCTypeDef *DMARxNextDesc;
//    /* get received frame */
////	   receiveLen=0;   
////     memset(receiveBuffer, 0, sizeof(receiveBuffer));
//     frame = ETH_Rx_Packet();
//    /* Obtain the size of the packet and put it into the "len" variable. */
//     receiveLen = frame.length;
//	   memcpy((u8*)(receiveBuffer), (u8*)frame.buffer, receiveLen);
//    /* Check if frame with multiple DMA buffer segments */
//    if (DMA_RX_FRAME_infos->Seg_Count > 1) 
//			{
//        DMARxNextDesc = DMA_RX_FRAME_infos->FS_Rx_Desc;
//       } 
//		else 
//			{
//        DMARxNextDesc = frame.descriptor;
//       }
//    /* Set Own bit in Rx descriptors: gives the buffers back to DMA */
//    for (i = 0; i < DMA_RX_FRAME_infos->Seg_Count; i++) 
//			 {
//        DMARxNextDesc->Status = ETH_DMARxDesc_OWN;
//        DMARxNextDesc = (ETH_DMADESCTypeDef *)(DMARxNextDesc->Buffer2NextDescAddr);
//       }
//    /* Clear Segment_Count */
//    DMA_RX_FRAME_infos->Seg_Count = 0;
// 
//    /* When Rx Buffer unavailable flag is set: clear it and resume reception */
//    if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)
//			{
//        /* Clear RBUS ETHERNET DMA flag */
//        ETH->DMASR = ETH_DMASR_RBUS;
//        /* Resume DMA reception */
//        ETH->DMARPDR = 0;
//      }
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
static void ETH_GPIO_Config_RMII(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ������STM32-F4�����������ӿڲ���RMII
		PA1/ETH_RMII_RX_CLK
		PA2/ETH_MDIO
		PA7/RMII_CRS_DV
		PC1/ETH_MDC
		PC4/ETH_RMII_RX_D0
		PC5/ETH_RMII_RX_D1
		PG11/ETH_RMII_TX_EN
		PG13/FSMC_A24/ETH_RMII_TXD0
		PG14/ETH_RMII_TXD1

		PH6/MII_INT
	*/
	/* Enable GPIOs clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOG |
	                     RCC_AHB1Periph_GPIOH, ENABLE);

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_RMII);

	/* Configure PA1, PA2 and PA7 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_ETH);

	/* Configure PC1, PC4 and PC5 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource1, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_ETH);

	/* Configure PG11, PG14 and PG13 */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource11, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource13, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_ETH);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
		/*dm9161��λӲ����*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 		//PB10 ������� 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	DM9161_RST;								//DM9000Ӳ����λ
	Delay_ms(10);
	DM9161_SET; 							//DM9000Ӳ����λ����
	Delay_ms(100);								
}



/**
  * @brief  Configure the PHY to generate an interrupt on change of link status.
  * @param PHYAddress: external PHY address
  * @retval None
  */
uint32_t Eth_Link_PHYITConfig(uint16_t PHYAddress)
{
  uint32_t tmpreg = 0;
	tmpreg = ETH_ReadPHYRegister(PHYAddress,21);
	tmpreg &= ~(3<<8);
	if(!(ETH_WritePHYRegister(PHYAddress,21,tmpreg)))
	{
			return ETH_ERROR;
	}
	return ETH_SUCCESS;
}

/**
  * @brief  EXTI configuration for Ethernet link status.
  * @param PHYAddress: external PHY address
  * @retval None
  */
//void Eth_Link_EXTIConfig(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//  EXTI_InitTypeDef EXTI_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;

//  /* Enable the INT (PB14) Clock */
//  RCC_AHB1PeriphClockCmd(ETH_LINK_GPIO_CLK, ENABLE);
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

//  /* Configure INT pin as input */
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_InitStructure.GPIO_Pin = ETH_LINK_PIN;
//  GPIO_Init(ETH_LINK_GPIO_PORT, &GPIO_InitStructure);

//  /* Connect EXTI Line to INT Pin */
//  SYSCFG_EXTILineConfig(ETH_LINK_EXTI_PORT_SOURCE, ETH_LINK_EXTI_PIN_SOURCE);

//  /* Configure EXTI line */
//  EXTI_InitStructure.EXTI_Line = ETH_LINK_EXTI_LINE;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure);

//  /* Enable and set the EXTI interrupt to priority 1*/
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//}

/**
  * @brief  This function handles Ethernet link status.
  * @param  None
  * @retval None
  */
//unsigned char tesh=0;
//unsigned char tesh1=0;
//void Eth_Link_ITHandler(uint16_t PHYAddress)
//{
//  /* Check whether the link interrupt has occurred or not */
//  if(((ETH_ReadPHYRegister(PHYAddress, PHY_MISR)) & PHY_LINK_STATUS) != 0)
//  {
//		tesh1++;
//    if((ETH_ReadPHYRegister(PHYAddress, 0x01) & PHY_LINK_STATUS))
//    {
// //     netif_set_link_up(&gnetif);
//		tesh=1;
//    }
//    else
//    {
// //     netif_set_link_down(&gnetif);
//		tesh=2;
//    }
//  }
//}

//void EXTI15_10_IRQHandler(void)
//{
//	tesh1++;
//  if(EXTI_GetITStatus(ETH_LINK_EXTI_LINE) != RESET)
//  {
//    Eth_Link_ITHandler(DM9162_PHY_ADDRESS);
//    /* Clear interrupt pending bit */
//    EXTI_ClearITPendingBit(ETH_LINK_EXTI_LINE);
//  }
//}

