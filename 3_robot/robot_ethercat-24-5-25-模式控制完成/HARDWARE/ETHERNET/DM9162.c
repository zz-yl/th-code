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
//LAN8720初始化
//返回值:0,成功;
//    其他,失败
u8 DM9162_Init(void)
{
	 u8 rval=0;
    ETH_GPIO_Config_RMII(); 
//	ETHERNET_NVICConfiguration();
	  rval=ETH_MACDMA_Config();//此处需等待100ms，等待以太网配置完成；
	  /* Read PHY status register: Get Ethernet link status */
		if(ETH_ReadPHYRegister(DM9162_PHY_ADDRESS, PHY_SR) & 4)
		{
			EthStatus |= ETH_LINK_FLAG;
		}
//  /* Configure the PHY to generate an interrupt on change of link status */
//  Eth_Link_PHYITConfig(DM9162_PHY_ADDRESS);
//  /* Configure the EXTI for Ethernet link status. */
//    Eth_Link_EXTIConfig();
	  return !rval;					//ETH的规则为:0,失败;1,成功;所以要取反一下 
}
//得到8720的速度模式
//返回值:
//001:10M半双工
//101:10M全双工
//010:100M半双工
//110:100M全双工
//其他:错误.
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
//以下部分为STM32F407网卡配置/接口函数.

//初始化ETH MAC层及DMA配置
//返回值:ETH_ERROR,发送失败(0)
//		ETH_SUCCESS,发送成功(1)

u8 ETH_MACDMA_Config(void)
{
	u8 rval;
	ETH_InitTypeDef ETH_InitStructure; 
	
	//使能以太网时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC | RCC_AHB1Periph_ETH_MAC_Tx |RCC_AHB1Periph_ETH_MAC_Rx, ENABLE);
                        
	ETH_DeInit();  								//AHB总线重启以太网
	ETH_SoftwareReset();  						//软件重启网络
	while (ETH_GetSoftwareResetStatus() == SET);//等待软件重启网络完成
	
	ETH_StructInit(&ETH_InitStructure); 	 	//初始化网络为默认值  

	///网络MAC参数设置 
	ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;   			//开启网络自适应功能
	ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;					//关闭反馈
	ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable; 		//关闭重传功能
	ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable; 	//关闭自动去除PDA/CRC功能 
	ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Enable;						//关闭接收所有的帧
	ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;//允许接收所有广播帧
	ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;			//关闭混合模式的地址过滤  
	ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;//对于组播地址使用完美地址过滤   
	ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;	//对单播地址使用完美地址过滤 
	ETH_InitStructure.ETH_ChecksumOffload = ETH_ChecksumOffload_Enable; 			//开启ipv4和TCP/UDP/ICMP的帧校验和卸载   
	//当我们使用帧校验和卸载功能的时候，一定要使能存储转发模式,存储转发模式中要保证整个帧存储在FIFO中,
	//这样MAC能插入/识别出帧校验值,当真校验正确的时候DMA就可以处理帧,否则就丢弃掉该帧
	ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable; //开启丢弃TCP/IP错误帧
	ETH_InitStructure.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;     //开启接收数据的存储转发模式    
	ETH_InitStructure.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;   //开启发送数据的存储转发模式  

	ETH_InitStructure.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Disable;     	//禁止转发错误帧  
	ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Disable;	//不转发过小的好帧 
	ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Enable;  		//打开处理第二帧功能
	ETH_InitStructure.ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;  	//开启DMA传输的地址对齐功能
	ETH_InitStructure.ETH_FixedBurst = ETH_FixedBurst_Enable;            			//开启固定突发功能    
	ETH_InitStructure.ETH_RxDMABurstLength = ETH_RxDMABurstLength_32Beat;     		//DMA发送的最大突发长度为32个节拍   
	ETH_InitStructure.ETH_TxDMABurstLength = ETH_TxDMABurstLength_32Beat;			//DMA接收的最大突发长度为32个节拍
	ETH_InitStructure.ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_2_1;

	rval=ETH_Init(&ETH_InitStructure,DM9162_PHY_ADDRESS);		//配置ETH

	EthStatus=rval;
//	if(rval==ETH_SUCCESS)//配置成功
//	{
//		ETH_DMAITConfig(ETH_DMA_IT_NIS|ETH_DMA_IT_R,ENABLE);  	//使能以太网接收中断	
//	}
	return rval;
}

//以太网中断服务函数
//void ETH_IRQHandler(void)
//{
//	while(ETH_CheckFrameReceived()) 	//检测是否收到数据包
//	{ 
//	 lwip_pkt_handle();		
//	}
//	ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
//	ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
//}  
//接收一个网卡数据包
//返回值:网络数据包帧结构体
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
//	//检查当前描述符,是否属于ETHERNET DMA(设置的时候)/CPU(复位的时候)
//	if((DMARxDescToGet->Status&ETH_DMARxDesc_OWN)!=(u32)RESET)
//	{	
//		frame.length=ETH_ERROR; 
//		if ((ETH->DMASR&ETH_DMASR_RBUS)!=(u32)RESET)  
//		{ 
//			ETH->DMASR = ETH_DMASR_RBUS;//清除ETH DMA的RBUS位  
//			ETH->DMARPDR=0;//恢复DMA接收
//		}
//		return frame;//错误,OWN位被设置了
//	}  
//	if(((DMARxDescToGet->Status&ETH_DMARxDesc_ES)==(u32)RESET)&& 
//	((DMARxDescToGet->Status & ETH_DMARxDesc_LS)!=(u32)RESET)&&  
//	((DMARxDescToGet->Status & ETH_DMARxDesc_FS)!=(u32)RESET))  
//	{       
//		framelength=((DMARxDescToGet->Status&ETH_DMARxDesc_FL)>>ETH_DMARxDesc_FrameLengthShift)-4;//得到接收包帧长度(不包含4字节CRC)
// 		frame.buffer = DMARxDescToGet->Buffer1Addr;//得到包数据所在的位置
//	}else framelength=ETH_ERROR;//错误  
//	frame.length=framelength; 
//	frame.descriptor=DMARxDescToGet;  
//  asdjsdk.length=framelength;
//	asdjsdk.descriptor=DMARxDescToGet;
//	//更新ETH DMA全局Rx描述符为下一个Rx描述符
//	//为下一次buffer读取设置下一个DMA Rx描述符
//	DMARxDescToGet=(ETH_DMADESCTypeDef*)(DMARxDescToGet->Buffer2NextDescAddr); 
//  sdfdfsf=(ETH_DMADESCTypeDef*)(DMARxDescToGet->Buffer2NextDescAddr);
//	return frame;  
}
//发送一个网卡数据包
//FrameLength:数据包长度
//返回值:ETH_ERROR,发送失败(0)
//		ETH_SUCCESS,发送成功(1)
u8 ETH_Tx_Packet(u16 FrameLength)
{   
	//检查当前描述符,是否属于ETHERNET DMA(设置的时候)/CPU(复位的时候)
	if((DMATxDescToSet->Status&ETH_DMATxDesc_OWN)!=(u32)RESET)return ETH_ERROR;//错误,OWN位被设置了 
 	DMATxDescToSet->ControlBufferSize=(FrameLength&ETH_DMATxDesc_TBS1);//设置帧长度,bits[12:0]
	DMATxDescToSet->Status|=ETH_DMATxDesc_LS|ETH_DMATxDesc_FS;//设置最后一个和第一个位段置位(1个描述符传输一帧)
  	DMATxDescToSet->Status|=ETH_DMATxDesc_OWN;//设置Tx描述符的OWN位,buffer重归ETH DMA
	if((ETH->DMASR&ETH_DMASR_TBUS)!=(u32)RESET)//当Tx Buffer不可用位(TBUS)被设置的时候,重置它.恢复传输
	{ 
		ETH->DMASR=ETH_DMASR_TBUS;//重置ETH DMA TBUS位 
		ETH->DMATPDR=0;//恢复DMA发送
	} 
	//更新ETH DMA全局Tx描述符为下一个Tx描述符
	//为下一次buffer发送设置下一个DMA Tx描述符 
	DMATxDescToSet=(ETH_DMADESCTypeDef*)(DMATxDescToSet->Buffer2NextDescAddr); 
	return ETH_SUCCESS;   
}
//得到当前描述符的Tx buffer地址
//返回值:Tx buffer地址
u32 ETH_GetCurrentTxBuffer(void)
{  
  return DMATxDescToSet->Buffer1Addr;//返回Tx buffer地址  
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

	/* 安富莱STM32-F4开发板网卡接口采用RMII
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
		/*dm9161复位硬引脚*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 		//PB10 推挽输出 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	DM9161_RST;								//DM9000硬件复位
	Delay_ms(10);
	DM9161_SET; 							//DM9000硬件复位结束
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

