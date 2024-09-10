#ifndef __DM9162_H
#define __DM9162_H
#include "sys.h"
#include "stm32f4x7_eth.h"
#include "stm32f4xx.h"

#define DM9162_PHY_ADDRESS       ((uint16_t) 0x01)

/* Specific defines for EXTI line, used to manage Ethernet link status */
#define ETH_LINK_EXTI_LINE             EXTI_Line14
#define ETH_LINK_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOB
#define ETH_LINK_EXTI_PIN_SOURCE       EXTI_PinSource14
#define ETH_LINK_EXTI_IRQn             EXTI15_10_IRQn
/* PB14 */
#define ETH_LINK_PIN                   GPIO_Pin_14
#define ETH_LINK_GPIO_PORT             GPIOB
#define ETH_LINK_GPIO_CLK              RCC_AHB1Periph_GPIOB

/* Ethernet Flags for EthStatus variable */
#define ETH_INIT_FLAG           0x01 /* Ethernet Init Flag */
#define ETH_LINK_FLAG           0x10 /* Ethernet Link Flag */ 


#define	DM9161_RST  GPIO_ResetBits(GPIOB,GPIO_Pin_10)								
#define	DM9161_SET  GPIO_SetBits(GPIOB,GPIO_Pin_10)


extern __align(4) ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB];/* Ethernet Rx MA Descriptor */
extern __align(4) ETH_DMADESCTypeDef  DMATxDscrTab[ETH_TXBUFNB];/* Ethernet Tx DMA Descriptor */
extern __align(4) uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE]; /* Ethernet Receive Buffer */
extern __align(4) uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE]; /* Ethernet Transmit Buffer */


extern ETH_DMADESCTypeDef  *DMATxDescToSet;			//DMA����������׷��ָ��
extern ETH_DMADESCTypeDef  *DMARxDescToGet; 		//DMA����������׷��ָ�� 
extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;	//DMA�����յ���֡��Ϣָ��
extern __IO uint32_t  EthStatus;	/* �ṩ��������ʹ�� */ 

void  ETH_BSP_Config(void);
uint32_t Eth_Link_PHYITConfig(uint16_t PHYAddress);
void Eth_Link_EXTIConfig(void);
void Eth_Link_ITHandler(uint16_t PHYAddress);
u8 DM9162_Init(void);
void MAC_Init(void);
u8 ETH_MACDMA_Config(void);
FrameTypeDef ETH_Rx_Packet(void);
u8 ETH_Tx_Packet(u16 FrameLength);
u32 ETH_GetCurrentTxBuffer(void);
#endif 

