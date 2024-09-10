/**
 ****************************************************************************************************
 * @file        lwip_comm.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-08-01
 * @brief       lwIP��������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� ������ H743������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20211202
 * ��һ�η���
 *
 ****************************************************************************************************
 */
 
#include "lwip_comm.h"
#include "netif/etharp.h"
#include "lwip/dhcp.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/init.h"
#include "ethernetif.h"
#include "lwip/timeouts.h"
#include "lwip/tcpip.h"
//#include "./MALLOC/malloc.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/ETHERNET/ethernet_chip.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>


__lwip_dev g_lwipdev;                                       /* lwip���ƽṹ�� */
struct netif g_lwip_netif;                                  /* ����һ��ȫ�ֵ�����ӿ� */

#if LWIP_DHCP
uint32_t g_dhcp_fine_timer = 0;                             /* DHCP��ϸ�����ʱ�� */
__IO uint8_t g_lwip_dhcp_state = LWIP_DHCP_OFF;             /* DHCP״̬��ʼ�� */
#endif

/* LINK�߳����� */
#define LWIP_LINK_TASK_PRIO             3                   /* �������ȼ� */
#define LWIP_LINK_STK_SIZE              128 * 2             /* �����ջ��С */
void lwip_link_thread( void * argument );                   /* ��·�߳� */

/* DHCP�߳����� */
#define LWIP_DHCP_TASK_PRIO             4                   /* �������ȼ� */
#define LWIP_DHCP_STK_SIZE              128 * 2             /* �����ջ��С */
void lwip_periodic_handle(void *argument);                  /* DHCP�߳� */
void lwip_link_status_updated(struct netif *netif);         /* DHCP״̬�ص����� */


/**
 * @breif       lwip Ĭ��IP����
 * @param       lwipx: lwip���ƽṹ��ָ��
 * @retval      ��
 */
void lwip_comm_default_ip_set(__lwip_dev *lwipx)
{
    /* Ĭ��Զ��IPΪ:192.168.1.134 */
    lwipx->remoteip[0] = 192;
    lwipx->remoteip[1] = 168;
    lwipx->remoteip[2] = 3;
    lwipx->remoteip[3] = 134;
    
    /* MAC��ַ���� */
    lwipx->mac[0] = 0xB8;
    lwipx->mac[1] = 0xAE;
    lwipx->mac[2] = 0x1D;
    lwipx->mac[3] = 0x00;
    lwipx->mac[4] = 0x04;
    lwipx->mac[5] = 0x00;
    
    /* Ĭ�ϱ���IPΪ:192.168.1.30 */
    lwipx->ip[0] = 192;
    lwipx->ip[1] = 168;
    lwipx->ip[2] = 3;
    lwipx->ip[3] = 240;
    /* Ĭ����������:255.255.255.0 */
    lwipx->netmask[0] = 255;
    lwipx->netmask[1] = 255;
    lwipx->netmask[2] = 255;
    lwipx->netmask[3] = 0;
    
    /* Ĭ������:192.168.1.1 */
    lwipx->gateway[0] = 192;
    lwipx->gateway[1] = 168;
    lwipx->gateway[2] = 3;
    lwipx->gateway[3] = 1;
    lwipx->dhcpstatus = 0; /* û��DHCP */
}

/**
 * @breif       LWIP��ʼ��(LWIP������ʱ��ʹ��)
 * @param       ��
 * @retval      0,�ɹ�
 *              1,�ڴ����
 *              2,��̫��оƬ��ʼ��ʧ��
 *              3,�������ʧ��.
 */
uint8_t lwip_comm_init(void)
{
    struct netif *netif_init_flag;                  /* ����netif_add()����ʱ�ķ���ֵ,�����ж������ʼ���Ƿ�ɹ� */
    ip_addr_t ipaddr;                               /* ip��ַ */
    ip_addr_t netmask;                              /* �������� */
    ip_addr_t gw;                                   /* Ĭ������ */
    
    tcpip_init(NULL, NULL);

    lwip_comm_default_ip_set(&g_lwipdev);           /* ����Ĭ��IP����Ϣ */

#if LWIP_DHCP                                       /* ʹ�ö�̬IP */
    ip_addr_set_zero_ip4(&ipaddr);
    ip_addr_set_zero_ip4(&netmask);
    ip_addr_set_zero_ip4(&gw);
#else   /* ʹ�þ�̬IP */
    IP4_ADDR(&ipaddr, g_lwipdev.ip[0], g_lwipdev.ip[1], g_lwipdev.ip[2], g_lwipdev.ip[3]);
    IP4_ADDR(&netmask, g_lwipdev.netmask[0], g_lwipdev.netmask[1], g_lwipdev.netmask[2], g_lwipdev.netmask[3]);
    IP4_ADDR(&gw, g_lwipdev.gateway[0], g_lwipdev.gateway[1], g_lwipdev.gateway[2], g_lwipdev.gateway[3]);
    printf("����en��MAC��ַΪ:................%d.%d.%d.%d.%d.%d\r\n", g_lwipdev.mac[0], g_lwipdev.mac[1], g_lwipdev.mac[2], g_lwipdev.mac[3], g_lwipdev.mac[4], g_lwipdev.mac[5]);
    printf("��̬IP��ַ........................%d.%d.%d.%d\r\n", g_lwipdev.ip[0], g_lwipdev.ip[1], g_lwipdev.ip[2], g_lwipdev.ip[3]);
    printf("��������..........................%d.%d.%d.%d\r\n", g_lwipdev.netmask[0], g_lwipdev.netmask[1], g_lwipdev.netmask[2], g_lwipdev.netmask[3]);
    printf("Ĭ������..........................%d.%d.%d.%d\r\n", g_lwipdev.gateway[0], g_lwipdev.gateway[1], g_lwipdev.gateway[2], g_lwipdev.gateway[3]);
#endif  /* �������б������һ������ */
    netif_init_flag = netif_add(&g_lwip_netif, (const ip_addr_t *)&ipaddr, (const ip_addr_t *)&netmask, (const ip_addr_t *)&gw, NULL, &ethernetif_init, &tcpip_input);

    
    if (netif_init_flag == NULL)
    {
        return 1;                                   /* �������ʧ�� */
    }
    else                                            /* ������ӳɹ���,����netifΪĬ��ֵ,���Ҵ�netif���� */
    {
        netif_set_default(&g_lwip_netif);           /* ����netifΪĬ������ */

#if LWIP_NETIF_LINK_CALLBACK
        lwip_link_status_updated(&g_lwip_netif);    /* DHCP����״̬���º��� */
        netif_set_link_callback(&g_lwip_netif, lwip_link_status_updated);
        /* ��ѯPHY����״̬���� */
        sys_thread_new("eth_link",
                       lwip_link_thread,            /* ������ں��� */
                       &g_lwip_netif,               /* ������ں������� */
                       LWIP_LINK_STK_SIZE,          /* ����ջ��С */
                       LWIP_LINK_TASK_PRIO);        /* ��������ȼ� */
#endif

    }
    g_lwipdev.link_status = LWIP_LINK_OFF;          /* ���ӱ��Ϊ0 */
#if LWIP_DHCP                                       /* ���ʹ��DHCP�Ļ� */
    g_lwipdev.dhcpstatus = 0;                       /* DHCP���Ϊ0 */
    /* DHCP��ѯ���� */
    sys_thread_new("eth_dhcp",
                   lwip_periodic_handle,            /* ������ں��� */
                   &g_lwip_netif,                   /* ������ں������� */
                   LWIP_DHCP_STK_SIZE,              /* ����ջ��С */
                   LWIP_DHCP_TASK_PRIO);            /* ��������ȼ� */
#endif
    return 0;                                       /* ����OK. */
}

/**
 * @brief       ֪ͨ�û�DHCP����״̬
 * @param       netif���������ƿ�
 * @retval      ��
 */
void lwip_link_status_updated(struct netif *netif)
{
    if (netif_is_up(netif))
    {
#if LWIP_DHCP
        /* Update DHCP state machine */
        g_lwip_dhcp_state = LWIP_DHCP_START;
#endif /* LWIP_DHCP */
        printf ("The network cable is connected \r\n");
    }
    else
    {
#if LWIP_DHCP
        /* Update DHCP state machine */
        g_lwip_dhcp_state = LWIP_DHCP_LINK_DOWN;
#endif /* LWIP_DHCP */
        printf ("The network cable is not connected \r\n");
    }
}


#if LWIP_DHCP   /* ���ʹ��DHCP */

/**
 * @breif       DHCP����
 * @param       argument:������β�
 * @retval      ��
 */
void lwip_periodic_handle(void *argument)
{
    struct netif *netif = (struct netif *) argument;
    uint32_t ip = 0;
    uint32_t netmask = 0;
    uint32_t gw = 0;
    struct dhcp *dhcp;
    uint8_t iptxt[20];

    while (1)
    {
        switch (g_lwip_dhcp_state)
        {
            case LWIP_DHCP_START:
            {
                /* ��IP��ַ�����ص�ַ������ҳ��������� */
                ip_addr_set_zero_ip4(&netif->ip_addr);
                ip_addr_set_zero_ip4(&netif->netmask);
                ip_addr_set_zero_ip4(&netif->gw);
                ip_addr_set_zero_ip4(&netif->ip_addr);
                ip_addr_set_zero_ip4(&netif->netmask);
                ip_addr_set_zero_ip4(&netif->gw);
                
                g_lwip_dhcp_state = LWIP_DHCP_WAIT_ADDRESS;
                
                printf ("State: Looking for DHCP server ...\r\n");
                dhcp_start(netif);
            }
            break;
            case LWIP_DHCP_WAIT_ADDRESS:
            {
                if (dhcp_supplied_address(netif))
                {
                    g_lwip_dhcp_state = LWIP_DHCP_ADDRESS_ASSIGNED;
                    
                    ip = g_lwip_netif.ip_addr.addr;       /* ��ȡ��IP��ַ */
                    netmask = g_lwip_netif.netmask.addr;  /* ��ȡ�������� */
                    gw = g_lwip_netif.gw.addr;            /* ��ȡĬ������ */
                    
                    sprintf((char *)iptxt, "%s", ip4addr_ntoa(netif_ip4_addr(netif)));
                    printf ("IP address assigned by a DHCP server: %s\r\n", iptxt);
                    
                    if (ip != 0)
                    {
                        g_lwipdev.dhcpstatus = 2;         /* DHCP�ɹ� */
                        printf("����en��MAC��ַΪ:................%d.%d.%d.%d.%d.%d\r\n", g_lwipdev.mac[0], g_lwipdev.mac[1], g_lwipdev.mac[2], g_lwipdev.mac[3], g_lwipdev.mac[4], g_lwipdev.mac[5]);
                        /* ������ͨ��DHCP��ȡ����IP��ַ */
                        g_lwipdev.ip[3] = (uint8_t)(ip >> 24);
                        g_lwipdev.ip[2] = (uint8_t)(ip >> 16);
                        g_lwipdev.ip[1] = (uint8_t)(ip >> 8);
                        g_lwipdev.ip[0] = (uint8_t)(ip);
                        printf("ͨ��DHCP��ȡ��IP��ַ..............%d.%d.%d.%d\r\n", g_lwipdev.ip[0], g_lwipdev.ip[1], g_lwipdev.ip[2], g_lwipdev.ip[3]);
                        /* ����ͨ��DHCP��ȡ�������������ַ */
                        g_lwipdev.netmask[3] = (uint8_t)(netmask >> 24);
                        g_lwipdev.netmask[2] = (uint8_t)(netmask >> 16);
                        g_lwipdev.netmask[1] = (uint8_t)(netmask >> 8);
                        g_lwipdev.netmask[0] = (uint8_t)(netmask);
                        printf("ͨ��DHCP��ȡ����������............%d.%d.%d.%d\r\n", g_lwipdev.netmask[0], g_lwipdev.netmask[1], g_lwipdev.netmask[2], g_lwipdev.netmask[3]);
                        /* ������ͨ��DHCP��ȡ����Ĭ������ */
                        g_lwipdev.gateway[3] = (uint8_t)(gw >> 24);
                        g_lwipdev.gateway[2] = (uint8_t)(gw >> 16);
                        g_lwipdev.gateway[1] = (uint8_t)(gw >> 8);
                        g_lwipdev.gateway[0] = (uint8_t)(gw);
                        printf("ͨ��DHCP��ȡ����Ĭ������..........%d.%d.%d.%d\r\n", g_lwipdev.gateway[0], g_lwipdev.gateway[1], g_lwipdev.gateway[2], g_lwipdev.gateway[3]);
                        
                        g_lwipdev.lwip_display_fn(2);
                    }
                }
                else
                {
                    dhcp = (struct dhcp *)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);

                    /* DHCP timeout */
                    if (dhcp->tries > LWIP_MAX_DHCP_TRIES)
                    {
                        g_lwip_dhcp_state = LWIP_DHCP_TIMEOUT;
                        g_lwipdev.dhcpstatus = 0XFF;
                        /* ʹ�þ�̬IP��ַ */
                        IP4_ADDR(&(g_lwip_netif.ip_addr), g_lwipdev.ip[0], g_lwipdev.ip[1], g_lwipdev.ip[2], g_lwipdev.ip[3]);
                        IP4_ADDR(&(g_lwip_netif.netmask), g_lwipdev.netmask[0], g_lwipdev.netmask[1], g_lwipdev.netmask[2], g_lwipdev.netmask[3]);
                        IP4_ADDR(&(g_lwip_netif.gw), g_lwipdev.gateway[0], g_lwipdev.gateway[1], g_lwipdev.gateway[2], g_lwipdev.gateway[3]);
                        netif_set_addr(netif, &g_lwip_netif.ip_addr, &g_lwip_netif.netmask, &g_lwip_netif.gw);

                        sprintf((char *)iptxt, "%s", ip4addr_ntoa(netif_ip4_addr(netif)));
                        printf ("DHCP Timeout !! \r\n");
                        printf ("Static IP address: %s\r\n", iptxt);
                        
                        g_lwipdev.lwip_display_fn(2);
                    }
                }
            }
            break;
            case LWIP_DHCP_LINK_DOWN:
            {
                g_lwip_dhcp_state = LWIP_DHCP_OFF;
            }
            break;
            default: break;
        }

        /* wait 1000 ms */
        vTaskDelay(1000);
    }
}
#endif

/**
  * @brief       ���ETH��·״̬������netif
  * @param       argument: netif
  * @retval      ��
  */
void lwip_link_thread( void * argument )
{
    uint32_t regval = 0;
    struct netif *netif = (struct netif *) argument;
    int link_again_num = 0;

    g_lwipdev.dhcpstatus = 0XFF;
    while(1)
    {
        /* ��ȡPHY״̬�Ĵ�������ȡ������Ϣ */
        HAL_ETH_ReadPHYRegister(&g_eth_handler, ETH_CHIP_ADDR,ETH_CHIP_BSR, &regval);

        /* �ж�����״̬ */
        if((regval & ETH_CHIP_BSR_LINK_STATUS) == 0)
        {
            g_lwipdev.link_status = LWIP_LINK_OFF;
            
            link_again_num ++ ;
            
            if (link_again_num >= 2)                    /* ����һ��ʱ��û�в��� */
            {
                continue;
            }
            else                                        /* �ر�������������̫���ж� */
            {
#if LWIP_DHCP
                g_lwip_dhcp_state = LWIP_DHCP_LINK_DOWN;
                dhcp_stop(netif);
#endif
                HAL_ETH_Stop_IT(&g_eth_handler);
                netif_set_down(netif);
                netif_set_link_down(netif);
            }
        }
        else                                            /* ���߲����� */
        {
            link_again_num = 0;

            if (g_lwipdev.link_status == LWIP_LINK_OFF)/* ������̫������������ */
            {
                g_lwipdev.link_status = LWIP_LINK_ON;
                HAL_ETH_Start_IT(&g_eth_handler);
                netif_set_up(netif);
                netif_set_link_up(netif);
            }
        }

        vTaskDelay(100);
    }
}
