 
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdint.h>
#include <stdio.h>
#include <lwip/sockets.h>
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip_demo.h"

#include "th_cfg.h"
#include "./BSP/DAC/dac.h"


/* 设置远程IP地址 */
//#define DEST_IP_ADDR0               192
//#define DEST_IP_ADDR1               168
//#define DEST_IP_ADDR2                 1
//#define DEST_IP_ADDR3               167

/* 需要自己设置远程IP地址 */
//#define IP_ADDR   "192.168.1.167"

#define LWIP_DEMO_RX_BUFSIZE         200                        /* 最大接收数据长度 */
#define LWIP_DEMO_PORT               18000//8080                       /* 连接的本地端口号 */
#define LWIP_SEND_THREAD_PRIO       ( tskIDLE_PRIORITY + 3 )    /* 发送数据线程优先级 */

/* 接收数据缓冲区 */
uint8_t g_lwip_demo_recvbuf[LWIP_DEMO_RX_BUFSIZE];  
/* 发送数据内容 */
uint8_t g_lwip_demo_sendbuf[] = "ALIENTEK DATA \r\n";

/* 数据发送标志位 */
uint8_t g_lwip_send_flag;
int g_sock_conn;                          /* 请求的 socked */
int g_lwip_connect_state = 0;
static void lwip_send_thread(void *arg);
extern QueueHandle_t g_display_queue;     /* 显示消息队列句柄 */

/**
 * @brief       发送数据线程
 * @param       无
 * @retval      无
 */
void lwip_data_send(void)
{
    sys_thread_new("lwip_send_thread", lwip_send_thread, NULL, 512, LWIP_SEND_THREAD_PRIO );
}
/**
* @brief 发送处理
* @attention 
*/
static void comm_tx(void)
{
    if(g_lwip_connect_state) /* 连接成功 */
    {
        comm_msg.footrest1 = IO_FOOTREST1;
        comm_msg.footrest2 = IO_FOOTREST2;
        comm_msg.tim = sys_timer;
        send(g_sock_conn, (unsigned char *)(&comm_msg), sizeof(comm_msg), 0); /* 发送数据 */
        comm_msg.frame_number++;
    }
}
/**
* @brief 接收处理
* @attention 
*/
static void comm_rx(void)
{
    comm_msg.state_spectrometer = comm_cmd.spectrometer;
    comm_msg.state_halogen      = comm_cmd.halogen;
    comm_msg.state_led          = comm_cmd.led;
    comm_msg.state_led_level    = comm_cmd.led_level;
    
    IO_OUT1(comm_msg.state_led);
    IO_OUT2(comm_msg.state_halogen);
    IO_OUT3(comm_msg.state_spectrometer);
    dac_set_voltage(1, comm_cmd.led_level);
}
/**
 * @brief       lwip_demo实验入口
 * @param       无
 * @retval      无
 */
void lwip_demo()
{
    struct sockaddr_in server_addr; /* 服务器地址 */
    struct sockaddr_in conn_addr;   /* 连接地址 */
    socklen_t addr_len;             /* 地址长度 */
    int err;
    int length;
    int sock_fd;
    BaseType_t lwip_err;
    lwip_data_send();                                    /* 创建一个发送线程 */
    
    sock_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); /* 建立一个新的socket连接 */
    memset(&server_addr, 0, sizeof(server_addr));        /* 将服务器地址清空 */
    server_addr.sin_family = AF_INET;                    /* 地址家族 */
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);     /* 注意转化为网络字节序 */
    server_addr.sin_port = htons(LWIP_DEMO_PORT);        /* 使用SERVER_PORT指定为程序头设定的端口号 */

//    sprintf((char *)tbuf, "Port:%d", LWIP_DEMO_PORT); /* 客户端端口号 */
    
    err = bind(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)); /* 建立绑定 */

    if (err < 0)                /* 如果绑定失败则关闭套接字 */
    {
        closesocket(sock_fd);   /* 关闭套接字 */
    }

    err = listen(sock_fd, 4);   /* 监听连接请求 */

    if (err < 0)                /* 如果监听失败则关闭套接字 */
    {
        closesocket(sock_fd);   /* 关闭套接字 */
    }
    
    while(1)
    {
        g_lwip_connect_state = 0;
        addr_len = sizeof(struct sockaddr_in); /* 将链接地址赋值给addr_len */

        g_sock_conn = accept(sock_fd, (struct sockaddr *)&conn_addr, &addr_len); /* 对监听到的请求进行连接，状态赋值给g_sock_conn */

        if (g_sock_conn < 0) /* 状态小于0代表连接故障，此时关闭套接字 */
        {
            closesocket(sock_fd);
        }
        else 
        {
//            lcd_show_string(5, 90, 200, 16, 16, "State:Connection Successful", BLUE);
            g_lwip_connect_state = 1;
        }

        while (1)
        {
//            memset(g_lwip_demo_recvbuf,0,LWIP_DEMO_RX_BUFSIZE);
//            length = recv(g_sock_conn, (unsigned int *)g_lwip_demo_recvbuf, sizeof(g_lwip_demo_recvbuf), 0); /* 将收到的数据放到接收Buff */
            length = recv(g_sock_conn, (unsigned char *)(&comm_cmd), sizeof(comm_cmd), 0); /* 将收到的数据放到接收Buff */
            comm_rx();
//            comm_tx();
            
            if (length <= 0)
            {
                goto atk_exit;
            }
            
//            printf("%s",g_lwip_demo_recvbuf);
//            lwip_err = xQueueSend(g_display_queue,&g_lwip_demo_recvbuf,0);

//            if (lwip_err == errQUEUE_FULL)
//            {
//                printf("队列Key_Queue已满，数据发送失败!\r\n");
//            }
        }
atk_exit:
        if (g_sock_conn >= 0)
        {          
            closesocket(g_sock_conn);
            g_sock_conn = -1;
//            lcd_fill(5, 89, lcddev.width,110, WHITE);
//            lcd_show_string(5, 90, 200, 16, 16, "State:Disconnect", BLUE);
//            myfree(SRAMIN, tbuf);
        }
        
    }
}
/**
 * @brief       发送数据线程函数
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
static void lwip_send_thread(void *pvParameters)
{
    pvParameters = pvParameters;
    
    while (1)
    {
//        if (((g_lwip_send_flag & LWIP_SEND_DATA) == LWIP_SEND_DATA) && (g_lwip_connect_state == 1)) /* 有数据要发送 */
//        {
//            send(g_sock_conn, g_lwip_demo_sendbuf, sizeof(g_lwip_demo_sendbuf), 0); /* 发送数据 */
//            g_lwip_send_flag &= ~LWIP_SEND_DATA;
//        }
        comm_tx();

        vTaskDelay(10);
    }
}
