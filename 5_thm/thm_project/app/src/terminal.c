/**@file   terminal.c
* @brief   终端
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "terminal.h"
#include "stdio.h"
#include "string.h"
#include "bsp_cfg.h"
#include "data_queue.h"
#include "test.h"

#include "control.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define TER_TX_SIZE     2000
#define TER_RX_SIZE     64

/**
* @enum    terminal_page_e
* @brief   页码
*/
typedef enum
{
    TERMINAL_PAGEA,
    TERMINAL_PAGEB,
    TERMINAL_PAGEC,
    TERMINAL_PAGED,
}terminal_page_t;

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  terminal_dis_t
* @brief   显示页面
*/
typedef struct
{
    terminal_page_t page;
    void (*init)(void);
    void (*work)(void);
}terminal_dis_t;

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

uint8_t ter_tx_buf[TER_TX_SIZE];
uint8_t ter_rx_buf[TER_RX_SIZE];

/**
* @struct  ter_data
* @brief   终端模块数据
*/
struct
{
    queue_list_t tx;
    queue_list_t rx;
    uint16_t up;
    uint8_t state;
    uint8_t page;
    uint8_t object;
    uint8_t object_last;
    uint8_t cmd;
    uint32_t cmd_data;
}ter_data;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/*串口printf配置*************************************************************************************************/
/**
* @brief  重定义fputc函数
*/
int fputc(int ch, FILE *f)
{
    queue_input_u8_single(&ter_data.tx, ch);
    
    return ch;
}
/**
* @brief  数据接收函数重定义
*/
void uart8_receive(uint8_t *data, uint16_t len)
{
    queue_input_u8(&ter_data.rx, data, len);
}
/**
* @brief  printf运行,数据发送
*/
void printf_run(void)
{
    uart8_send_q(&ter_data.tx);
}
/**
* @brief  页面刷新
*/
static void terminal_refresh(void)
{
    uint16_t i = 0;
    
    if(ter_data.state == 0)
    {
        ter_data.state = 1;
        
        i = 160;
        while(i)
        {
            printf("\r\n");
            i--;
        }
        i = 80;
        while(i)
        {
            printf("\x1B\x5B\x41");
            i--;
        }
    }
    while(ter_data.up)
    {
        printf("\x1B\x5B\x41");
        ter_data.up--;
    }
//    if(ter_data.page != TERMINAL_PAGEA)
//    {
        printf("\r");
//    }
}
/**
* @brief  PageA初始化
*/
static void ter_page_a_init(void)
{
    
//    printf("L1 (b)----L2 (c)----L3 (d)----L4 (e)----L5 (f)----L6 (g)----L7 (h)----L8 (i)----L9 (j)---- \r\n");
//    printf("L10(k)----L11(l)----L12(m)----L13(n)----L14(o)----L15(p)----L16(q)----L17(r)----L18(s)---- \r\n");
    
    ter_data.object = 'b';
}
/**
* @brief  PageA显示
*/
static void ter_page_a(void)
{
    printf("操作说明:→:下一项, ←:上一项, a:统一控制, b~s:选择1~18号led单独控制, ↑↓:led使能 \r\n"); ter_data.up++;
    printf("输入数字控制频率,退格删除,0代表常亮,频率范围0~30000Hz,使能后led才会亮 \r\n"); ter_data.up++;
    printf("定时器总频率30000,记数周期=30000/频率,遇小数舍弃会导致某些频率值会偏高 \r\n"); ter_data.up++;
    printf("KEY1:改变所有led使能, KEY4:保存当前状态 \r\n"); ter_data.up++;
    printf("编号  使能 频率 \r\n"); ter_data.up++;
    printf("N1(b)->%1d,%5d;F1(c)->%1d,%5d;L1(d)->%1d,%5d;N2(e)->%1d,%5d;L2(f)->%1d,%5d;N3(g)->%1d,%5d \r\n"
            , led_data.state[0], led_data.frq[0], led_data.state[1], led_data.frq[1], led_data.state[2], led_data.frq[2]
            , led_data.state[3], led_data.frq[3], led_data.state[4], led_data.frq[4], led_data.state[5], led_data.frq[5]); ter_data.up++;
    printf("N4(h)->%1d,%5d;F2(i)->%1d,%5d;L3(j)->%1d,%5d;N5(k)->%1d,%5d;N6(l)->%1d,%5d;F3(m)->%1d,%5d \r\n"
            , led_data.state[6], led_data.frq[6], led_data.state[7], led_data.frq[7], led_data.state[8], led_data.frq[8]
            , led_data.state[9], led_data.frq[9], led_data.state[10], led_data.frq[10], led_data.state[11], led_data.frq[11]); ter_data.up++;
    printf("N7(n)->%1d,%5d;F4(o)->%1d,%5d;L4(p)->%1d,%5d;N8(q)->%1d,%5d;F5(r)->%1d,%5d;N9(s)->%1d,%5d \r\n"
            , led_data.state[12], led_data.frq[12], led_data.state[13], led_data.frq[13], led_data.state[14], led_data.frq[14]
            , led_data.state[15], led_data.frq[15], led_data.state[16], led_data.frq[16], led_data.state[17], led_data.frq[17]); ter_data.up++;
    printf("输入指令: \r\n"); ter_data.up++;
    printf("编号:%c,↑↓使能:%1d,频率:%5d \r\n", ter_data.object, ter_data.cmd, ter_data.cmd_data); ter_data.up++;
}
/**
* @brief  PageB显示
*/
static void ter_page_b(void)
{
    
}
/**
* @brief  PageC初始化
*/
static void ter_page_c_init(void)
{
    
}
/**
* @brief  PageC显示
*/
static void ter_page_c(void)
{

}
/**
* @brief  PageD显示
*/
static void ter_page_d(void)
{
    
}

/**
* @struct  ter_page
* @brief   页面
*/
terminal_dis_t ter_page[] =
{
    {.page = TERMINAL_PAGEA, .init = ter_page_a_init, .work = ter_page_a,},
    {.page = TERMINAL_PAGEB, .work = ter_page_b,},
    {.page = TERMINAL_PAGEC, .init = ter_page_c_init, .work = ter_page_c},
    {.page = TERMINAL_PAGED, .work = ter_page_d,},
    
    {.work = NULL,}
};
/**
* @brief  换项处理
*/
static void ter_change(void)
{
    if(ter_data.object_last == ter_data.object)
    {
        return;
    }
    else
    {
        ter_data.object_last = ter_data.object;
    }
    switch(ter_data.object)
    {
        case 'a': ter_data.cmd = led_data.state[0];  ter_data.cmd_data = led_data.frq[0];  break;
        case 'b': ter_data.cmd = led_data.state[0];  ter_data.cmd_data = led_data.frq[0];  break;
        case 'c': ter_data.cmd = led_data.state[1];  ter_data.cmd_data = led_data.frq[1];  break;
        case 'd': ter_data.cmd = led_data.state[2];  ter_data.cmd_data = led_data.frq[2];  break;
        case 'e': ter_data.cmd = led_data.state[3];  ter_data.cmd_data = led_data.frq[3];  break;
        case 'f': ter_data.cmd = led_data.state[4];  ter_data.cmd_data = led_data.frq[4];  break;
        case 'g': ter_data.cmd = led_data.state[5];  ter_data.cmd_data = led_data.frq[5];  break;
        case 'h': ter_data.cmd = led_data.state[6];  ter_data.cmd_data = led_data.frq[6];  break;
        case 'i': ter_data.cmd = led_data.state[7];  ter_data.cmd_data = led_data.frq[7];  break;
        case 'j': ter_data.cmd = led_data.state[8];  ter_data.cmd_data = led_data.frq[8];  break;
        case 'k': ter_data.cmd = led_data.state[9];  ter_data.cmd_data = led_data.frq[9];  break;
        case 'l': ter_data.cmd = led_data.state[10]; ter_data.cmd_data = led_data.frq[10]; break;
        case 'm': ter_data.cmd = led_data.state[11]; ter_data.cmd_data = led_data.frq[11]; break;
        case 'n': ter_data.cmd = led_data.state[12]; ter_data.cmd_data = led_data.frq[12]; break;
        case 'o': ter_data.cmd = led_data.state[13]; ter_data.cmd_data = led_data.frq[13]; break;
        case 'p': ter_data.cmd = led_data.state[14]; ter_data.cmd_data = led_data.frq[14]; break;
        case 'q': ter_data.cmd = led_data.state[15]; ter_data.cmd_data = led_data.frq[15]; break;
        case 'r': ter_data.cmd = led_data.state[16]; ter_data.cmd_data = led_data.frq[16]; break;
        case 's': ter_data.cmd = led_data.state[17]; ter_data.cmd_data = led_data.frq[17]; break;
        default: break;
    }
}
/**
* @brief  led控制
*/
static void ter_ctrl_led(uint8_t data)
{
    switch(ter_data.object)
    {
        case 'a': led_ctrl_all(ter_data.cmd, ter_data.cmd_data); break;
        case 'b': led_data.state[0] = ter_data.cmd;  led_data.frq[0]  = ter_data.cmd_data; break;
        case 'c': led_data.state[1] = ter_data.cmd;  led_data.frq[1]  = ter_data.cmd_data; break;
        case 'd': led_data.state[2] = ter_data.cmd;  led_data.frq[2]  = ter_data.cmd_data; break;
        case 'e': led_data.state[3] = ter_data.cmd;  led_data.frq[3]  = ter_data.cmd_data; break;
        case 'f': led_data.state[4] = ter_data.cmd;  led_data.frq[4]  = ter_data.cmd_data; break;
        case 'g': led_data.state[5] = ter_data.cmd;  led_data.frq[5]  = ter_data.cmd_data; break;
        case 'h': led_data.state[6] = ter_data.cmd;  led_data.frq[6]  = ter_data.cmd_data; break;
        case 'i': led_data.state[7] = ter_data.cmd;  led_data.frq[7]  = ter_data.cmd_data; break;
        case 'j': led_data.state[8] = ter_data.cmd;  led_data.frq[8]  = ter_data.cmd_data; break;
        case 'k': led_data.state[9] = ter_data.cmd;  led_data.frq[9]  = ter_data.cmd_data; break;
        case 'l': led_data.state[10] = ter_data.cmd; led_data.frq[10] = ter_data.cmd_data; break;
        case 'm': led_data.state[11] = ter_data.cmd; led_data.frq[11] = ter_data.cmd_data; break;
        case 'n': led_data.state[12] = ter_data.cmd; led_data.frq[12] = ter_data.cmd_data; break;
        case 'o': led_data.state[13] = ter_data.cmd; led_data.frq[13] = ter_data.cmd_data; break;
        case 'p': led_data.state[14] = ter_data.cmd; led_data.frq[14] = ter_data.cmd_data; break;
        case 'q': led_data.state[15] = ter_data.cmd; led_data.frq[15] = ter_data.cmd_data; break;
        case 'r': led_data.state[16] = ter_data.cmd; led_data.frq[16] = ter_data.cmd_data; break;
        case 's': led_data.state[17] = ter_data.cmd; led_data.frq[17] = ter_data.cmd_data; break;
        default: break;
    }
}

/**
* @brief  接收数据处理
*/
void terminal_cmd(void)
{
    uint8_t data = 0;
    
    if(queue_length(&ter_data.rx))
    {
        while(queue_output_u8_single(&ter_data.rx, &data) == QUEUE_SUCCESS)
        {
            if(data == 0x1B)
            {
                queue_output_u8_single(&ter_data.rx, &data);
                if(data != 0x5B)
                {
                    return;
                }
                queue_output_u8_single(&ter_data.rx, &data);
                switch(data)
                {
                    case 0x41: //↑
                        ter_data.cmd = 1;
                        break;
                    case 0x42: //↓
                        ter_data.cmd = 0;
                        break;
                    case 0x43: //→
                        if(ter_data.object < 's')
                        {
                            ter_data.object++;
                        }
                        break;
                    case 0x44: //←
                        if(ter_data.object > 'a')
                        {
                            ter_data.object--;
                        }
                        break;
                    default: break;
                }
            }
            if((data >= 'a') && (data <= 's'))
            {
                ter_data.object = data;
            }
            ter_change();
            if((data >= '0') && (data <= '9'))
            {
                ter_data.cmd_data = ter_data.cmd_data * 10 + (data - '0');
                if(ter_data.cmd_data > FRQ_TIMER)
                {
                    ter_data.cmd_data = FRQ_TIMER;
                }
            }
            if(data == 0x08)
            {
                ter_data.cmd_data /= 10;
            }
            
            switch(ter_data.page)
            {
                case TERMINAL_PAGEA: ter_ctrl_led(data); break;
                case TERMINAL_PAGEB: break;
                case TERMINAL_PAGEC: break;
                case TERMINAL_PAGED: break;
                default: break;
            }
        }
    }
}
/**
* @brief  终端初始化
*/
void terminal_init(void)
{
    queue_init(&ter_data.tx, ter_tx_buf, TER_TX_SIZE);
    queue_init(&ter_data.rx, ter_rx_buf, TER_RX_SIZE);
}
/**
* @brief  终端运行
*/
void terminal_run(void)
{
    static uint16_t tim = 0;  //计时
    static uint8_t page_last = 1;  //上次的页码

    if(page_last != ter_data.page)
    {
        page_last = ter_data.page;
        
        queue_clear(&ter_data.tx);
        terminal_refresh();
        if(ter_page[ter_data.page].init != NULL)
        {
            ter_page[ter_data.page].init();
        }
    }
    
    tim += 2;
    if(tim > 200)
    {
        tim = 0;
        
        terminal_refresh();
        if(ter_page[ter_data.page].work != NULL)
        {
            ter_page[ter_data.page].work();
        }
    }
    terminal_cmd();

    printf_run();
}
