/**@file   comm.c
* @brief   通信模块
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "comm.h"
#include "comm_cmd.h"
#include "comm_msg.h"
#include "crc_check.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

uint8_t comm_ctrl_tx_data[COMM_QUEUE_SIZE] = {0};
uint8_t comm_ctrl_rx_data[COMM_QUEUE_SIZE] = {0};
comm_data_t comm_ctrl = {0};

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    queue_input_u8(&comm_ctrl.tx, (uint8_t *)&ch, 1);
    return ch;
}
/**
* @brief  串口8接收回调函数
* @attention 
*/
void uart8_receive(uint8_t *data, uint16_t len)
{
    queue_input_u8(&comm_ctrl.rx, data, len);  //数据入队
}
/**
* @brief  发送给上位机数据写入发送缓存
* @attention 
*/
void comm_fill_msg(uint8_t cmd, uint8_t *data, uint8_t len)
{
    uint8_t data_buf[COMM_QUEUE_SIZE] = {0};
    uint16_t cnt = 0;
    uint8_t crc8 = 0;
    
    data_buf[cnt++] = (uint8_t)COMM_ADDR_CTRL;
    data_buf[cnt++] = cmd;
    data_buf[cnt++] = len;
    
    while(len--)
    {
        data_buf[cnt++] = *(data++);
    }
    crc8 = crc_get_crc8(data_buf, cnt, 0xFF);
    data_buf[cnt++] = crc8;
    
    queue_input_u8(&comm_ctrl.tx, data_buf, cnt);
}
/**
* @brief  上位机串口数据发送
* @attention 
*/
static uint16_t comm_tx(void)
{
    uint8_t data_buf[COMM_QUEUE_SIZE] = {0};
    uint16_t len = 0;
    
    len = queue_length(&comm_ctrl.tx);
    if(len == 0)  //队列空
    {
        return 0;
    }
    len = queue_read_u8(&comm_ctrl.tx, data_buf, len);  //读取数据
    len = uart8_send(data_buf, len);  //数据写入DMA
    queue_delete(&comm_ctrl.tx, len); //队列中删除已发送数据
    
    return len;
}
/**
* @brief  通信模块运行
* @attention 
*/
void comm_run(void)
{
//    comm_rx(&comm_ctrl, COMM_ADDR_DEVICE);
    comm_tx();
}
/**
* @brief  通信模块初始化
* @attention 
*/
void comm_init(void)
{
    queue_init(&comm_ctrl.tx, comm_ctrl_tx_data, COMM_QUEUE_SIZE);
    queue_init(&comm_ctrl.rx, comm_ctrl_rx_data, COMM_QUEUE_SIZE);
}
