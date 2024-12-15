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
#include "memory.h"

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
uint32_t comm_len_data = 0;  //临时用作上位机存储长数据处理

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
    if((data[0] == COMM_ADDR_DEVICE) && (data[1] == CMD_WRITE_DATA))  //上位机存储数据做特殊处理
    {
        memcpy(mem_data.buf, data, len);
        comm_len_data = len;
    }
    else
    {
        queue_input_u8(&comm_ctrl.rx, data, len);  //数据入队
    }
}
/**
* @brief  接收存储数据处理
* @attention 
*/
static void comm_rx_data(void)
{
    uint8_t cmd = 0;
    uint8_t crc8 = 0;
    /* 有数据 */
    if(comm_len_data)
    {
        cmd = mem_data.buf[1];
        crc8 = crc_get_crc8(mem_data.buf, comm_len_data-1, 0xFF);
        if(crc8 != mem_data.buf[comm_len_data-1])
        {
            return;
        }
        
        comm_cmd(cmd, &mem_data.buf[2]);
        comm_len_data = 0;
    }
}
/**
* @brief  接收数据处理
* @attention 
*/
static void comm_rx(comm_data_t *qp, uint8_t addr)
{
    uint8_t data_buf[COMM_QUEUE_SIZE] = {0};
    uint16_t count = 0;
    uint8_t cmd = 0;
    uint8_t len = 0;
    uint8_t crc8 = 0;
    /* 遍历队列 */
    while(queue_length(&qp->rx) != 0)  
    {
        count += queue_output_u8(&qp->rx, &data_buf[count], 1);  //地址
        if(data_buf[0] != addr)
        {
            return;
        }
        count += queue_output_u8(&qp->rx, &data_buf[count], 1);  //指令
        cmd = data_buf[count-1];
        count += queue_output_u8(&qp->rx, &data_buf[count], 1);  //长度
        len = data_buf[count-1];
        if(queue_length(&qp->rx) < len)
        {
            return;
        }
        count += queue_output_u8(&qp->rx, &data_buf[count], len);  //数据
        crc8 = crc_get_crc8(data_buf, count, 0xFF);

        queue_output_u8(&qp->rx, &data_buf[count], 1);  //校验
        if(crc8 != data_buf[count])
        {
            return;
        }
        
        comm_cmd(cmd, &data_buf[2]);
    }
}
uint8_t comm_send_data[5000] = {0};
/**
* @brief  长数据发送
* @attention 
*/
void comm_long_msg_tx(uint8_t cmd, uint8_t *data, uint16_t len)
{
    uint16_t cnt = 0;
    uint8_t crc8 = 0;
    
    if(len == 0)
    {
        return;
    }
    if(__HAL_UART_GET_FLAG(&uart8_handle.uart, USART_FLAG_TC) != SET)  //上次发送未完成
    {
        return;
    }
    if(len > 2400)  //长度超出DMA缓存区
    {
        len = 2400;
    }
    
    comm_send_data[cnt++] = (uint8_t)COMM_ADDR_CTRL;
    comm_send_data[cnt++] = cmd;
    comm_send_data[cnt++] = 0;
    
    memcpy(&(comm_send_data[cnt]), data, len);
    cnt += len;
    crc8 = crc_get_crc8(comm_send_data, cnt, 0xFF);
    comm_send_data[cnt++] = crc8;

    HAL_UART_Transmit_DMA(&uart8_handle.uart, comm_send_data, cnt);
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
    comm_rx(&comm_ctrl, COMM_ADDR_DEVICE);
    comm_rx_data();
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
