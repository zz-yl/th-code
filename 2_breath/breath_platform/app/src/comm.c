/**@file   comm.c
* @brief   通信模块
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "comm.h"
#include "crc_check.h"
#include "thb_cfg.h"
#include "motor.h"
#include "valve.h"

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

uint8_t comm_motor1_tx_data[COMM_QUEUE_SIZE] = {0};
uint8_t comm_motor1_rx_data[COMM_QUEUE_SIZE] = {0};
comm_data_t comm_motor1 = {0};

comm_cmd_t comm_cmd;

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
* @brief  串口接收回调函数
* @attention 
*/
void uart1_receive(uint8_t *data, uint16_t len)
{
#ifndef COMM_TT
    queue_input_u8(&comm_ctrl.rx, data, len);  //数据入队
#else
    queue_input_u8(&comm_wifi.tx, data, len);
#endif
}

///**
//* @brief  串口接收回调函数
//* @attention 
//*/
//void uart2_receive(uint8_t *data, uint16_t len)
//{
//#ifndef COMM_TT
//    queue_input_u8(&comm_wifi.rx, data, len);  //数据入队
//#else
//    queue_input_u8(&comm_ctrl.tx, data, len);
//#endif
//}
/**
* @brief  上位机接收数据处理
* @attention 
*/
void comm_rx(void)
{
    uint8_t data_buf[COMM_QUEUE_SIZE] = {0};
    comm_data_t *qp = &comm_ctrl;
    uint16_t len = 0;
    uint16_t data = 0;
    uint16_t cnt = 1;
    
    len = queue_length(&qp->rx);
    
    if(len)
    {
        queue_output_u8(&qp->rx, data_buf, len);
        
        if((data_buf[0] != 's') && (data_buf[0] != 'f'))
        {
            return;
        }  
        if(data_buf[1] != ' ')
        {
            return;
        }
        len--;
        if(data_buf[len--] != 0x0A)
        {
            return;
        }
        if(data_buf[len--] != 0x0D)
        {
            return;
        }
        data += data_buf[len--] - '0';
        cnt *= 10;
        while(len > 1)
        {
            data += (data_buf[len--] - '0') * cnt;
            cnt *= 10;
        }
        if(data_buf[0] == 's')
        {
            if(data > 55000)
            {
                data = 55000;
            }
            comm_cmd.type = COMM_SPEED;
            motor_data.cmd = data;
        }
        else if(data_buf[0] == 'f')
        {
            if(data > 1000)
            {
                data = 1000;
            }
            comm_cmd.type = COMM_FLOW;
            comm_cmd.flow = data;
        }
    }
}
/**
* @brief  发送电机速度
* @attention 
*/
void comm_motor_speed(float speed)
{
    uint8_t data_buf[20] = {0};
    uint16_t speed_buf = 0;
    
    if((speed < 0) || (speed > 55000))
    {
        speed = 0;
    }
    if(speed > 45000)  //39000时电流约8.5A
    {
        speed = 45000;
    }
    
    speed_buf = (speed / 55000) * 0xFFF;
    
    data_buf[0] = 0x40 | ((speed_buf >> 8) & 0x0F);
    data_buf[1] = 0x80 | ((speed_buf >> 4) & 0x0F);
    data_buf[2] = 0xC0 | (speed_buf & 0x0F);
    
    queue_input_u8(&comm_motor1.tx, data_buf, 3);
}
/**
* @brief  电机串口数据发送
* @attention 
*/
static uint16_t comm_motor_tx(void)
{
    uint8_t data_buf[COMM_QUEUE_SIZE] = {0};
    uint16_t len = 0;
    
    len = queue_length(&comm_motor1.tx);
    if(len == 0)  //队列空
    {
        return 0;
    }
    len = queue_read_u8(&comm_motor1.tx, data_buf, len);  //读取数据
    len = uart3_send(data_buf, len);  //数据写入DMA
    uart2_send(data_buf, len);  //数据写入DMA,电机2
    queue_delete(&comm_motor1.tx, len); //队列中删除已发送数据
    
    return len;
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
    len = queue_read_u8(&comm_ctrl.tx, data_buf, len);  //读取队列
    len = uart1_send(data_buf, len);  //数据写入DMA
    queue_delete(&comm_ctrl.tx, len); //队列中删除已发送数据
    
    return len;
}
/**
* @brief  通信模块运行
* @attention 
*/
void comm_run(void)
{
    comm_rx();
    comm_tx();
    comm_motor_tx();
}
/**
* @brief  通信模块初始化
* @attention 
*/
void comm_init(void)
{
    queue_init(&comm_ctrl.tx, comm_ctrl_tx_data, COMM_QUEUE_SIZE);
    queue_init(&comm_ctrl.rx, comm_ctrl_rx_data, COMM_QUEUE_SIZE);
    queue_init(&comm_motor1.tx, comm_motor1_tx_data, COMM_QUEUE_SIZE);
    queue_init(&comm_motor1.rx, comm_motor1_rx_data, COMM_QUEUE_SIZE);
}
