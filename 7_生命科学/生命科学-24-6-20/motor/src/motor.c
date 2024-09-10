/**@file   motor.c
* @brief   485电机控制
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "motor.h"
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

uint8_t motor_comm_tx_data[COMM_QUEUE_SIZE] = {0};
uint8_t motor_comm_rx_data[COMM_QUEUE_SIZE] = {0};
comm_data_t motor_comm;
motor_data_t motor_data;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
//int fputc(int ch, FILE *f)
//{
//    queue_input_u8(&comm_ctrl.tx, (uint8_t *)&ch, 1);
//    return ch;
//}

/**
* @brief  串口接收回调函数
* @attention 
*/
void uart2_receive(uint8_t *data, uint16_t len)
{
    queue_input_u8(&motor_comm.rx, data, len);  //数据入队
}
/**
* @brief  接收数据处理
* @attention 
*/
static void motor_get_data(uint8_t *data)
{
    float fbuf = 0;

    if(data[0] == 0xFF)
    {
        motor_data.fb_en = 1;
    }
    else
    {
        motor_data.fb_en = 0;
    }
    fbuf = (data[4] << 8) + data[5];
    fbuf /= 10;
    motor_data.fb_cmd_speed = fbuf;
    motor_data.fb_alarm.all = data[7];
    fbuf = (data[8] << 8) + data[9];
    fbuf /= 10;
    motor_data.fb_speed = fbuf;
}
/**
* @brief  接收数据处理
* @attention 
*/
static void motor_comm_rx(uint8_t addr)
{
    comm_data_t *qp = &motor_comm;
    uint8_t data_buf[COMM_QUEUE_SIZE] = {0};
    uint16_t count = 0;
    uint8_t cmd = 0;
    uint8_t len = 0;
    uint16_t crc16 = 0;
    /* 遍历队列 */
    while(queue_length(&qp->rx) != 0)  
    {
        count += queue_output_u8(&qp->rx, &data_buf[count], 1);  //地址
        if(data_buf[0] != addr)
        {
            return;
        }
        count += queue_output_u8(&qp->rx, &data_buf[count], 1);  //功能
        cmd = data_buf[count-1];
        count += queue_output_u8(&qp->rx, &data_buf[count], 1);  //长度
        len = data_buf[count-1];
        if(queue_length(&qp->rx) < len)
        {
            return;
        }
        count += queue_output_u8(&qp->rx, &data_buf[count], len);  //数据
        crc16 = crc_get_crc16(data_buf, count);

        queue_output_u8(&qp->rx, &data_buf[count], 2);  //校验
        if(crc16 != ((data_buf[count] << 8) + data_buf[count+1]))
        {
            return;
        }
        
        motor_get_data(&data_buf[3]);
    }
}
/**
* @brief  发送给电机的数据写入发送缓存
* @attention 
*/
void motor_comm_send(uint8_t *data, uint8_t len)
{
    uint8_t data_buf[COMM_QUEUE_SIZE] = {0};
    uint16_t cnt = 0;
    uint16_t crc16 = 0;
    
    while(len--)
    {
        data_buf[cnt++] = *(data++);
    }
    crc16 = crc_get_crc16(data_buf, cnt);
    data_buf[cnt++] = (uint8_t)(crc16 >> 8);
    data_buf[cnt++] = (uint8_t)crc16;
    
    queue_input_u8(&motor_comm.tx, data_buf, cnt);
}
/**
* @brief  电机串口数据发送
* @attention 
*/
static uint16_t motor_comm_tx(void)
{
    uint8_t data_buf[COMM_QUEUE_SIZE] = {0};
    uint16_t len = 0;
    
    len = queue_length(&motor_comm.tx);
    if(len == 0)  //队列空
    {
        return 0;
    }
    len = queue_read_u8(&motor_comm.tx, data_buf, len);  //读取队列
    len = uart2_send(data_buf, len);  //数据写入DMA
    queue_delete(&motor_comm.tx, len); //队列中删除已发送数据
    
    return len;
}
/**
* @brief  电机读取状态
* @attention 
*/
void motor_read_state(void)
{
    uint8_t data[10] = {0};
    uint8_t len = 0;
    
    data[len++] = MOTOR_ADDR;  //驱动器地址
    data[len++] = 0x03;  //写指令
    data[len++] = 0x00;  //寄存器地址
    data[len++] = 0x00;  //寄存器地址
    data[len++] = 0x00;  //寄存器数量
    data[len++] = 0x05;  //寄存器数量
    motor_comm_send(data, len);
}
/**
* @brief  电机使能控制
* @attention 
*/
void motor_en_ctrl(void)
{
    static uint8_t en_last = 0;
    uint8_t data[10] = {0};
    uint8_t len = 0;
    
    if((en_last != motor_data.cmd_en) || ((motor_data.cmd_en != motor_data.fb_en) && (motor_data.cmd_en == 1)))
    {
        en_last = motor_data.cmd_en;
        
        data[len++] = MOTOR_ADDR;  //驱动器地址
        data[len++] = 0x06;  //写指令
        data[len++] = 0x00;  //寄存器地址
        data[len++] = 0x00;  //寄存器地址
        data[len++] = (uint8_t)(motor_data.cmd_en ? 0xFF : 0);
        data[len++] = (uint8_t)0;
        motor_comm_send(data, len);
    }
}
/**
* @brief  电机速度控制
* @attention 
*/
void motor_speed_ctrl(void)
{
    static float speed_last = 0;
    uint8_t data[10] = {0};
    uint8_t len = 0;
    uint16_t buf_speed = 0;
    
    if(speed_last != motor_data.cmd_speed)
    {
        speed_last = motor_data.cmd_speed;
        if(motor_data.cmd_speed != 0)  //如果速度不为零则直接打开使能
        {
            motor_data.cmd_en = 1;
        }
        
        buf_speed = motor_data.cmd_speed * 10;
        data[len++] = MOTOR_ADDR;  //驱动器地址
        data[len++] = 0x06;  //写指令
        data[len++] = 0x00;  //寄存器地址
        data[len++] = 0x02;  //寄存器地址
        data[len++] = (uint8_t)(buf_speed >> 8);
        data[len++] = (uint8_t)buf_speed;
        motor_comm_send(data, len);
    }
}
/**
* @brief  电机模块运行
* @attention 
*/
void motor_run(void)
{
    static uint8_t flag_rw = 0;

    motor_comm_rx(MOTOR_ADDR);
    if(flag_rw == 0)
    {
        flag_rw = 1;
        motor_en_ctrl();
    }
    else if(flag_rw == 1)
    {
        flag_rw = 2;
        motor_speed_ctrl();
    }
    else
    {
        flag_rw = 0;
        motor_read_state();
    }
    motor_comm_tx();
}
/**
* @brief  电机模块初始化
* @attention 
*/
void motor_init(void)
{
    uart2_init();
    queue_init(&motor_comm.tx, motor_comm_tx_data, COMM_QUEUE_SIZE);
    queue_init(&motor_comm.rx, motor_comm_rx_data, COMM_QUEUE_SIZE);
}
