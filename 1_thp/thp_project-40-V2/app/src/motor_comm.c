/**@file   motor_comm.c
* @brief   串口电机控制(韩国电机)
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "motor_comm.h"
#include "comm_protocol.h"
#include "motor_state_machine.h"
#include "control.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define MOTOR_COMM_QUEUE_SIZE  64  //队列大小

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @struct  motor_comm_t
* @brief   电机通信数据队列
*/
typedef struct
{
	queue_list_t tx;  //发送队列
    queue_list_t rx;  //接收队列
}motor_comm_t;

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

/**
* @struct  motor_comm_q
* @brief   电机通信数据队列数组
*/
struct
{
    uint8_t motor1_comm_tx_data[MOTOR_COMM_QUEUE_SIZE];
    uint8_t motor1_comm_rx_data[MOTOR_COMM_QUEUE_SIZE];
    uint8_t motor2_comm_tx_data[MOTOR_COMM_QUEUE_SIZE];
    uint8_t motor2_comm_rx_data[MOTOR_COMM_QUEUE_SIZE];
    uint8_t motor3_comm_tx_data[MOTOR_COMM_QUEUE_SIZE];
    uint8_t motor3_comm_rx_data[MOTOR_COMM_QUEUE_SIZE];
    uint8_t motor4_comm_tx_data[MOTOR_COMM_QUEUE_SIZE];
    uint8_t motor4_comm_rx_data[MOTOR_COMM_QUEUE_SIZE];
    uint8_t motor5_comm_tx_data[MOTOR_COMM_QUEUE_SIZE];
    uint8_t motor5_comm_rx_data[MOTOR_COMM_QUEUE_SIZE];
}motor_comm_q;
/**
* @brief   电机通信数据
*/
motor_comm_t motor_comm[5];

motor_t motor_data[5];

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

static void motor_comm_read(uint8_t *data, motor_id_t id)
{
    uint16_t buf = 0;
    
    buf = ((uint16_t)data[0] + ((uint16_t)data[1] << 8));
    
    motor_data[id].pos = (float)buf / 4095.0f * 39.6f;
}
/**
* @brief  接收信息处理
* @attention 
*/
void motor_msg(uint8_t *data, motor_id_t id)
{
    motor_comm_read(data, id);
}

/**
* @brief  串口4接收回调函数
* @attention 
*/
void uart4_receive(uint8_t *data, uint16_t len)
{
    static uint8_t cnt = 0;
    
    queue_input_u8(&motor_comm[MOTOR_1].rx, data, len);  //数据入队
    if(len != 9)
    {
        cnt++;
    }
    if(cnt > 3)
    {
        cnt = 0;
        uart4_init();
    }
}
/**
* @brief  串口3接收回调函数
* @attention 
*/
void uart3_receive(uint8_t *data, uint16_t len)
{
    static uint8_t cnt = 0;
    
    queue_input_u8(&motor_comm[MOTOR_2].rx, data, len);  //数据入队
    
    if(len != 9)
    {
        cnt++;
    }
    if(cnt > 3)
    {
        cnt = 0;
        uart3_init();
    }
}
/**
* @brief  串口5接收回调函数
* @attention 
*/
void uart5_receive(uint8_t *data, uint16_t len)
{
    static uint8_t cnt = 0;
    
    queue_input_u8(&motor_comm[MOTOR_3].rx, data, len);  //数据入队
    
    if(len != 9)
    {
        cnt++;
    }
    if(cnt > 3)
    {
        cnt = 0;
        uart5_init();
    }
}
/**
* @brief  串口7接收回调函数
* @attention 
*/
void uart7_receive(uint8_t *data, uint16_t len)
{
    static uint8_t cnt = 0;
    
    queue_input_u8(&motor_comm[MOTOR_4].rx, data, len);  //数据入队
    
    if(len != 9)
    {
        cnt++;
    }
    if(cnt > 3)
    {
        cnt = 0;
        uart7_init();
    }
}
/**
* @brief  串口6接收回调函数
* @attention 
*/
void uart6_receive(uint8_t *data, uint16_t len)
{
    static uint8_t cnt = 0;
    
    queue_input_u8(&motor_comm[MOTOR_5].rx, data, len);  //数据入队
    
    if(len != 9)
    {
        cnt++;
    }
    if(cnt > 3)
    {
        cnt = 0;
        uart6_init();
    }
}

/**
* @brief  校验和
* @attention 
*/
static uint8_t motor_check_sum(uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    
    while(len--)
    {
        sum += data[len];
    }
    
    return 0xFF - sum;
}

/**
* @brief  接收数据处理
* @attention 
*/
static void motor_rx(motor_id_t id)
{
    uint8_t data_buf[MOTOR_COMM_QUEUE_SIZE] = {0};
    uint16_t count = 0;
    uint8_t len = 0;
    uint8_t check = 0;
    motor_comm_t *qp = &motor_comm[id];
    /* 遍历队列 */
    while(queue_length(&qp->rx) != 0)  
    {
        queue_output_u8(&qp->rx, &data_buf[count], 3);  //帧头
        if((data_buf[0] != 0xFF) || (data_buf[1] != 0xFF) || (data_buf[2] != 0xFF))
        {
            return;
        }
        count += queue_output_u8(&qp->rx, &data_buf[count], 1);  //地址
        if(data_buf[0] != (id+1))
        {
            return;
        }
        count += queue_output_u8(&qp->rx, &data_buf[count], 1);  //长度
        len = data_buf[count-1];
        if(queue_length(&qp->rx) < len)
        {
            return;
        }
        
        count += queue_output_u8(&qp->rx, &data_buf[count], 1);  //ERROR
        
        count += queue_output_u8(&qp->rx, &data_buf[count], len - 2);  //数据
        check = motor_check_sum(data_buf, count);

        queue_output_u8(&qp->rx, &data_buf[count], 1);  //校验
        if(check != data_buf[count])
        {
            return;
        }
        
        motor_msg(&data_buf[3], id);
    }
}
/**
* @brief  电机指令写入发送缓存
* @attention 
*/
void motor_fill_msg(motor_id_t id, uint16_t cmd, uint8_t *data, uint8_t len)
{
    uint8_t data_buf[MOTOR_COMM_QUEUE_SIZE] = {0};
    uint16_t cnt = 0;
    uint8_t check = 0;
    
    data_buf[cnt++] = 0xFF;  //帧头
    data_buf[cnt++] = 0xFF;
    data_buf[cnt++] = 0xFF;
    data_buf[cnt++] = id + 1;  //电机id从1开始
    data_buf[cnt++] = len + 3;  //长度除数据外还有指令2和校验和1
    data_buf[cnt++] = (uint8_t)(cmd >> 8);
    data_buf[cnt++] = (uint8_t)cmd;
    
    while(len--)
    {
        data_buf[cnt++] = *(data++);
    }
    check = motor_check_sum(&data_buf[3], cnt - 3);
    data_buf[cnt++] = check;
    
    queue_input_u8(&motor_comm[id].tx, data_buf, cnt);
}
/**
* @brief  电机串口指令发送
* @attention 
*/
static uint16_t motor_tx(motor_id_t id)
{
    uint8_t data_buf[MOTOR_COMM_QUEUE_SIZE] = {0};
    uint16_t len = 0;
    motor_comm_t *qp = &motor_comm[id];
    
    len = queue_length(&qp->tx);
    if(len == 0)  //队列空
    {
        return 0;
    }
    len = queue_read_u8(&qp->tx, data_buf, len);  //读取数据
    /* 数据写入DMA */
    switch(id)
    {
        case MOTOR_1: len = uart4_send(data_buf, len); break;
        case MOTOR_2: len = uart3_send(data_buf, len); break;
        case MOTOR_3: len = uart5_send(data_buf, len); break;
        case MOTOR_4: len = uart7_send(data_buf, len); break;
        case MOTOR_5: len = uart6_send(data_buf, len); break;
        default: break;
    }
    queue_delete(&qp->tx, len); //队列中删除已发送数据
    
    return len;
}
static uint16_t motor_comm_MOTOR_COMM_CTRL_POS(uint8_t *data, motor_id_t id)
{
    uint16_t len = 0;
    uint16_t buf = 0;
    
    buf = motor_data[id].cmd_pos / 39.6f * 4095.0f;
    
    data[len++] = (uint8_t)buf;
    data[len++] = (uint8_t)(buf >> 8);
    
    return len;
}
static uint16_t motor_comm_MOTOR_COMM_READ_POS(uint8_t *data, motor_id_t id)
{
    uint16_t len = 0;
    
    data[len++] = 2;
    
    return len;
}

/**
* @brief 电机指令
* @attention 
*/
void motor_cmd(motor_id_t id, uint16_t cmd)
{
    uint8_t data[MOTOR_COMM_QUEUE_SIZE] = {0};
    uint16_t len = 0;

    switch(cmd)
    {
        case MOTOR_COMM_CTRL_POS: len = motor_comm_MOTOR_COMM_CTRL_POS(data, id); break;
        case MOTOR_COMM_READ_POS: len = motor_comm_MOTOR_COMM_READ_POS(data, id); break;
        default:break;
    }

    motor_fill_msg(id, cmd, data, len);
}
/**
* @brief  电机模块运行
* @attention 
*/
void motor_run(void)
{
    motor_rx(MOTOR_1);
    motor_rx(MOTOR_2);
    motor_rx(MOTOR_3);
    motor_rx(MOTOR_4);
    motor_rx(MOTOR_5);
    motor_tx(MOTOR_1);
    motor_tx(MOTOR_2);
    motor_tx(MOTOR_3);
    motor_tx(MOTOR_4);
    motor_tx(MOTOR_5);
}
/**
* @brief  电机模块初始化
* @attention 
*/
void motor_init(void)
{
    queue_init(&motor_comm[MOTOR_1].tx, motor_comm_q.motor1_comm_tx_data, MOTOR_COMM_QUEUE_SIZE);
    queue_init(&motor_comm[MOTOR_1].rx, motor_comm_q.motor1_comm_rx_data, MOTOR_COMM_QUEUE_SIZE);
    queue_init(&motor_comm[MOTOR_2].tx, motor_comm_q.motor2_comm_tx_data, MOTOR_COMM_QUEUE_SIZE);
    queue_init(&motor_comm[MOTOR_2].rx, motor_comm_q.motor2_comm_rx_data, MOTOR_COMM_QUEUE_SIZE);
    queue_init(&motor_comm[MOTOR_3].tx, motor_comm_q.motor3_comm_tx_data, MOTOR_COMM_QUEUE_SIZE);
    queue_init(&motor_comm[MOTOR_3].rx, motor_comm_q.motor3_comm_rx_data, MOTOR_COMM_QUEUE_SIZE);
    queue_init(&motor_comm[MOTOR_4].tx, motor_comm_q.motor4_comm_tx_data, MOTOR_COMM_QUEUE_SIZE);
    queue_init(&motor_comm[MOTOR_4].rx, motor_comm_q.motor4_comm_rx_data, MOTOR_COMM_QUEUE_SIZE);
    queue_init(&motor_comm[MOTOR_5].tx, motor_comm_q.motor5_comm_tx_data, MOTOR_COMM_QUEUE_SIZE);
    queue_init(&motor_comm[MOTOR_5].rx, motor_comm_q.motor5_comm_rx_data, MOTOR_COMM_QUEUE_SIZE);
    
    ctrl_led(CTRL_GREEN);
}
/**
* @brief 所有电机获取位置
* @attention 
*/
void motor_get_pos_all(void)
{
    motor_cmd(MOTOR_1, MOTOR_COMM_READ_POS);
    motor_cmd(MOTOR_2, MOTOR_COMM_READ_POS);
    motor_cmd(MOTOR_3, MOTOR_COMM_READ_POS);
    motor_cmd(MOTOR_4, MOTOR_COMM_READ_POS);
}
/**
* @brief 所有电机位置发送
* @attention 
*/
void motor_send_pos_all(void)
{
    motor_cmd(MOTOR_1, MOTOR_COMM_CTRL_POS);
    motor_cmd(MOTOR_2, MOTOR_COMM_CTRL_POS);
    motor_cmd(MOTOR_3, MOTOR_COMM_CTRL_POS);
    motor_cmd(MOTOR_4, MOTOR_COMM_CTRL_POS);
}
/**
* @brief 控制电机位置
* @attention 
*/
void motor_ctrl_pos(motor_id_t id, float pos)
{
    motor_data[id].cmd_pos = pos;
    sm_motor_toggle(MOTOR_STATE_RUN);
}
/**
* @brief  电机监测
* @attention 
*/
void motor_monitor(void)
{
    if(fabs(motor_data[MOTOR_1].cmd_pos - motor_data[MOTOR_1].pos) < 0.5)
    {
        motor_data[MOTOR_1].state_r = MOTOR_STOP;
    }
    if(fabs(motor_data[MOTOR_2].cmd_pos - motor_data[MOTOR_2].pos) < 0.5)
    {
        motor_data[MOTOR_2].state_r = MOTOR_STOP;
    }
    if(fabs(motor_data[MOTOR_3].cmd_pos - motor_data[MOTOR_3].pos) < 0.5)
    {
        motor_data[MOTOR_3].state_r = MOTOR_STOP;
    }
    if(fabs(motor_data[MOTOR_4].cmd_pos - motor_data[MOTOR_4].pos) < 0.5)
    {
        motor_data[MOTOR_4].state_r = MOTOR_STOP;
    }
}
