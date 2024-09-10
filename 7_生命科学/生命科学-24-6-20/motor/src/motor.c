/**@file   motor.c
* @brief   485�������
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

/* �ض���fputc����, printf�������ջ�ͨ������fputc����ַ��������� */
//int fputc(int ch, FILE *f)
//{
//    queue_input_u8(&comm_ctrl.tx, (uint8_t *)&ch, 1);
//    return ch;
//}

/**
* @brief  ���ڽ��ջص�����
* @attention 
*/
void uart2_receive(uint8_t *data, uint16_t len)
{
    queue_input_u8(&motor_comm.rx, data, len);  //�������
}
/**
* @brief  �������ݴ���
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
* @brief  �������ݴ���
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
    /* �������� */
    while(queue_length(&qp->rx) != 0)  
    {
        count += queue_output_u8(&qp->rx, &data_buf[count], 1);  //��ַ
        if(data_buf[0] != addr)
        {
            return;
        }
        count += queue_output_u8(&qp->rx, &data_buf[count], 1);  //����
        cmd = data_buf[count-1];
        count += queue_output_u8(&qp->rx, &data_buf[count], 1);  //����
        len = data_buf[count-1];
        if(queue_length(&qp->rx) < len)
        {
            return;
        }
        count += queue_output_u8(&qp->rx, &data_buf[count], len);  //����
        crc16 = crc_get_crc16(data_buf, count);

        queue_output_u8(&qp->rx, &data_buf[count], 2);  //У��
        if(crc16 != ((data_buf[count] << 8) + data_buf[count+1]))
        {
            return;
        }
        
        motor_get_data(&data_buf[3]);
    }
}
/**
* @brief  ���͸����������д�뷢�ͻ���
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
* @brief  ����������ݷ���
* @attention 
*/
static uint16_t motor_comm_tx(void)
{
    uint8_t data_buf[COMM_QUEUE_SIZE] = {0};
    uint16_t len = 0;
    
    len = queue_length(&motor_comm.tx);
    if(len == 0)  //���п�
    {
        return 0;
    }
    len = queue_read_u8(&motor_comm.tx, data_buf, len);  //��ȡ����
    len = uart2_send(data_buf, len);  //����д��DMA
    queue_delete(&motor_comm.tx, len); //������ɾ���ѷ�������
    
    return len;
}
/**
* @brief  �����ȡ״̬
* @attention 
*/
void motor_read_state(void)
{
    uint8_t data[10] = {0};
    uint8_t len = 0;
    
    data[len++] = MOTOR_ADDR;  //��������ַ
    data[len++] = 0x03;  //дָ��
    data[len++] = 0x00;  //�Ĵ�����ַ
    data[len++] = 0x00;  //�Ĵ�����ַ
    data[len++] = 0x00;  //�Ĵ�������
    data[len++] = 0x05;  //�Ĵ�������
    motor_comm_send(data, len);
}
/**
* @brief  ���ʹ�ܿ���
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
        
        data[len++] = MOTOR_ADDR;  //��������ַ
        data[len++] = 0x06;  //дָ��
        data[len++] = 0x00;  //�Ĵ�����ַ
        data[len++] = 0x00;  //�Ĵ�����ַ
        data[len++] = (uint8_t)(motor_data.cmd_en ? 0xFF : 0);
        data[len++] = (uint8_t)0;
        motor_comm_send(data, len);
    }
}
/**
* @brief  ����ٶȿ���
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
        if(motor_data.cmd_speed != 0)  //����ٶȲ�Ϊ����ֱ�Ӵ�ʹ��
        {
            motor_data.cmd_en = 1;
        }
        
        buf_speed = motor_data.cmd_speed * 10;
        data[len++] = MOTOR_ADDR;  //��������ַ
        data[len++] = 0x06;  //дָ��
        data[len++] = 0x00;  //�Ĵ�����ַ
        data[len++] = 0x02;  //�Ĵ�����ַ
        data[len++] = (uint8_t)(buf_speed >> 8);
        data[len++] = (uint8_t)buf_speed;
        motor_comm_send(data, len);
    }
}
/**
* @brief  ���ģ������
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
* @brief  ���ģ���ʼ��
* @attention 
*/
void motor_init(void)
{
    uart2_init();
    queue_init(&motor_comm.tx, motor_comm_tx_data, COMM_QUEUE_SIZE);
    queue_init(&motor_comm.rx, motor_comm_rx_data, COMM_QUEUE_SIZE);
}
