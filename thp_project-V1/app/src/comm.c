/**@file   comm.c
* @brief   ͨ��ģ��
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "comm.h"
#include "comm_cmd.h"
#include "comm_msg.h"
#include "crc_check.h"
#include "device.h"
#include "comm_protocol.h"

#include "control.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

//static uint8_t M1TxData[16] = {0};
//static uint8_t M1RxData[16] = {0};
//COMM_DATA CommM1 = {.tx.data = M1TxData, .tx.size = sizeof(M1TxData) / sizeof(M1TxData[0]), .rx.data = M1RxData, .rx.size = sizeof(M1RxData) / sizeof(M1RxData[0])};
//static uint8_t M2TxData[16] = {0};
//static uint8_t M2RxData[16] = {0};
//COMM_DATA CommM2 = {.tx.data = M2TxData, .tx.size = sizeof(M2TxData) / sizeof(M2TxData[0]), .rx.data = M2RxData, .rx.size = sizeof(M2RxData) / sizeof(M2RxData[0])};
//static uint8_t M3TxData[16] = {0};
//static uint8_t M3RxData[16] = {0};
//COMM_DATA CommM3 = {.tx.data = M3TxData, .tx.size = sizeof(M3TxData) / sizeof(M3TxData[0]), .rx.data = M3RxData, .rx.size = sizeof(M3RxData) / sizeof(M3RxData[0])};
//static uint8_t M4TxData[16] = {0};
//static uint8_t M4RxData[16] = {0};
//COMM_DATA CommM4 = {.tx.data = M4TxData, .tx.size = sizeof(M4TxData) / sizeof(M4TxData[0]), .rx.data = M4RxData, .rx.size = sizeof(M4RxData) / sizeof(M4RxData[0])};
uint8_t GuiTxData[QUEUE_SIZE] = {0};
uint8_t GuiRxData[QUEUE_SIZE] = {0};
COMM_DATA CommGui = {.tx.data = GuiTxData, .tx.size = sizeof(GuiTxData) / sizeof(GuiTxData[0]), .rx.data = GuiRxData, .rx.size = sizeof(GuiRxData) / sizeof(GuiRxData[0])};

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void Usart1Receive(uint8_t *data, uint16_t len)
{
    if(data[0] != 0x01)
    {
        return;
    }
    if(data[1] == 0x02)
    {
        return;
    }
    if(data[1] == 0x9F)
    {
        return;
    }
    if(len == 3)
    {
        if(data[1] == 0x01)
        {
            motor1.flag_protect = 1;
        }
        if(data[1] == 0x00)
        {
            motor1.flag_protect = 0;
        }
    }
    if(len == 6)
    {
        OutData.e_pos1 = (float)(((int32_t)data[1] << 24) | ((int32_t)data[2] << 16) | ((int32_t)data[3] << 8) | ((int32_t)data[4] << 0)) / 65536;
        OutData.m_pos1 = OutData.init_pos1 + OutData.e_pos1;
    }
}

void Usart2Receive(uint8_t *data, uint16_t len)
{
    if(data[0] != 0x02)
    {
        return;
    }
    if(data[1] == 0x02)
    {
        return;
    }
    if(data[1] == 0x9F)
    {
        return;
    }
    if(len == 3)
    {
        if(data[1] == 0x01)
        {
            motor2.flag_protect = 1;
        }
        if(data[1] == 0x00)
        {
            motor2.flag_protect = 0;
        }
    }
    if(len == 6)
    {
        OutData.e_pos2 = (float)(((int32_t)data[1] << 24) | ((int32_t)data[2] << 16) | ((int32_t)data[3] << 8) | ((int32_t)data[4] << 0)) / 65536;
        OutData.m_pos2 = OutData.init_pos2 + OutData.e_pos2;
    }
}
void Usart3Receive(uint8_t *data, uint16_t len)
{
    if(data[0] != 0x03)
    {
        return;
    }
    if(data[1] == 0x02)
    {
        return;
    }
    if(data[1] == 0x9F)
    {
        return;
    }
    if(len == 3)
    {
        if(data[1] == 0x01)
        {
            motor3.flag_protect = 1;
        }
        if(data[1] == 0x00)
        {
            motor3.flag_protect = 0;
        }
    }
    if(len == 6)
    {
        OutData.e_pos3 = (float)(((int32_t)data[1] << 24) | ((int32_t)data[2] << 16) | ((int32_t)data[3] << 8) | ((int32_t)data[4] << 0)) / 65536;
        OutData.m_pos3 = OutData.init_pos3 + OutData.e_pos3;
    }
}
void Uart4Receive(uint8_t *data, uint16_t len)
{
    if(data[0] != 0x04)
    {
        return;
    }
    if(data[1] == 0x02)
    {
        return;
    }
    if(data[1] == 0x9F)
    {
        return;
    }
    if(len == 3)
    {
        if(data[1] == 0x01)
        {
            motor4.flag_protect = 1;
        }
        if(data[1] == 0x00)
        {
            motor4.flag_protect = 0;
        }
    }
    if(len == 6)
    {
        OutData.e_pos4 = (float)(((int32_t)data[1] << 24) | ((int32_t)data[2] << 16) | ((int32_t)data[3] << 8) | ((int32_t)data[4] << 0)) / 65536;
        OutData.m_pos4 = OutData.init_pos4 + OutData.e_pos4;
    }
}
/**
* @brief  ����6���ջص�����
* @attention 
*/
void Usart6Receive(uint8_t *data, uint16_t len)
{
    QueueInputDataU8(&CommGui.rx, data, len);  //�������
}
/**
* @brief  ����ٶȿ���
* @attention 
*/
void MotorSpeedCtrl(float speed, uint8_t id)
{
    uint8_t tx_data[6] = {0};
    uint16_t speed_level = 0;
    
    if((id == 0) || (id > 4))
    {
        return;
    }

    tx_data[0] = id;
    tx_data[1] = 0xF6;
    if(speed >= 0)
    {
        tx_data[2] = 0x10;
        speed_level = (uint16_t)(speed / 100 * 0x4FF);
    }
    else
    {
        tx_data[2] = 0x00;
        speed_level = (uint16_t)(-speed / 100 * 0x4FF);
    }
    tx_data[2] |= (uint8_t)(speed_level >> 8) & 0x0F;
    tx_data[3] = (uint8_t)speed_level;
    tx_data[4] = 0xFF;  //���ٶ�
    tx_data[5] = GetCRC8Reverse(tx_data, 5, 0x00);
    
    switch(id)
    {
        case 1: Usart1Send(tx_data,6); break;
        case 2: Usart2Send(tx_data,6); break;
        case 3: Usart3Send(tx_data,6); break;
        case 4: Uart4Send(tx_data,6); break;
        default: break;
    }
}
/**
* @brief  ���Ƶ��λ��
* @param[in] round:Ȧ��,ÿȦ1mm
* @param[in] speed:ת��,0%--100%
* @param[in] id:���id,1--4
* @param[in] vs:�ٶ�ƽ��,0:����,1:�ر�
* @attention 
*/
void CommCtrlMotorPos(float round, float speed, uint8_t id, uint8_t vs)
{
    uint8_t tx_data[9] = {0};
    uint16_t speed_level = 0;
    uint32_t step = 0;
    
    if((id == 0) || (id > 4))
    {
        return;
    }

    tx_data[0] = id;
    tx_data[1] = 0xFD;
    if(round >= 0)
    {
        tx_data[2] = 0x10;
        step = round * 25600;
    }
    else
    {
        tx_data[2] = 0x00;
        step = -round * 25600;
    }
    speed_level = (uint16_t)(speed / 100 * 0x4FF);
    tx_data[2] |= (uint8_t)(speed_level >> 8) & 0x0F;
    tx_data[3] = (uint8_t)speed_level;
    tx_data[4] = 0x0;  //���ٶ�
    if(vs)
    {
        tx_data[4] = 0xFF;  //���ٶ�
    }
    tx_data[5] = step >> 16;
    tx_data[6] = step >> 8;
    tx_data[7] = step;
    tx_data[8] = GetCRC8Reverse(tx_data, 8, 0x00);
    
    switch(id)
    {
        case 1: Usart1Send(tx_data,9); break;
        case 2: Usart2Send(tx_data,9); break;
        case 3: Usart3Send(tx_data,9); break;
        case 4: Uart4Send(tx_data,9); break;
        default: break;
    }
}
/**
* @brief  ��ȡ���λ��
* @param[in] id:���id,1--4
* @attention 
*/
void CommGetMotorPos(uint8_t id)
{
    uint8_t tx_data[3] = {0};
    
    if((id == 0) || (id > 4))
    {
        return;
    }

    tx_data[0] = id;
    tx_data[1] = 0x36;
    tx_data[2] = GetCRC8Reverse(tx_data, 2, 0x00);
    
    switch(id)
    {
        case 1: Usart1Send(tx_data,3); break;
        case 2: Usart2Send(tx_data,3); break;
        case 3: Usart3Send(tx_data,3); break;
        case 4: Uart4Send(tx_data,3); break;
        default: break;
    }
}
/**
* @brief  �������ݴ���
* @attention 
*/
static uint8_t CommRx(COMM_DATA *qp, uint8_t addr)
{
    uint8_t data_buf[QUEUE_SIZE] = {0};
    uint16_t count = 0;
    uint8_t cmd = 0;
    uint8_t len = 0;
    uint8_t crc8 = 0;
    /* �������� */
    while(QueueLength(&qp->rx) != 0)  
    {
        count += QueueOutputDataU8(&qp->rx, &data_buf[count], 1);  //��ַ
        if(data_buf[0] != addr)
        {
            return 0;
        }
        count += QueueOutputDataU8(&qp->rx, &data_buf[count], 1);  //ָ��
        cmd = data_buf[count-1];
        count += QueueOutputDataU8(&qp->rx, &data_buf[count], 1);  //����
        len = data_buf[count-1];
        if(QueueLength(&qp->rx) < len)
        {
            return 0;
        }
        count += QueueOutputDataU8(&qp->rx, &data_buf[count], len);  //����
        crc8 = GetCRC8(data_buf, count, 0xFF);
          
        QueueOutputDataU8(&qp->rx, &data_buf[count], 1);  //У�� /* �˴�Э�鶨�������⣬�������� */
        if(crc8 != data_buf[count])
        {
            return 0;
        }
//        if(data_buf[count-1] != 0x6B)
//        {
//            return 0;
//        }
        CommCmd(cmd, &data_buf[2]);
    }
}
/**
* @brief  ���͸�Gui����д�뷢�ͻ���
* @attention 
*/
void CommTxGui(uint8_t cmd, uint8_t *data, uint8_t len)
{
    uint8_t data_buf[QUEUE_SIZE] = {0};
    uint16_t cnt = 0;
    uint8_t crc8 = 0;
    
    data_buf[cnt++] = (uint8_t)COMM_ADDR_CTRL;
    data_buf[cnt++] = cmd;
    data_buf[cnt++] = len;
    
    while(len--)
    {
        data_buf[cnt++] = *(data++);
    }
    crc8 = GetCRC8(data_buf, cnt, 0xFF);
    data_buf[cnt++] = crc8;
    
    QueueInputDataU8(&CommGui.tx, data_buf, cnt);
}
/**
* @brief  Gui�������ݷ���
* @attention 
*/
static uint16_t CommGuiSend(void)
{
    uint8_t data_buf[QUEUE_SIZE] = {0};
    uint16_t len = 0;
    
    len = QueueLength(&CommGui.tx);
    if(len == 0)  //���п�
    {
        return 0;
    }
    len = QueueReadDataU8(&CommGui.tx, data_buf, len);  //��ȡ����
    len = Usart6Send(data_buf, len);  //����д��DMA
    QueueDeleteData(&CommGui.tx, len);
    
    return len;
}
/**
* @brief  �������ݴ���
* @attention 
*/
static void CommTx(void)
{
    static uint16_t ui_tim = 0;
    
    ui_tim += 1;
    if(ui_tim >= 1000)  //��ʱ���͸�UI
    {
        ui_tim = 0;

    }
    
    CommGuiSend();
}
/**
* @brief  ͨ��ģ������
* @attention 
*/
void CommRun(void)
{
    CommRx(&CommGui, COMM_ADDR_DEVICE);
    CommTx();
}
