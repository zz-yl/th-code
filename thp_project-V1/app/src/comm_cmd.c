/**@file   comm_cmd.c
* @brief   ͨ��,ָ����մ���
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "comm_cmd.h"
#include "comm.h"
#include "comm_msg.h"
#include "device.h"
#include "memory.h"

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

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  u8תfloat
* @attention 
*/
static float comm_u8_to_float(uint8_t *buf_u8)
{
    union
    {
        uint8_t buf[4];
        float fl;
    }data;
    
    data.buf[0] = buf_u8[0];
    data.buf[1] = buf_u8[1];
    data.buf[2] = buf_u8[2];
    data.buf[3] = buf_u8[3];
    
    return data.fl;
}
/*ָ�����*************************************************************************************************/
static void comm_cmd_CMD_CTRL_MOTOR_POS(uint8_t *data)
{
    float data_f = 0;
    
//    if((commu.nextaction == cmd_stepmove) || (commu.nextaction == cmd_move) || (commu.nextaction == cmd_stopmove))
    if((commu.nextaction == cmd_fault) || (commu.nextaction == cmd_move) || (commu.nextaction == cmd_stopmove))
    {
        return;
    }
    else
    {
        commu.nextaction = cmd_move;
    }
    /* λ��ָ���ȡ */
    data_f = comm_u8_to_float(&data[1]);
    commu.len_m1 = data_f;
    data_f = comm_u8_to_float(&data[5]);
    commu.len_m2 = data_f;
    data_f = comm_u8_to_float(&data[9]);
    commu.len_m3 = data_f;
    data_f = comm_u8_to_float(&data[13]);
    commu.len_m4 = data_f;
    /* λ��ָ��ִ�� */

    /* ����Ӧ����Ϣ */
    CommTxFillMsg(MSG_REPLY_MOTOR);
}
static void comm_cmd_CMD_STOP_MOTOR(uint8_t *data)
{
    commu.nextaction = cmd_stopmove;
    /* ����Ӧ����Ϣ */
    CommTxFillMsg(MSG_REPLY_STOP);
}
static void comm_cmd_CMD_CLEAR_MOTOR_PROTECT(uint8_t *data)
{
    commu.nextaction = cmd_removeprotect;
    /* ����Ӧ����Ϣ */
    CommTxFillMsg(MSG_REPLY_PROTECT);
}
static void comm_cmd_CMD_READ_MOTOR_POS(uint8_t *data)
{
    CommTxFillMsg(MSG_MOTOR_POS);
}
static void comm_cmd_CMD_READ_TYPE(uint8_t *data)
{
    CommTxFillMsg(MSG_TYPE);
}
static void comm_cmd_CMD_READ_DEVICE_STATE(uint8_t *data)
{
    CommTxFillMsg(MSG_DEVICE_STATE);
}
static void comm_cmd_CMD_MOTOR_CALIBRATE(uint8_t *data)
{
//    cal_ctrl.cmd = (calibrate_cmd_t)data[0];
}
static void comm_cmd_CMD_SET_SN(uint8_t *data)
{
    memcpy(SN, &data[1], 15);
    MemSNWrite(&data[1], 15);
    /* ����Ӧ����Ϣ */
    CommTxFillMsg(MSG_REPLY_SN);
}
static void comm_cmd_CMD_READ_SN(uint8_t *data)
{
    CommTxFillMsg(MSG_SN);
}
static void comm_cmd_CMD_READ_VERSION(uint8_t *data)
{
    CommTxFillMsg(MSG_VERSION);
}
static void comm_cmd_CMD_READ_ALL_POS(uint8_t *data)
{
    CommTxFillMsg(MSG_ALL_POS);
}
static void comm_cmd_CMD_READ_ALL_TIME(uint8_t *data)
{
    CommTxFillMsg(MSG_ALL_TIME);
}
static void comm_cmd_CMD_WRITE_CALIBRATE_DATA(uint8_t *data)
{
//    /* ����Ӧ����Ϣ */
    comm_msg_data(MSG_REPLY_CALIBRATE_DATA, 0);
    /* дE2 */
    STMFLASH_Write(FLASH_CAL, (uint32_t*)data, 120);
//    mem_e2prom_write(&data[1], data[0], MEM_ADDR_DATA);
//    mem_e2prom_write(&data[0], 1, MEM_ADDR_DATA_SIZE);
    /* ����Ӧ����Ϣ */
    comm_msg_data(MSG_REPLY_CALIBRATE_DATA, 1);
}
static void comm_cmd_CMD_READ_CALIBRATE_DATA(uint8_t *data)
{
    CommTxFillMsg(MSG_CALIBRATE_DATA);
}
/*ָ�����================================================================================================*/
/**
* @brief  ָ��ִ��
* @attention 
*/
void CommCmd(uint16_t cmd, uint8_t *data)
{
    switch(cmd)  //����ָ��
    {
        case CMD_CTRL_MOTOR_POS        : comm_cmd_CMD_CTRL_MOTOR_POS(data);         break;
        case CMD_STOP_MOTOR            : comm_cmd_CMD_STOP_MOTOR(data);             break;
        case CMD_CLEAR_MOTOR_PROTECT   : comm_cmd_CMD_CLEAR_MOTOR_PROTECT(data);    break;
        case CMD_READ_MOTOR_POS        : comm_cmd_CMD_READ_MOTOR_POS(data);         break;
        case CMD_READ_TYPE             : comm_cmd_CMD_READ_TYPE(data);              break;
        case CMD_READ_DEVICE_STATE     : comm_cmd_CMD_READ_DEVICE_STATE(data);      break;
        case CMD_MOTOR_CALIBRATE       : comm_cmd_CMD_MOTOR_CALIBRATE(data);        break;
        case CMD_SET_SN                : comm_cmd_CMD_SET_SN(data);                 break;
        case CMD_READ_SN               : comm_cmd_CMD_READ_SN(data);                break;
        case CMD_READ_VERSION          : comm_cmd_CMD_READ_VERSION(data);           break;
        case CMD_READ_ALL_POS          : comm_cmd_CMD_READ_ALL_POS(data);           break;
        case CMD_READ_ALL_TIME         : comm_cmd_CMD_READ_ALL_TIME(data);          break;
        case CMD_WRITE_CALIBRATE_DATA  : comm_cmd_CMD_WRITE_CALIBRATE_DATA(data);   break;
        case CMD_READ_CALIBRATE_DATA   : comm_cmd_CMD_READ_CALIBRATE_DATA(data);    break;
        default: break;
    }
}
