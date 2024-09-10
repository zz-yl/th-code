/**@file   comm_cmd.c
* @brief   通信,指令接收处理
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "comm_cmd.h"
#include "comm.h"
#include "comm_msg.h"

#include "thp_cfg.h"
#include "motor_comm.h"
#include "sys_state_machine.h"
#include "motor_state_machine.h"
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

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  u8转float
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
/*指令解析*************************************************************************************************/
static void comm_cmd_CMD_CTRL_MOTOR_POS(uint8_t *data)
{
    float data_f = 0;
    
    /* 位置指令读取 */
    data_f = comm_u8_to_float(&data[1]);
    motor_data[MOTOR_1].cmd_pos = data_f;
    data_f = comm_u8_to_float(&data[5]);
    motor_data[MOTOR_2].cmd_pos = data_f;
    data_f = comm_u8_to_float(&data[9]);
    motor_data[MOTOR_3].cmd_pos = data_f;
    data_f = comm_u8_to_float(&data[13]);
    motor_data[MOTOR_4].cmd_pos = data_f;
    data_f = comm_u8_to_float(&data[17]);
    motor_data[MOTOR_5].cmd_pos = data_f;
    /* 位置指令执行 */
    sm_motor_toggle(MOTOR_STATE_RUN);
    /* 发送应答信息 */
    comm_msg(MSG_REPLY_MOTOR);
}
static void comm_cmd_CMD_STOP_MOTOR(uint8_t *data)
{
//    motor_stop_all();
    sm_motor_toggle(MOTOR_STATE_STOP);
    /* 发送应答信息 */
    comm_msg(MSG_REPLY_STOP);
}
static void comm_cmd_CMD_CLEAR_MOTOR_PROTECT(uint8_t *data)
{
//    motor_clear_lock();
    /* 发送应答信息 */
    comm_msg(MSG_REPLY_PROTECT);
}
static void comm_cmd_CMD_READ_MOTOR_POS(uint8_t *data)
{
    comm_msg(MSG_MOTOR_POS);
}
static void comm_cmd_CMD_READ_TYPE(uint8_t *data)
{
    comm_msg(MSG_TYPE);
}
static void comm_cmd_CMD_READ_DEVICE_STATE(uint8_t *data)
{
    comm_msg(MSG_DEVICE_STATE);
}
static void comm_cmd_CMD_MOTOR_CALIBRATE(uint8_t *data)
{
    
}
static void comm_cmd_CMD_SET_SN(uint8_t *data)
{
    memcpy(thp_sn, &data[1], 15);
    mem_e2prom_write(&data[1], 15, MEM_ADDR_SN);
    /* 发送应答信息 */
    comm_msg(MSG_REPLY_SN);
}
static void comm_cmd_CMD_READ_SN(uint8_t *data)
{
    comm_msg(MSG_SN);
}
static void comm_cmd_CMD_READ_VERSION(uint8_t *data)
{
    comm_msg(MSG_VERSION);
}
static void comm_cmd_CMD_READ_ALL_POS(uint8_t *data)
{
    comm_msg(MSG_ALL_POS);
}
static void comm_cmd_CMD_READ_ALL_TIME(uint8_t *data)
{
    comm_msg(MSG_ALL_TIME);
}
static void comm_cmd_CMD_WRITE_CALIBRATE_DATA(uint8_t *data)
{
    /* 发送应答信息 */
    comm_msg_data(MSG_REPLY_CALIBRATE_DATA, 0);
    /* 写E2 */
    mem_e2prom_write(&data[1], data[0], MEM_ADDR_DATA);
    mem_e2prom_write(&data[0], 1, MEM_ADDR_DATA_SIZE);
    /* 发送应答信息 */
    comm_msg_data(MSG_REPLY_CALIBRATE_DATA, 1);
}
static void comm_cmd_CMD_READ_CALIBRATE_DATA(uint8_t *data)
{
    comm_msg(MSG_CALIBRATE_DATA);
}
/*指令解析================================================================================================*/
/**
* @brief  指令执行
* @attention 
*/
void comm_cmd(uint16_t cmd, uint8_t *data)
{
    switch(cmd)  //处理指令
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
