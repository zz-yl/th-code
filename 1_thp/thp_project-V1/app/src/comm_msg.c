/**@file   comm_msg.c
* @brief   通信,发送信息处理
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "comm_msg.h"
#include "comm.h"
#include "device.h"

#include "control.h"
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
* @brief  float转u8
* @attention 
*/
static void comm_float_to_u8(float buf_fl, uint8_t *buf_u8)
{
    union
    {
        uint8_t buf[4];
        float fl;
    }data;
    
    data.fl = buf_fl;
    buf_u8[0] = data.buf[0];
    buf_u8[1] = data.buf[1];
    buf_u8[2] = data.buf[2];
    buf_u8[3] = data.buf[3];
}
/**
* @brief  float转u8
* @attention 
*/
static void comm_float_to_u8_f(float buf_fl, uint8_t *buf_u8)
{
    union
    {
        uint8_t buf[4];
        float fl;
    }data;
    
    data.fl = buf_fl;
    buf_u8[0] = data.buf[3];
    buf_u8[1] = data.buf[2];
    buf_u8[2] = data.buf[1];
    buf_u8[3] = data.buf[0];
}
/**
* @brief  u32转u8
* @attention 
*/
static void comm_u32_to_u8(uint32_t buf_u32, uint8_t *buf_u8)
{
    union
    {
        uint8_t buf[4];
        uint32_t out;
    }data;
    
    data.out = buf_u32;
    buf_u8[0] = data.buf[0];
    buf_u8[1] = data.buf[1];
    buf_u8[2] = data.buf[2];
    buf_u8[3] = data.buf[3];
}
/*数据打包*************************************************************************************************/
static uint16_t comm_msg_MSG_REPLY_MOTOR(uint8_t *data)
{
    uint16_t len = 0;
    
    return len;
}
static uint16_t comm_msg_MSG_REPLY_STOP(uint8_t *data)
{
    uint16_t len = 0;
    
    return len;
}
static uint16_t comm_msg_MSG_REPLY_PROTECT(uint8_t *data)
{
    uint16_t len = 0;
    
    return len;
}
static uint16_t comm_msg_MSG_MOTOR_POS(uint8_t *data)
{
    uint16_t len = 0;
    
    comm_float_to_u8(OutData.m_pos1, &data[len]);
    len += 4;
    comm_float_to_u8(OutData.m_pos2, &data[len]);
    len += 4;
    comm_float_to_u8(OutData.m_pos3, &data[len]);
    len += 4;
    comm_float_to_u8(OutData.m_pos4, &data[len]);
    len += 4;
    comm_float_to_u8(0.0f, &data[len]);
    len += 4;
    
    return len;
}
static uint16_t comm_msg_MSG_TYPE(uint8_t *data)
{
    uint16_t len = 0;
    
    data[len++] = MotorType;
    
    return len;
}
static uint16_t comm_msg_MSG_DEVICE_STATE(uint8_t *data)
{
    uint16_t len = 0;
    
    data[len++] = (uint8_t)(motor1.flag_move | (motor2.flag_move << 1) | (motor3.flag_move << 2) | (motor4.flag_move << 3));
    data[len++] = (uint8_t)(motor1.flag_protect | (motor2.flag_protect << 1) | (motor3.flag_protect << 2) | (motor4.flag_protect << 3));
    data[len++] = 0;
    data[len++] = 0;
    
    return len;
}
static uint16_t comm_msg_MSG_REPLY_SN(uint8_t *data)
{
    uint16_t len = 0;
    
    return len;
}
static uint16_t comm_msg_MSG_SN(uint8_t *data)
{
    uint16_t len = 15;
    
    memcpy(data, SN, len);
    
    return len;
}
static uint16_t comm_msg_MSG_VERSION(uint8_t *data)
{
    uint16_t len = 0;
    
    data[len++] = THP_VERSION_X;
    data[len++] = THP_VERSION_Y;
    data[len++] = THP_VERSION_Z;
    data[len++] = THP_VERSION_B;
    
    return len;
}
static uint16_t comm_msg_MSG_ALL_POS(uint8_t *data)
{
    uint16_t len = 0;
    
//    comm_u32_to_u8(motor_data.m1.pos_all, &data[len]);
//    len += 4;
//    comm_u32_to_u8(motor_data.m2.pos_all, &data[len]);
//    len += 4;
//    comm_u32_to_u8(motor_data.m3.pos_all, &data[len]);
//    len += 4;
//    comm_u32_to_u8(motor_data.m4.pos_all, &data[len]);
//    len += 4;
//    comm_u32_to_u8(motor_data.m5.pos_all, &data[len]);
//    len += 4;
    
    return len;
}
static uint16_t comm_msg_MSG_ALL_TIME(uint8_t *data)
{
    uint16_t len = 0;
    
//    comm_u32_to_u8(motor_data.tim_run, &data[len]);
//    len += 4;
    
    return len;
}
static uint16_t comm_msg_MSG_REPLY_CALIBRATE_DATA(uint8_t *data, uint8_t value)
{
    uint16_t len = 0;
    
    data[len++] = value;
    
    return len;
}
static uint16_t comm_msg_MSG_CALIBRATE_DATA(uint8_t *data)
{
    uint16_t len = 0;
    uint8_t buf[5] = {0};
    uint16_t i = 0;
    float *qf = &robot_para.lh;
    
//    /* 读E2 */
//    mem_e2prom_read(&buf[0], 1, MEM_ADDR_DATA_SIZE);
//    len = buf[0];
//    if(len == 0xFF)
//    {
//        len = 120;
//    }
//    mem_e2prom_read(&data[0], len, MEM_ADDR_DATA);
    
    STMFLASH_Read(FLASH_CAL,(uint32_t*)data, 120);
//    memset(&data[0], 0, 120);
//    memcpy(&data[0], (uint8_t *)(&robot_para), 76);
//    for(i=0; i<19; i++)
//    {
//        comm_float_to_u8_f(qf[i], &data[i*4]);
//    }
    len = 120;
    
    
    return len;
}


/**
* @brief 填充发送信息
* @attention 
*/
void CommTxFillMsg(uint8_t cmd)
{
    uint8_t data[QUEUE_SIZE] = {0};
    uint16_t len = 0;

    switch(cmd)
    {
        case MSG_REPLY_MOTOR         : len = comm_msg_MSG_REPLY_MOTOR(data);          break;
        case MSG_REPLY_STOP          : len = comm_msg_MSG_REPLY_STOP(data);           break;
        case MSG_REPLY_PROTECT       : len = comm_msg_MSG_REPLY_PROTECT(data);        break;
        case MSG_MOTOR_POS           : len = comm_msg_MSG_MOTOR_POS(data);            break;
        case MSG_TYPE                : len = comm_msg_MSG_TYPE(data);                 break;
        case MSG_DEVICE_STATE        : len = comm_msg_MSG_DEVICE_STATE(data);         break;
        case MSG_REPLY_SN            : len = comm_msg_MSG_REPLY_SN(data);             break;
        case MSG_SN                  : len = comm_msg_MSG_SN(data);                   break;
        case MSG_VERSION             : len = comm_msg_MSG_VERSION(data);              break;
        case MSG_ALL_POS             : len = comm_msg_MSG_ALL_POS(data);              break;
        case MSG_ALL_TIME            : len = comm_msg_MSG_ALL_TIME(data);             break;
//        case MSG_REPLY_CALIBRATE_DATA: len = comm_msg_MSG_REPLY_CALIBRATE_DATA(data); break;
        case MSG_CALIBRATE_DATA      : len = comm_msg_MSG_CALIBRATE_DATA(data);       break;
        default:break;
    }

    CommTxGui(cmd, data, len);
}
/**
* @brief 填充发送信息
* @attention 
*/
void comm_msg_data(uint8_t cmd, uint8_t value)
{
    uint8_t data[QUEUE_SIZE] = {0};
    uint16_t len = 0;

    switch(cmd)
    {
        case MSG_REPLY_CALIBRATE_DATA: len = comm_msg_MSG_REPLY_CALIBRATE_DATA(data, value); break;
        default:break;
    }

    CommTxGui(cmd, data, len);
}
