/**@file   comm_protocol.h
* @brief   ͨ��Э��
* @author  ��׿��
* @date    2023/9/26
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef COMM_PROTOCOL_H_
#define COMM_PROTOCOL_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "stdint.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**
* @enum    comm_addr_t
* @brief   ͨ�ŵ�ַ
*/
typedef enum
{
    COMM_ADDR_CTRL_OLD  = 0xFFFF,
    COMM_ADDR_CTRL    = 0x31,
    COMM_ADDR_DEVICE  = 0x32,
}comm_addr_t;

///**
//* @enum    comm_cmd_t
//* @brief   ָ��,����λ��->��λ��
//*/
//typedef enum
//{
//    CMD_CTRL_MOTOR_POS      = 0x01,  ///< ��ֳ��ȿ���
//    CMD_CTRL_TCP            = 0x02,  ///< TCP����
//    CMD_READ_MOTOR_POS      = 0x04,  ///< ��ȡ��ֳ���
//    CMD_READ_TCP            = 0x08,  ///< ��ȡTCPλ��
//    CMD_STOP_MOTOR          = 0x10,  ///< ֹͣ�˶�
//    CMD_READ_MOTOR_STATE    = 0x11,  ///< ��ѯ��е��״̬
//    CMD_CLEAR_MOTOR_PROTECT = 0x12,  ///< ���ת
//    CMD_SEND_CALIBRATE_DATA = 0x14,  ///< ����У׼����
//    CMD_SET_SN              = 0x18,  ///< ����SN��
//    CMD_GET_VERSION         = 0x22,  ///< ��ѯ����汾��
//    CMD_GET_SN              = 0x24,  ///< ��ѯSN��
//}comm_cmd_t;

///**
//* @enum    comm_msg_t
//* @brief   ��Ϣ,����λ��->��λ��
//*/
//typedef enum
//{
//    MSG_REPLY_MOTOR          = 0x01,  ///< �յ���ֳ��ȿ���
//    MSG_REPLY_TCP            = 0x02,  ///< �յ�TCP��������
//    MSG_MOTOR_POS            = 0x04,  ///< ��ֳ���
//    MSG_TCP_POS              = 0x08,  ///< TCPλ��
//    MSG_REPLY_STOP           = 0x10,  ///< �յ�ֹͣ�˶�
//    MSG_MOTOR_STATE          = 0x11,  ///< ��е��״̬
//    MSG_REPLY_PROTECT        = 0x12,  ///< �յ����ת
//    MSG_REPLY_CALIBRATE_DATA = 0x14,  ///< �յ�У׼����
//    MSG_REPLY_SN             = 0x18,  ///< ��������SN��
//    MSG_VERSION              = 0x22,  ///< ��������汾��
//    MSG_SN                   = 0x24,  ///< ����SN��
//}comm_msg_t;

/**
* @enum    comm_cmd_t
* @brief   ָ��,����λ��->��λ��
*/
typedef enum
{
    CMD_CTRL_MOTOR_POS         = 0x01, ///< ���λ�ÿ���ָ��
    CMD_STOP_MOTOR             = 0x02, ///< ���е��ֹͣ�˶�
    CMD_CLEAR_MOTOR_PROTECT    = 0x03, ///< ���е�����ת
    CMD_READ_MOTOR_POS         = 0x05, ///< ��ȡ���λ�ü�״̬
    CMD_READ_TYPE              = 0x06, ///< ��ȡװ���ͺ�
    CMD_READ_DEVICE_STATE      = 0x07, ///< ��ȡ�豸״̬
    CMD_MOTOR_CALIBRATE        = 0x0A, ///< У׼
    CMD_SET_SN                 = 0x0B, ///< ����SN��(����010602023090001)
    CMD_READ_SN                = 0x0C, ///< ��ѯSN��
    CMD_READ_VERSION           = 0x0D, ///< ��ѯ����汾��
    CMD_READ_ALL_POS           = 0x0E, ///< ����������г�
    CMD_READ_ALL_TIME          = 0x0F, ///< ���������ʱ��
    CMD_WRITE_CALIBRATE_DATA   = 0x11, ///< дУ׼����
    CMD_READ_CALIBRATE_DATA    = 0x12, ///< ��У׼����
}comm_cmd_t;

/**
* @enum    comm_msg_t
* @brief   ��Ϣ,����λ��->��λ��
*/
typedef enum
{
    MSG_REPLY_MOTOR          = 0x81,  ///< �յ���ֳ��ȿ�������
    MSG_REPLY_STOP           = 0x82,  ///< �յ�ֹͣ����
    MSG_REPLY_PROTECT        = 0x83,  ///< �յ����ת����
    MSG_MOTOR_POS            = 0x85,  ///< ���λ�ü�״̬
    MSG_TYPE                 = 0x86,  ///< װ���ͺ�
    MSG_DEVICE_STATE         = 0x87,  ///< �豸״̬
    MSG_REPLY_SN             = 0x8B,  ///< ����SN�ųɹ�
    MSG_SN                   = 0x8C,  ///< SN��
    MSG_VERSION              = 0x8D,  ///< ����汾��
    MSG_ALL_POS              = 0x8E,  ///< ����������г�
    MSG_ALL_TIME             = 0x8F,  ///< ���������ʱ��
    MSG_REPLY_CALIBRATE_DATA = 0x91,  ///< У׼�������ռ�д��״̬
    MSG_CALIBRATE_DATA       = 0x92,  ///< У׼����
}comm_msg_t;

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

#endif /* COMM_PROTOCOL_H_ */
