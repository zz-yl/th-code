/**@file   comm_protocol.h
* @brief   通信协议
* @author  陈卓哲
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
* @brief   通信地址
*/
typedef enum
{
    COMM_ADDR_CTRL_OLD  = 0xFFFF,
    COMM_ADDR_CTRL    = 0x31,
    COMM_ADDR_DEVICE  = 0x32,
}comm_addr_t;

/**
* @enum    comm_cmd_t
* @brief   指令,即上位机->下位机
*/
typedef enum
{
    CMD_CTRL_MOTOR_POS         = 0x01, ///< 电机位置控制指令
    CMD_STOP_MOTOR             = 0x02, ///< 所有电机停止运动
    CMD_CLEAR_MOTOR_PROTECT    = 0x03, ///< 所有电机清堵转
    CMD_READ_MOTOR_POS         = 0x05, ///< 读取电机位置及状态
    CMD_READ_TYPE              = 0x06, ///< 读取装置型号
    CMD_READ_DEVICE_STATE      = 0x07, ///< 读取设备状态
    CMD_MOTOR_CALIBRATE        = 0x0A, ///< 校准
    CMD_SET_SN                 = 0x0B, ///< 设置SN号(例：010602023090001)
    CMD_READ_SN                = 0x0C, ///< 查询SN号
    CMD_READ_VERSION           = 0x0D, ///< 查询软件版本号
    CMD_READ_ALL_POS           = 0x0E, ///< 电机运行总行程
    CMD_READ_ALL_TIME          = 0x0F, ///< 电机运行总时间
    CMD_WRITE_CALIBRATE_DATA   = 0x11, ///< 写校准参数
    CMD_READ_CALIBRATE_DATA    = 0x12, ///< 读校准参数
}comm_cmd_t;

/**
* @enum    comm_msg_t
* @brief   信息,即下位机->上位机
*/
typedef enum
{
    MSG_REPLY_MOTOR          = 0x81,  ///< 收到电钢长度控制命令
    MSG_REPLY_STOP           = 0x82,  ///< 收到停止命令
    MSG_REPLY_PROTECT        = 0x83,  ///< 收到清堵转命令
    MSG_MOTOR_POS            = 0x85,  ///< 电机位置及状态
    MSG_TYPE                 = 0x86,  ///< 装置型号
    MSG_DEVICE_STATE         = 0x87,  ///< 设备状态
    MSG_REPLY_SN             = 0x8B,  ///< 设置SN号成功
    MSG_SN                   = 0x8C,  ///< SN号
    MSG_VERSION              = 0x8D,  ///< 软件版本号
    MSG_ALL_POS              = 0x8E,  ///< 电机运行总行程
    MSG_ALL_TIME             = 0x8F,  ///< 电机运行总时间
    MSG_REPLY_CALIBRATE_DATA = 0x91,  ///< 校准参数接收及写入状态
    MSG_CALIBRATE_DATA       = 0x92,  ///< 校准参数
}comm_msg_t;

/**
* @enum    motor_comm_cmd_t
* @brief   韩国电机指令
*/
typedef enum
{
    MOTOR_COMM_CTRL_POS      = 0xF386,  ///< 
    MOTOR_COMM_READ_POS      = 0xF28C,  ///< 
}motor_comm_cmd_t;

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
