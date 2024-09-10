/**@file   protocol.h
* @brief   
* @author  陈卓哲
* @date    2023/11/28
* @version 1.00.0.0
**************************************************************************************************/

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#pragma pack(push, 4)

/**
* @struct  cmd_t
* @brief   指令,上位机=>下位机
*/
typedef struct
{
    unsigned char spectrometer;  //光谱仪,1:开启;0:关闭
    unsigned char halogen;       //卤光灯,1:开启;0:关闭
    unsigned char led;           //LED灯,1:开启;0:关闭
    unsigned int  led_level;     //LED灯强度,0~5000:0%~100%
}cmd_t;

/**
* @struct  msg_t
* @brief   信息,下位机=>上位机
*/
typedef struct
{
    unsigned char state_spectrometer;  //光谱仪状态反馈,1:开启;0:关闭
    unsigned char state_halogen;       //卤光灯状态反馈,1:开启;0:关闭
    unsigned char state_led;           //LED灯状态反馈,1:开启;0:关闭
    unsigned int  state_led_level;     //LED灯强度反馈,0~5000:0%~100%
    unsigned char footrest1;           //脚踏1,1:按下;0:抬起
    unsigned char footrest2;           //脚踏2,1:按下;0:抬起
    unsigned int  tim;                 //时间(ms),启动后从0开始计时,溢出后(约49.71天)重新从0计时
    unsigned int  frame_number;        //帧号,发送给上位机的数据帧编号,从0开始计数,溢出后(2^32)重新计数
}msg_t;

#pragma pack(pop)

#endif /* PROTOCOL_H_ */
