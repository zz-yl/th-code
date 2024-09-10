/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

extern UART_HandleTypeDef huart5;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
#define SCREEN_HOME_PAGE 0 //主页面
#define SCREEN_SET_DATE 1 //设置日期
#define SCREEN_HISTORY  2 //历史记录
#define SCREEN_SET_TIME 3 //设置时间
#define SCREEN_POWER_ON 4 //开机界面
#define SCREEN_CURVE    5 //历史记录曲线
#define SCREEN_PSW      6 //输入密码界面


#define HOME_ICON_BUBBLE 1 //气泡图标
#define HOME_ICON_STATUS 2 //工作状态图标
#define HOME_ICON_WIFI 3 //wifi图标
#define HOME_ICON_BATTERY 4//电池电量
#define HOME_ICON_AC 5 //交流电图标 
#define HOME_BTN_SPEED_ADD 6 //速度+
#define HOME_BTN_SPEED_DEC 7 //速度-
#define HOME_BTN_TEMP_ADD 8 //温度+
#define HOME_BTN_TEMP_DEC 9 //温度-
#define HOME_BTN_REFRESH 10 //压力刷新
#define HOME_BTN_HISTORY 11//历史记录
#define HOME_BTN_SETTING 12 // 设置
#define HOME_RTC         13//RTC
#define HOME_TXT_BATTERY 14 //电池电量
#define HOME_TXT_SPEED_CUR 15 //当前速度
#define HOME_TXT_YINLIU_1 16//引流量1
#define HOME_TXT_YINLIU_2 17//引流量2
#define HOME_TXT_FLOW1 18//流量1
#define HOME_TXT_FLOW2 19//流量2
#define HOME_TXT_FLOW3 20 //流量3
#define HOME_TXT_RESP 21//呼吸机RESP
#define HOME_TXT_RAP 22//呼吸机RAP
#define HOME_TXT_RES_1 23//阻力指数1
#define HOME_TXT_RES_2 24//阻力指数2
#define HOME_TXT_PRE_1 25//静脉压
#define HOME_TXT_PRE_2 26//动脉压
#define HOME_TXT_TEMP_ORGAN 27//器官温度
#define HOME_TXT_TEMP_BLOOD 28//许爷温度
#define HOME_TXT_TEMP_SET 29//设置温度
#define HOME_TXT_LIQ 30//液位百分比
#define HOME_PROGRESS_LIQ 31//液位比例进度条
#define HOME_TXT_SET_SPEED 32//设置速度
#define HOME_TXT_TIME 33//持续灌注时间
#define HOME_TXT_BUBBLE 34//气泡大小
#define HOME_BTN_STATUS 35 //更改器官图标

#define DATE_BTN_BACK 11 //返回按键
#define DATE_BTN_SET 6//确定按键
#define DATE_BTN_TIME 1//设置时间
#define DATE_SELECTOR_YEAR 2//年
#define DATE_SELECTOR_MON 7//月
#define DATE_SELECTOR_DAY 8//日
#define DATE_TXT_BATTERY 14//电池电量百分比
#define DATE_RTC 13//RTC
#define DATE_ICON_WIFI 3
#define DATE_ICON_BATTERY 4
#define DATE_ICON_AC 5

#define TIME_BTN_BACK 11 //返回按键
#define TIME_BTN_SET 6//确定按键
#define TIME_BTN_TIME 1//设置时间
#define TIME_SELECTOR_HOUR 2//时
#define TIME_SELECTOR_MIN 7//分
#define TIME_SELECTOR_SEC 8//秒
#define TIME_TXT_BATTERY 14//电池电量百分比
#define TIME_RTC 13//RTC
#define TIME_ICON_WIFI 3
#define TIME_ICON_BATTERY 4
#define TIME_ICON_AC 5

#define CURVE_CURVE_DONG 1//动脉压
#define CURVE_CURVE_JING 2//静脉压
#define CURVE_CURVE_FLOW 6//流量
#define CURVE_CURVE_RES 7 //阻力指数

#define PSW_BTN_0    6
#define PSW_BTN_1    5
#define PSW_BTN_2    10
#define PSW_BTN_3    7
#define PSW_BTN_4    4
#define PSW_BTN_5    11
#define PSW_BTN_6    8
#define PSW_BTN_7    3
#define PSW_BTN_8    12
#define PSW_BTN_9    9
#define PSW_BTN_BACK 60
#define PSW_BTN_DEL  13
#define PSW_TXT_PSW  33
#define PSW_PNG_ERR  1

#define PSW_DISPLAY_TRUE 0//显示真实值或者*

typedef enum
{
	DATA_TYPE_DC = 0x01,
	DATA_TYPE_DR = 0x02,
	DATA_TYPE_DA = 0x03,
	DATA_TYPE_DD = 0x04
}IBP_DATA_TYPE;
//注意对齐方式
#pragma pack(1)
typedef struct
{
	uint8_t head;//帧头，0xfa
	uint8_t length;//包长度
	uint8_t parType;//标识不同模块,IBP为0x07
	uint8_t dataType;//数据包类型,命令类型
	uint8_t dataID;//数据包ID,命令码
	uint32_t dataSN;//数据包序列号，小端
	uint8_t data[255];//数据内容
	uint8_t chkCum;//校验码
}IBP_DATA_FRAME;//IBP数据帧
#pragma pack()

#define IBP_QUE_DEL			ibpCmdQue.readIndex ++; \
												if(ibpCmdQue.readIndex == OP_MAX) \
												{ \
													ibpCmdQue.readIndex = 0; \
												} \
												ibpCmdQue.length --;

typedef enum
{
	IBP_OP_HAND_SHAKE = 0x01,//上电握手
	IBP_OP_INFO = 0x02,//查询模块信息
	IBP_OP_ADJUST_SET_ZERO = 0x03,//校0，校准
	IBP_OP_SET_HUMAN = 0x04,//设置病人类型
	IBP_OP_SET_TIME = 0x05,//IBP平均时间设置
	IBP_OP_ADJUST_INFO = 0x06,//校0/校准信息查询
	IBP_OP_SET_FILTER = 0x07,//IBP滤波设置
	IBP_OP_SET_MARK = 0x09,//IBP表明设置
	IBP_OP_SET_ADJUST_TIME = 0x0c,//校零校准时间设置
	IBP_OP_IAP = 0x7f,//在线升级
}IBP_OP;//操作命令
#define OP_MAX 5//最多缓存命令条数
typedef enum
{
	OP_STATUS_FREE,//还没发
	OP_STATUS_WAIT_ACK,//等待ack
	OP_STATUS_DONE,//完成
}OP_CMD_STATUS;
typedef struct
{
	uint8_t status;
	uint16_t timer;//用于定时，没有收到响应的时候重发
	IBP_OP op;
	uint8_t cmdPar[7];//最多7个字节命令参数
}IBP_CMD;

typedef struct
{
	IBP_CMD opCmd[OP_MAX];
	uint8_t readIndex;
	uint8_t writeIndex;
	uint8_t length;
}IBP_OP_QUE;



#define MessageScreen 0x01//页面切换类消息
#define MessageControl 0x11 //控件类消息

#define NOTIFY_TOUCH_PRESS      0X01   //触摸屏按下通知
#define NOTIFY_TOUCH_RELEASE  0X03  //触摸屏松开通知
#define NOTIFY_WRITE_FLASH_OK  0X0C  //写FLASH成功
#define NOTIFY_WRITE_FLASH_FAILD  0X0D  //写FLASH失败
#define NOTIFY_READ_FLASH_OK  0X0B  //读FLASH成功
#define NOTIFY_READ_FLASH_FAILD  0X0F  //读FLASH失败
#define NOTIFY_MENU                        0X14  //菜单事件通知
#define NOTIFY_TIMER                       0X43  //定时器超时通知
#define NOTIFY_CONTROL                0XB1  //控件更新通知
#define NOTIFY_RTC                    0xf7 //RTC时间

//根据串口屏数据格式定义此结构体
#pragma pack(push)
#pragma pack(1)
typedef struct
{
	uint8_t cmdHead;//帧头
	uint8_t cmdType;//命令类型
	uint8_t ctrlMsg;//消息类型
	uint16_t secreenId;//产生消息的画面ID
	uint16_t controlId;//产生消息的控件ID
	uint8_t controlType;//控件类型
	uint8_t param[256];//可变长度参数，最多256个字节
	uint8_t cmdTail[4];//帧尾
}CTRL_MSG,*PCTRL_MSG;
#pragma pack(pop)
#define PTR2U16(PTR) ((((uint8_t *)(PTR))[0]<<8)|((uint8_t *)(PTR))[1])  //从缓冲区取16位数据
#define PTR2U32(PTR) ((((uint8_t *)(PTR))[0]<<24)|(((uint8_t *)(PTR))[1]<<16)|(((uint8_t *)(PTR))[2]<<8)|((uint8_t *)(PTR))[3])  //从缓冲区取32位数据

enum CtrlType
{
	kCtrlUnkonw = 0x0,//未知控件
	kCtrlButton = 0x10,//按钮控件
	kCtrlText,//文本控件
	kCtrlProgress,  //进度条
	kCtrlSlider,    //滑动条
	kCtrlMeter,  //仪表
	kCtrlDropList, //下拉列表
	kCtrlAnimation, //动画
	kCtrlRTC, //时间显示
	kCtrlGraph, //曲线图控件
	kCtrlTable, //表格控件
	kCtrlMenu,//菜单控件
	kCtrlSelector,//选择控件
	kCtrlQRCode//二维码
};

#define HMI_MEX_RECEIVE_LENGTH 30
typedef struct
{
	uint8_t buf[HMI_MEX_RECEIVE_LENGTH];
	uint8_t length;
}HMI_DATA_PER_FRAME;
#define HMI_MAX_CMD 10//最多缓冲10条指令,理论上用不完的
typedef struct
{
	uint8_t readIndex;
	uint8_t writeIndex;
	HMI_DATA_PER_FRAME cmd[HMI_MAX_CMD];
}HMI_FRAME;
#define PSW_BUF_LENGTH  3 //密码位数
typedef struct
{
	uint8_t inputIndex;//当前输入位置
	uint8_t pswBuf[PSW_BUF_LENGTH];//密码字符数
}PSW_DATA;//密码相关
void UsartSendData(uint8_t indexUart);
void UsartSetTxBuf(uint8_t *txBuf,uint8_t length,uint8_t indexUart);
uint8_t IBPChkSum(uint8_t *txBuf);
void IBPDataAnalysis(void);
uint8_t IBPCmdAdd(IBP_OP op,uint8_t *par);
void IBPCmdSend(void);
void FlowDealt(void);
void BatteryDealt(void);
//流量计获取信息
uint16_t MakeCrcModbus(uint8_t *str,uint8_t length);
//超声波流量计CRC
uint8_t MakeCrcFlow(uint8_t *buf);
//100ms一次
void FLowGetData(void);
void BatteryGetData(void);
void FlowSetBuble(void);
void ChangeTxtValue(uint16_t screen,uint16_t id,uint8_t *value,uint8_t isFloat);
void ChangeTxt(uint16_t screen,uint16_t id,uint8_t *txt,uint8_t len);
void ChangeCurveRange(uint16_t screen,uint16_t id,int32_t min,int32_t max);
void ChangeScreen(uint16_t screenId);
void ClearTxtContent(uint16_t screen,uint8_t control);
void SetCtrlVisable(uint16_t screen,uint16_t id,uint8_t vis);
void ChangeSelectorIndex(uint16_t screenId,uint16_t controlId,uint8_t index);
void HmiCmdDealt(void);//触摸屏命令解析
void ChangeIconFrame(uint16_t screen,uint16_t id,uint8_t value);
void ChangeSetSpeed(uint8_t flag,uint8_t value);
void AddHistorySampleDateFloat(uint16_t screen,uint16_t id,uint8_t *val);
void AddHistorySampleDateFloatTwice(uint16_t screen,uint16_t id,uint8_t *val1,uint8_t *val2);
void HmiAdjustDected(void);
void ChangeSetTemp(uint8_t flag,float value);
void GetRTCFromHMI(void);
void SetRTCToHmi(void);
void HmiWorkTimerRefresh(void);
void BubbleTimer(void);
void BatteryUpDate(void);
void AdcSetSpeed(void);
void SetScreenCurveRange(void);
void ChangePSWTxt(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

