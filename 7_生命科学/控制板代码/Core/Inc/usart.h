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
#define SCREEN_HOME_PAGE 0 //��ҳ��
#define SCREEN_SET_DATE 1 //��������
#define SCREEN_HISTORY  2 //��ʷ��¼
#define SCREEN_SET_TIME 3 //����ʱ��
#define SCREEN_POWER_ON 4 //��������
#define SCREEN_CURVE    5 //��ʷ��¼����
#define SCREEN_PSW      6 //�����������


#define HOME_ICON_BUBBLE 1 //����ͼ��
#define HOME_ICON_STATUS 2 //����״̬ͼ��
#define HOME_ICON_WIFI 3 //wifiͼ��
#define HOME_ICON_BATTERY 4//��ص���
#define HOME_ICON_AC 5 //������ͼ�� 
#define HOME_BTN_SPEED_ADD 6 //�ٶ�+
#define HOME_BTN_SPEED_DEC 7 //�ٶ�-
#define HOME_BTN_TEMP_ADD 8 //�¶�+
#define HOME_BTN_TEMP_DEC 9 //�¶�-
#define HOME_BTN_REFRESH 10 //ѹ��ˢ��
#define HOME_BTN_HISTORY 11//��ʷ��¼
#define HOME_BTN_SETTING 12 // ����
#define HOME_RTC         13//RTC
#define HOME_TXT_BATTERY 14 //��ص���
#define HOME_TXT_SPEED_CUR 15 //��ǰ�ٶ�
#define HOME_TXT_YINLIU_1 16//������1
#define HOME_TXT_YINLIU_2 17//������2
#define HOME_TXT_FLOW1 18//����1
#define HOME_TXT_FLOW2 19//����2
#define HOME_TXT_FLOW3 20 //����3
#define HOME_TXT_RESP 21//������RESP
#define HOME_TXT_RAP 22//������RAP
#define HOME_TXT_RES_1 23//����ָ��1
#define HOME_TXT_RES_2 24//����ָ��2
#define HOME_TXT_PRE_1 25//����ѹ
#define HOME_TXT_PRE_2 26//����ѹ
#define HOME_TXT_TEMP_ORGAN 27//�����¶�
#define HOME_TXT_TEMP_BLOOD 28//��ү�¶�
#define HOME_TXT_TEMP_SET 29//�����¶�
#define HOME_TXT_LIQ 30//Һλ�ٷֱ�
#define HOME_PROGRESS_LIQ 31//Һλ����������
#define HOME_TXT_SET_SPEED 32//�����ٶ�
#define HOME_TXT_TIME 33//������עʱ��
#define HOME_TXT_BUBBLE 34//���ݴ�С
#define HOME_BTN_STATUS 35 //��������ͼ��

#define DATE_BTN_BACK 11 //���ذ���
#define DATE_BTN_SET 6//ȷ������
#define DATE_BTN_TIME 1//����ʱ��
#define DATE_SELECTOR_YEAR 2//��
#define DATE_SELECTOR_MON 7//��
#define DATE_SELECTOR_DAY 8//��
#define DATE_TXT_BATTERY 14//��ص����ٷֱ�
#define DATE_RTC 13//RTC
#define DATE_ICON_WIFI 3
#define DATE_ICON_BATTERY 4
#define DATE_ICON_AC 5

#define TIME_BTN_BACK 11 //���ذ���
#define TIME_BTN_SET 6//ȷ������
#define TIME_BTN_TIME 1//����ʱ��
#define TIME_SELECTOR_HOUR 2//ʱ
#define TIME_SELECTOR_MIN 7//��
#define TIME_SELECTOR_SEC 8//��
#define TIME_TXT_BATTERY 14//��ص����ٷֱ�
#define TIME_RTC 13//RTC
#define TIME_ICON_WIFI 3
#define TIME_ICON_BATTERY 4
#define TIME_ICON_AC 5

#define CURVE_CURVE_DONG 1//����ѹ
#define CURVE_CURVE_JING 2//����ѹ
#define CURVE_CURVE_FLOW 6//����
#define CURVE_CURVE_RES 7 //����ָ��

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

#define PSW_DISPLAY_TRUE 0//��ʾ��ʵֵ����*

typedef enum
{
	DATA_TYPE_DC = 0x01,
	DATA_TYPE_DR = 0x02,
	DATA_TYPE_DA = 0x03,
	DATA_TYPE_DD = 0x04
}IBP_DATA_TYPE;
//ע����뷽ʽ
#pragma pack(1)
typedef struct
{
	uint8_t head;//֡ͷ��0xfa
	uint8_t length;//������
	uint8_t parType;//��ʶ��ͬģ��,IBPΪ0x07
	uint8_t dataType;//���ݰ�����,��������
	uint8_t dataID;//���ݰ�ID,������
	uint32_t dataSN;//���ݰ����кţ�С��
	uint8_t data[255];//��������
	uint8_t chkCum;//У����
}IBP_DATA_FRAME;//IBP����֡
#pragma pack()

#define IBP_QUE_DEL			ibpCmdQue.readIndex ++; \
												if(ibpCmdQue.readIndex == OP_MAX) \
												{ \
													ibpCmdQue.readIndex = 0; \
												} \
												ibpCmdQue.length --;

typedef enum
{
	IBP_OP_HAND_SHAKE = 0x01,//�ϵ�����
	IBP_OP_INFO = 0x02,//��ѯģ����Ϣ
	IBP_OP_ADJUST_SET_ZERO = 0x03,//У0��У׼
	IBP_OP_SET_HUMAN = 0x04,//���ò�������
	IBP_OP_SET_TIME = 0x05,//IBPƽ��ʱ������
	IBP_OP_ADJUST_INFO = 0x06,//У0/У׼��Ϣ��ѯ
	IBP_OP_SET_FILTER = 0x07,//IBP�˲�����
	IBP_OP_SET_MARK = 0x09,//IBP��������
	IBP_OP_SET_ADJUST_TIME = 0x0c,//У��У׼ʱ������
	IBP_OP_IAP = 0x7f,//��������
}IBP_OP;//��������
#define OP_MAX 5//��໺����������
typedef enum
{
	OP_STATUS_FREE,//��û��
	OP_STATUS_WAIT_ACK,//�ȴ�ack
	OP_STATUS_DONE,//���
}OP_CMD_STATUS;
typedef struct
{
	uint8_t status;
	uint16_t timer;//���ڶ�ʱ��û���յ���Ӧ��ʱ���ط�
	IBP_OP op;
	uint8_t cmdPar[7];//���7���ֽ��������
}IBP_CMD;

typedef struct
{
	IBP_CMD opCmd[OP_MAX];
	uint8_t readIndex;
	uint8_t writeIndex;
	uint8_t length;
}IBP_OP_QUE;



#define MessageScreen 0x01//ҳ���л�����Ϣ
#define MessageControl 0x11 //�ؼ�����Ϣ

#define NOTIFY_TOUCH_PRESS      0X01   //����������֪ͨ
#define NOTIFY_TOUCH_RELEASE  0X03  //�������ɿ�֪ͨ
#define NOTIFY_WRITE_FLASH_OK  0X0C  //дFLASH�ɹ�
#define NOTIFY_WRITE_FLASH_FAILD  0X0D  //дFLASHʧ��
#define NOTIFY_READ_FLASH_OK  0X0B  //��FLASH�ɹ�
#define NOTIFY_READ_FLASH_FAILD  0X0F  //��FLASHʧ��
#define NOTIFY_MENU                        0X14  //�˵��¼�֪ͨ
#define NOTIFY_TIMER                       0X43  //��ʱ����ʱ֪ͨ
#define NOTIFY_CONTROL                0XB1  //�ؼ�����֪ͨ
#define NOTIFY_RTC                    0xf7 //RTCʱ��

//���ݴ��������ݸ�ʽ����˽ṹ��
#pragma pack(push)
#pragma pack(1)
typedef struct
{
	uint8_t cmdHead;//֡ͷ
	uint8_t cmdType;//��������
	uint8_t ctrlMsg;//��Ϣ����
	uint16_t secreenId;//������Ϣ�Ļ���ID
	uint16_t controlId;//������Ϣ�Ŀؼ�ID
	uint8_t controlType;//�ؼ�����
	uint8_t param[256];//�ɱ䳤�Ȳ��������256���ֽ�
	uint8_t cmdTail[4];//֡β
}CTRL_MSG,*PCTRL_MSG;
#pragma pack(pop)
#define PTR2U16(PTR) ((((uint8_t *)(PTR))[0]<<8)|((uint8_t *)(PTR))[1])  //�ӻ�����ȡ16λ����
#define PTR2U32(PTR) ((((uint8_t *)(PTR))[0]<<24)|(((uint8_t *)(PTR))[1]<<16)|(((uint8_t *)(PTR))[2]<<8)|((uint8_t *)(PTR))[3])  //�ӻ�����ȡ32λ����

enum CtrlType
{
	kCtrlUnkonw = 0x0,//δ֪�ؼ�
	kCtrlButton = 0x10,//��ť�ؼ�
	kCtrlText,//�ı��ؼ�
	kCtrlProgress,  //������
	kCtrlSlider,    //������
	kCtrlMeter,  //�Ǳ�
	kCtrlDropList, //�����б�
	kCtrlAnimation, //����
	kCtrlRTC, //ʱ����ʾ
	kCtrlGraph, //����ͼ�ؼ�
	kCtrlTable, //���ؼ�
	kCtrlMenu,//�˵��ؼ�
	kCtrlSelector,//ѡ��ؼ�
	kCtrlQRCode//��ά��
};

#define HMI_MEX_RECEIVE_LENGTH 30
typedef struct
{
	uint8_t buf[HMI_MEX_RECEIVE_LENGTH];
	uint8_t length;
}HMI_DATA_PER_FRAME;
#define HMI_MAX_CMD 10//��໺��10��ָ��,�������ò����
typedef struct
{
	uint8_t readIndex;
	uint8_t writeIndex;
	HMI_DATA_PER_FRAME cmd[HMI_MAX_CMD];
}HMI_FRAME;
#define PSW_BUF_LENGTH  3 //����λ��
typedef struct
{
	uint8_t inputIndex;//��ǰ����λ��
	uint8_t pswBuf[PSW_BUF_LENGTH];//�����ַ���
}PSW_DATA;//�������
void UsartSendData(uint8_t indexUart);
void UsartSetTxBuf(uint8_t *txBuf,uint8_t length,uint8_t indexUart);
uint8_t IBPChkSum(uint8_t *txBuf);
void IBPDataAnalysis(void);
uint8_t IBPCmdAdd(IBP_OP op,uint8_t *par);
void IBPCmdSend(void);
void FlowDealt(void);
void BatteryDealt(void);
//�����ƻ�ȡ��Ϣ
uint16_t MakeCrcModbus(uint8_t *str,uint8_t length);
//������������CRC
uint8_t MakeCrcFlow(uint8_t *buf);
//100msһ��
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
void HmiCmdDealt(void);//�������������
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

