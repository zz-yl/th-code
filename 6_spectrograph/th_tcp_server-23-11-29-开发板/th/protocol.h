/**@file   protocol.h
* @brief   
* @author  ��׿��
* @date    2023/11/28
* @version 1.00.0.0
**************************************************************************************************/

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#pragma pack(push, 4)

/**
* @struct  cmd_t
* @brief   ָ��,��λ��=>��λ��
*/
typedef struct
{
    unsigned char spectrometer;  //������,1:����;0:�ر�
    unsigned char halogen;       //±���,1:����;0:�ر�
    unsigned char led;           //LED��,1:����;0:�ر�
    unsigned int  led_level;     //LED��ǿ��,0~5000:0%~100%
}cmd_t;

/**
* @struct  msg_t
* @brief   ��Ϣ,��λ��=>��λ��
*/
typedef struct
{
    unsigned char state_spectrometer;  //������״̬����,1:����;0:�ر�
    unsigned char state_halogen;       //±���״̬����,1:����;0:�ر�
    unsigned char state_led;           //LED��״̬����,1:����;0:�ر�
    unsigned int  state_led_level;     //LED��ǿ�ȷ���,0~5000:0%~100%
    unsigned char footrest1;           //��̤1,1:����;0:̧��
    unsigned char footrest2;           //��̤2,1:����;0:̧��
    unsigned int  tim;                 //ʱ��(ms),�������0��ʼ��ʱ,�����(Լ49.71��)���´�0��ʱ
    unsigned int  frame_number;        //֡��,���͸���λ��������֡���,��0��ʼ����,�����(2^32)���¼���
}msg_t;

#pragma pack(pop)

#endif /* PROTOCOL_H_ */
