/**@file   bsp_io.c
* @brief   IO������
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_io.h"

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
* @brief  LEDС��
* @attention 
*/
void BspLedInit(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_StructInit(&GPIO_InitStruct);
    /* IO���� */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_13;  //���ź�
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;      //ģʽ
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //���ģʽ
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //����������
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO��������·��Ӧ�ٶ�
    GPIO_Init(GPIOC, &GPIO_InitStruct);  //��ʼ��
    /* ���� */
    LED_RUN_OFF;
}
/**
* @brief  ���IO��
* @attention 
*/
void BspOutputIOInit(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_StructInit(&GPIO_InitStruct);
    /* IO���� */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;  //���ź�
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;      //ģʽ
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //���ģʽ
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //����������
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO��������·��Ӧ�ٶ�
    GPIO_Init(GPIOB, &GPIO_InitStruct);  //��ʼ��
    
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8;  //���ź�
    GPIO_Init(GPIOC, &GPIO_InitStruct);  //��ʼ��
    /* ���� */
    POWER_ON;
    OUT1_OFF;
    OUT2_OFF;
    OUT3_OFF;
    OUT4_OFF;
    RS485_RX;
}
/**
* @brief  ����IO��
* @attention 
*/
void BspInputIOInit(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_StructInit(&GPIO_InitStruct);
    /* IO���� */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;   //���ź�
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;  //ģʽ
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;  //����
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO��������·��Ӧ�ٶ�
    GPIO_Init(GPIOB, &GPIO_InitStruct);  //��ʼ��
}
/**
* @brief  IO�ڿ��Ƴ�ʼ��
* @attention 
*/
void BspIOInit(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);  //ʹ��GPIO��ʱ��
    
    BspLedInit();
    BspOutputIOInit();
    BspInputIOInit();
}
/**
* @brief  IO�ڿ���
*/
uint8_t BspIOCtrl(uint16_t id, uint8_t cmd)
{
    switch(id)
    {
        case IO_LED_RUN : if(cmd) LED_RUN_ON; else LED_RUN_OFF; break;
        case IO_POWER_ON: if(cmd) POWER_ON; else POWER_OFF; break;
        case IO_OUT1_ON : if(cmd) OUT1_ON; else OUT1_OFF; break;
        case IO_OUT2_ON : if(cmd) OUT2_ON; else OUT2_OFF; break;
        case IO_OUT3_ON : if(cmd) OUT3_ON; else OUT3_OFF; break;
        case IO_OUT4_ON : if(cmd) OUT4_ON; else OUT4_OFF; break;
        case IO_RS485_TX: if(cmd) RS485_TX; else RS485_RX; break;
        case IO_READ_IN1: return READ_IN1;
        case IO_READ_IN2: return READ_IN2;
        default: break;
    }
}

