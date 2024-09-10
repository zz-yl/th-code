/**@file   bsp_ads131m08.c
* @brief   ads131m08����,spiƵ��25MHz
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_ads131m08.h"
#include "bsp_cfg.h"

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
* @brief  ads131m08 GPIO��ʼ��
* @attention 
*/
void Ads131IOInit(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_StructInit(&GPIO_InitStruct);
    /* ʹ��ʱ�� */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);  //ʹ��GPIO��ʱ��
    /* IO���� */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0;  //���ź�
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;      //ģʽ
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      //���ģʽ
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       //����������
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO��������·��Ӧ�ٶ�
    GPIO_Init(GPIOB, &GPIO_InitStruct);  //��ʼ��
    
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_5;  //���ź�
    GPIO_Init(GPIOC, &GPIO_InitStruct);  //��ʼ��
    
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_4;   //���ź�
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;  //ģʽ
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;  //����
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //IO��������·��Ӧ�ٶ�
    GPIO_Init(GPIOC, &GPIO_InitStruct);  //��ʼ��
    
    /* ���� */
    AD_SYNC_UP;
    AD_CS_UP;
}
/**
* @brief  ads131m08��ʼ��
* @attention 
*/
void Ads131Init(void)
{
    Ads131IOInit();
}
