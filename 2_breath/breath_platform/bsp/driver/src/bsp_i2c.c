/**@file   bsp_i2c.c
* @brief   Ӳ��I2C����,�ж�
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_i2c.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

I2C_HandleTypeDef i2c1_handle;  //i2c���
I2C_HandleTypeDef i2c2_handle;  //i2c���
I2C_HandleTypeDef i2c3_handle;  //i2c���

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/*I2C1================================================================================================*/
/**
* @brief  i2c�ȴ�����
*/
void i2c1_wait(void)
{
    while(i2c1_handle.State != HAL_I2C_STATE_READY);
}
/**
* @brief  i2cд
*/
void i2c1_write(uint8_t *buf, uint16_t len, uint8_t addr_dev)
{
    HAL_I2C_Master_Transmit_IT(&i2c1_handle, addr_dev, buf, len);
}
/**
* @brief  i2c��
*/
void i2c1_read(uint8_t *buf, uint16_t len, uint8_t addr_dev)
{
    HAL_I2C_Master_Receive_IT(&i2c1_handle, addr_dev, buf, len);
}
/**
* @brief  i2c��ʼ��
*/
void i2c1_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_I2C_DeInit(&i2c1_handle);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    /* IO���� */
    gpio_init_struct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;           //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_OD;      //ģʽ,!!!�˴����뿪©��������ͨ��!!!
    gpio_init_struct.Pull      = GPIO_PULLUP;//GPIO_PULLUP;GPIO_NOPULL          //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;  //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF4_I2C1;        //����
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);  //��ʼ��
    /* i2c���� */
    i2c1_handle.Instance              = I2C1;                     //�Ĵ�����ַ
    i2c1_handle.Init.Timing           = 0x00702991;//0x00300EAC;//0x00702991;//0x00B03FDB;               //������
    i2c1_handle.Init.OwnAddress1      = 0;                        //�Լ��ĵ�һ���豸��ַ
    i2c1_handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;  //ѡ��7λ��10λѰַģʽ
    i2c1_handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;  //�Ƿ�ѡ��˫Ѱַģʽ
    i2c1_handle.Init.OwnAddress2      = 0;                        //�Լ��ĵڶ����豸��ַ(˫Ѱַģʽ)
    i2c1_handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;           //��ѡ��˫Ѱַ��ʽʱ��ȷ�������ַΪ����ڶ��豸��ַ
    i2c1_handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;  //�Ƿ�ʹ�ù㲥
    i2c1_handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;    //ָ���Ƿ�ѡ��nostretchģʽ
    HAL_I2C_Init(&i2c1_handle);  //��ʼ��
    
    HAL_I2CEx_ConfigAnalogFilter(&i2c1_handle, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&i2c1_handle, 0);
    
    /* i2c�ж����� */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 3, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);          //ʹ���ж�
}
/* I2C�¼��ж� */
void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2c1_handle);
}
/* I2C�����ж� */
void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2c1_handle);
}
/*I2C1*************************************************************************************************/
/*I2C2================================================================================================*/
/**
* @brief  i2c�ȴ�����
*/
void i2c2_wait(void)
{
    while(i2c2_handle.State != HAL_I2C_STATE_READY);
}
/**
* @brief  i2cд
*/
void i2c2_write(uint8_t *buf, uint16_t len, uint8_t addr_dev)
{
    HAL_I2C_Master_Transmit_IT(&i2c2_handle, addr_dev, buf, len);
}
/**
* @brief  i2c��
*/
void i2c2_read(uint8_t *buf, uint16_t len, uint8_t addr_dev)
{
    HAL_I2C_Master_Receive_IT(&i2c2_handle, addr_dev, buf, len);
}
/**
* @brief  i2c��ʼ��
*/
void i2c2_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_I2C_DeInit(&i2c2_handle);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    /* IO���� */
    gpio_init_struct.Pin       = GPIO_PIN_10 | GPIO_PIN_11; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_OD;      //ģʽ,!!!�˴����뿪©��������ͨ��!!!
    gpio_init_struct.Pull      = GPIO_PULLUP;          //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;  //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF4_I2C2;        //����
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);  //��ʼ��
    /* i2c���� */
    i2c2_handle.Instance              = I2C2;                     //�Ĵ�����ַ
    i2c2_handle.Init.Timing           = 0x00702991;               //������
    i2c2_handle.Init.OwnAddress1      = 0;                        //�Լ��ĵ�һ���豸��ַ
    i2c2_handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;  //ѡ��7λ��10λѰַģʽ
    i2c2_handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;  //�Ƿ�ѡ��˫Ѱַģʽ
    i2c2_handle.Init.OwnAddress2      = 0;                        //�Լ��ĵڶ����豸��ַ(˫Ѱַģʽ)
    i2c2_handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;           //��ѡ��˫Ѱַ��ʽʱ��ȷ�������ַΪ����ڶ��豸��ַ
    i2c2_handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;  //�Ƿ�ʹ�ù㲥
    i2c2_handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;    //ָ���Ƿ�ѡ��nostretchģʽ
    HAL_I2C_Init(&i2c2_handle);  //��ʼ��
    /* i2c�ж����� */
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 1, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 1, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);          //ʹ���ж�
}
/* I2C�¼��ж� */
void I2C2_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2c2_handle);
}
/* I2C�����ж� */
void I2C2_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2c2_handle);
}
/*I2C2*************************************************************************************************/
/*I2C3================================================================================================*/
/**
* @brief  i2cд
*/
void i2c3_write(uint8_t *buf, uint16_t len, uint8_t addr_dev)
{
    HAL_I2C_Master_Transmit_IT(&i2c3_handle, addr_dev, buf, len);
}
/**
* @brief  i2c��
*/
void i2c3_read(uint8_t *buf, uint16_t len, uint8_t addr_dev)
{
    HAL_I2C_Master_Receive_IT(&i2c3_handle, addr_dev, buf, len);
}
/**
* @brief  i2c��ʼ��
*/
void i2c3_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    HAL_I2C_DeInit(&i2c3_handle);
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_I2C3_CLK_ENABLE();
    /* IO���� */
    gpio_init_struct.Pin       = GPIO_PIN_0 | GPIO_PIN_1; //���ź�
    gpio_init_struct.Mode      = GPIO_MODE_AF_OD;      //ģʽ,!!!�˴����뿪©��������ͨ��!!!
    gpio_init_struct.Pull      = GPIO_PULLUP;          //����������
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;  //IO��������·��Ӧ�ٶ�
    gpio_init_struct.Alternate = GPIO_AF4_I2C3;        //����
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);  //��ʼ��
    /* i2c���� */
    i2c3_handle.Instance              = I2C3;                     //�Ĵ�����ַ
    i2c3_handle.Init.Timing           = 0x00702991;               //������
    i2c3_handle.Init.OwnAddress1      = 0;                        //�Լ��ĵ�һ���豸��ַ
    i2c3_handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;  //ѡ��7λ��10λѰַģʽ
    i2c3_handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;  //�Ƿ�ѡ��˫Ѱַģʽ
    i2c3_handle.Init.OwnAddress2      = 0;                        //�Լ��ĵڶ����豸��ַ(˫Ѱַģʽ)
    i2c3_handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;           //��ѡ��˫Ѱַ��ʽʱ��ȷ�������ַΪ����ڶ��豸��ַ
    i2c3_handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;  //�Ƿ�ʹ�ù㲥
    i2c3_handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;    //ָ���Ƿ�ѡ��nostretchģʽ
    HAL_I2C_Init(&i2c3_handle);  //��ʼ��
    /* i2c�ж����� */
    HAL_NVIC_SetPriority(I2C3_EV_IRQn, 1, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);          //ʹ���ж�
    HAL_NVIC_SetPriority(I2C3_ER_IRQn, 1, 0);  //��ռ���ȼ��������ȼ�
    HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);          //ʹ���ж�
}
/* I2C�¼��ж� */
void I2C3_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2c3_handle);
}
/* I2C�����ж� */
void I2C3_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2c3_handle);
}
/*I2C3*************************************************************************************************/

