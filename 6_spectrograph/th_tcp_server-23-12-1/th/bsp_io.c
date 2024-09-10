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
void bsp_led_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};

    /* IO���� */
    gpio_init_struct.Pin   = GPIO_PIN_2;        //���ź�
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;     //ģʽ
    gpio_init_struct.Pull  = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_MEDIUM;  //IO��������·��Ӧ�ٶ�
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);  //��ʼ��
    /* ���� */
    IO_LED_RUN(GPIO_PIN_RESET);
}
/**
* @brief  ���IO��
* @attention 
*/
void bsp_output_io_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};

    /* IO���� */
    gpio_init_struct.Pin   = GPIO_PIN_15; //���ź�
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;     //ģʽ
    gpio_init_struct.Pull  = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);  //��ʼ��
    gpio_init_struct.Pin   = GPIO_PIN_10 | GPIO_PIN_11; //���ź�
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);  //��ʼ��
    /* ���� */
    IO_OUT1(0);
    IO_OUT2(0);
    IO_OUT3(0);
}
/**
* @brief  ����IO��
* @attention 
*/
void bsp_input_io_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};

    /* IO���� */
    gpio_init_struct.Pin   = GPIO_PIN_1 | GPIO_PIN_2;              //���ź�
    gpio_init_struct.Mode  = GPIO_MODE_INPUT;         //ģʽ
    gpio_init_struct.Pull  = GPIO_PULLUP;             //����������
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;    //IO��������·��Ӧ�ٶ�
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);  //��ʼ��
}
/**
* @brief  IO�ڿ��Ƴ�ʼ��
* @attention 
*/
void bsp_io_init(void)
{
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    /* IO��ʼ�� */
    bsp_led_init();
    bsp_output_io_init();
    bsp_input_io_init();
}
