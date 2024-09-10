/**@file   bsp_motor.c
* @brief   ������������������,����IO��PWM
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_motor.h"
#include "bsp_cfg.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define TIM_STEP_ARR_INIT  (1000-1)
#define TIM_STEP_PSC_INIT  (1-1)

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

TIM_HandleTypeDef motor1_pwm_handle = {0};
TIM_HandleTypeDef motor2_pwm_handle = {0};
TIM_HandleTypeDef motor3_pwm_handle = {0};
TIM_HandleTypeDef motor4_pwm_handle = {0};
TIM_HandleTypeDef motor5_pwm_handle = {0};

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  ���1����Ƶ�ʿ���
* @attention 
*/
void motor1_step_frq(float frq)
{
    float tmp = 0;
    uint32_t arr = 0;
    uint32_t cmp = 0;
    
    if(frq > 0)
    {
        tmp = TIM_CLOCK / frq;
        arr = (uint32_t)(tmp - 1);
        cmp = (uint32_t)(tmp / 2);
    }
    else
    {
        cmp = 0;
    }
    __HAL_TIM_SET_AUTORELOAD(&motor1_pwm_handle, arr);
    __HAL_TIM_SET_COUNTER(&motor1_pwm_handle, 0);  //���������¼���
    __HAL_TIM_SET_COMPARE(&motor1_pwm_handle, TIM_CHANNEL_4, cmp);  //ռ�ձ�50%
}
/**
* @brief  ���2����Ƶ�ʿ���
* @attention 
*/
void motor2_step_frq(float frq)
{
    float tmp = 0;
    uint32_t arr = 0;
    uint32_t cmp = 0;
    
    if(frq > 0)
    {
        tmp = TIM_CLOCK / frq;
        arr = (uint32_t)(tmp - 1);
        cmp = (uint32_t)(tmp / 2);
    }
    else
    {
        cmp = 0;
    }
    __HAL_TIM_SET_AUTORELOAD(&motor2_pwm_handle, arr);
    __HAL_TIM_SET_COUNTER(&motor2_pwm_handle, 0);  //���������¼���
    __HAL_TIM_SET_COMPARE(&motor2_pwm_handle, TIM_CHANNEL_4, cmp);  //ռ�ձ�50%
}
/**
* @brief  ���3����Ƶ�ʿ���
* @attention 
*/
void motor3_step_frq(float frq)
{
    float tmp = 0;
    uint32_t arr = 0;
    uint32_t cmp = 0;
    
    if(frq > 0)
    {
        tmp = TIM_CLOCK / frq;
        arr = (uint32_t)(tmp - 1);
        cmp = (uint32_t)(tmp / 2);
    }
    else
    {
        cmp = 0;
    }
    __HAL_TIM_SET_AUTORELOAD(&motor3_pwm_handle, arr);
    __HAL_TIM_SET_COUNTER(&motor3_pwm_handle, 0);  //���������¼���
    __HAL_TIM_SET_COMPARE(&motor3_pwm_handle, TIM_CHANNEL_2, cmp);  //ռ�ձ�50%
}
/**
* @brief  ���4����Ƶ�ʿ���
* @attention 
*/
void motor4_step_frq(float frq)
{
    float tmp = 0;
    uint32_t arr = 0;
    uint32_t cmp = 0;
    
    if(frq > 0)
    {
        tmp = TIM_CLOCK / frq;
        arr = (uint32_t)(tmp - 1);
        cmp = (uint32_t)(tmp / 2);
    }
    else
    {
        cmp = 0;
    }
    __HAL_TIM_SET_AUTORELOAD(&motor4_pwm_handle, arr);
    __HAL_TIM_SET_COUNTER(&motor4_pwm_handle, 0);  //���������¼���
    __HAL_TIM_SET_COMPARE(&motor4_pwm_handle, TIM_CHANNEL_2, cmp);  //ռ�ձ�50%
}
/**
* @brief  ���5����Ƶ�ʿ���
* @attention 
*/
void motor5_step_frq(float frq)
{
    float tmp = 0;
    uint32_t arr = 0;
    uint32_t cmp = 0;
    
    if(frq > 0)
    {
        tmp = TIM_CLOCK / frq;
        arr = (uint32_t)(tmp - 1);
        cmp = (uint32_t)(tmp / 2);
    }
    else
    {
        cmp = 0;
    }
    __HAL_TIM_SET_AUTORELOAD(&motor5_pwm_handle, arr);
    __HAL_TIM_SET_COUNTER(&motor5_pwm_handle, 0);  //���������¼���
    __HAL_TIM_SET_COMPARE(&motor5_pwm_handle, TIM_CHANNEL_3, cmp);  //ռ�ձ�50%
}
/**
* @brief  ���1��ʼ��
* @attention 
*/
void motor1_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_OC_InitTypeDef tim_oc_struct = {0};
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    /* IO���� */
    /* M1 GPIO Configuration
    M1_STEP  ======> PA11->tim1-ch4
    M1_DIR   ======> PA12
    M1_EN    ======> PD3
    M1_SLEEP ======> PD4
    M1_FAULT ======> PD6
    */
    /* out */
    gpio_init_struct.Pin   = GPIO_PIN_12; //���ź�
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;   //ģʽ
    gpio_init_struct.Pull  = GPIO_PULLUP;           //����������
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;  //IO��������·��Ӧ�ٶ�
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);        //��ʼ��
    gpio_init_struct.Pin   = GPIO_PIN_3 | GPIO_PIN_4; //���ź�
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);        //��ʼ��
    /* in */
    gpio_init_struct.Pin   = GPIO_PIN_6;            //���ź�
    gpio_init_struct.Mode  = MODE_INPUT;            //ģʽ
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);        //��ʼ��
    /* pwm */
    gpio_init_struct.Pin   = GPIO_PIN_11;           //���ź�
    gpio_init_struct.Mode  = GPIO_MODE_AF_PP;       //ģʽ
    gpio_init_struct.Alternate = GPIO_AF1_TIM1;     //����
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);        //��ʼ��
    /* PWM���� */
    motor1_pwm_handle.Instance               = TIM1;                   //�Ĵ���
    motor1_pwm_handle.Init.Prescaler         = TIM_STEP_PSC_INIT;      //Ԥ��Ƶϵ��
    motor1_pwm_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;     //��������ģʽ
    motor1_pwm_handle.Init.Period            = TIM_STEP_ARR_INIT;      //�Զ���װ��ֵ
    motor1_pwm_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; //ʱ�ӻ���
    motor1_pwm_handle.Init.RepetitionCounter = 0;                      //�ظ�������
    motor1_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //��װ�ظ���ʱ��,ʹ�ܺ��¸����ڸ���arr,������������
    HAL_TIM_PWM_Init(&motor1_pwm_handle);                              //��ʼ��
    /* PWMͨ������ */
    tim_oc_struct.OCMode       = TIM_OCMODE_PWM1;       //��ʱ��ģʽ
    tim_oc_struct.Pulse        = 0;                     //�Ƚ�ֵ,��ֵ����ȷ��ռ�ձ�
    tim_oc_struct.OCPolarity   = TIM_OCPOLARITY_HIGH;   //�������
    tim_oc_struct.OCNPolarity  = TIM_OCNPOLARITY_HIGH;  //�����������
    tim_oc_struct.OCFastMode   = TIM_OCFAST_DISABLE;    //����ģʽ״̬
    tim_oc_struct.OCIdleState  = TIM_OCIDLESTATE_RESET; //����״̬ʱ��TIM����Ƚ�����״̬
    tim_oc_struct.OCNIdleState = TIM_OCNIDLESTATE_RESET;//����״̬ʱ��TIM����Ƚ�����״̬
    HAL_TIM_PWM_ConfigChannel(&motor1_pwm_handle, &tim_oc_struct, TIM_CHANNEL_4); //����TIMxͨ��y
    /* ���� */
    HAL_TIM_PWM_Start(&motor1_pwm_handle, TIM_CHANNEL_4);  //������ӦPWMͨ��
    M1_DIR(0);
    M1_EN(0);
    M1_SLEEP(1);
}
/**
* @brief  ���2��ʼ��
* @attention 
*/
void motor2_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_OC_InitTypeDef tim_oc_struct = {0};
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* IO���� */
    /* M2 GPIO Configuration
    M2_STEP  ======> PB1->tim3-ch4
    M2_DIR   ======> PE9
    M2_EN    ======> PE10
    M2_SLEEP ======> PC5
    M2_FAULT ======> PC4
    */
    /* out */
    gpio_init_struct.Pin   = GPIO_PIN_5; //���ź�
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;   //ģʽ
    gpio_init_struct.Pull  = GPIO_PULLUP;           //����������
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;  //IO��������·��Ӧ�ٶ�
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);        //��ʼ��
    gpio_init_struct.Pin   = GPIO_PIN_9 | GPIO_PIN_10; //���ź�
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);        //��ʼ��
    /* in */
    gpio_init_struct.Pin   = GPIO_PIN_4;            //���ź�
    gpio_init_struct.Mode  = MODE_INPUT;            //ģʽ
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);        //��ʼ��
    /* pwm */
    gpio_init_struct.Pin   = GPIO_PIN_1;            //���ź�
    gpio_init_struct.Mode  = GPIO_MODE_AF_PP;       //ģʽ
    gpio_init_struct.Alternate = GPIO_AF2_TIM3;     //����
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);        //��ʼ��
    /* PWM���� */
    motor2_pwm_handle.Instance               = TIM3;                   //�Ĵ���
    motor2_pwm_handle.Init.Prescaler         = TIM_STEP_PSC_INIT;      //Ԥ��Ƶϵ��
    motor2_pwm_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;     //��������ģʽ
    motor2_pwm_handle.Init.Period            = TIM_STEP_ARR_INIT;      //�Զ���װ��ֵ
    motor2_pwm_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; //ʱ�ӻ���
    motor2_pwm_handle.Init.RepetitionCounter = 0;                      //�ظ�������
    motor2_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //��װ�ظ���ʱ��,ʹ�ܺ��¸����ڸ���arr,������������
    HAL_TIM_PWM_Init(&motor2_pwm_handle);                              //��ʼ��
    /* PWMͨ������ */
    tim_oc_struct.OCMode       = TIM_OCMODE_PWM1;       //��ʱ��ģʽ
    tim_oc_struct.Pulse        = 0;                     //�Ƚ�ֵ,��ֵ����ȷ��ռ�ձ�
    tim_oc_struct.OCPolarity   = TIM_OCPOLARITY_HIGH;   //�������
    tim_oc_struct.OCNPolarity  = TIM_OCNPOLARITY_HIGH;  //�����������
    tim_oc_struct.OCFastMode   = TIM_OCFAST_DISABLE;    //����ģʽ״̬
    tim_oc_struct.OCIdleState  = TIM_OCIDLESTATE_RESET; //����״̬ʱ��TIM����Ƚ�����״̬
    tim_oc_struct.OCNIdleState = TIM_OCNIDLESTATE_RESET;//����״̬ʱ��TIM����Ƚ�����״̬
    HAL_TIM_PWM_ConfigChannel(&motor2_pwm_handle, &tim_oc_struct, TIM_CHANNEL_4); //����TIMxͨ��y
    /* ���� */
    HAL_TIM_PWM_Start(&motor2_pwm_handle, TIM_CHANNEL_4);  //������ӦPWMͨ��
    M2_DIR(0);
    M2_EN(0);
    M2_SLEEP(1);
}
/**
* @brief  ���3��ʼ��
* @attention 
*/
void motor3_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_OC_InitTypeDef tim_oc_struct = {0};
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    /* IO���� */
    /* M3 GPIO Configuration
    M3_STEP  ======> PB7->tim4-ch2
    M3_DIR   ======> PE4
    M3_EN    ======> PE5
    M3_SLEEP ======> PE6
    M3_FAULT ======> PE2
    */
    /* out */
    gpio_init_struct.Pin   = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6; //���ź�
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;   //ģʽ
    gpio_init_struct.Pull  = GPIO_PULLUP;           //����������
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;  //IO��������·��Ӧ�ٶ�
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);        //��ʼ��
    /* in */
    gpio_init_struct.Pin   = GPIO_PIN_2;            //���ź�
    gpio_init_struct.Mode  = MODE_INPUT;            //ģʽ
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);        //��ʼ��
    /* pwm */
    gpio_init_struct.Pin   = GPIO_PIN_7;            //���ź�
    gpio_init_struct.Mode  = GPIO_MODE_AF_PP;       //ģʽ
    gpio_init_struct.Alternate = GPIO_AF2_TIM4;     //����
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);        //��ʼ��
    /* PWM���� */
    motor3_pwm_handle.Instance               = TIM4;                   //�Ĵ���
    motor3_pwm_handle.Init.Prescaler         = TIM_STEP_PSC_INIT;      //Ԥ��Ƶϵ��
    motor3_pwm_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;     //��������ģʽ
    motor3_pwm_handle.Init.Period            = TIM_STEP_ARR_INIT;      //�Զ���װ��ֵ
    motor3_pwm_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; //ʱ�ӻ���
    motor3_pwm_handle.Init.RepetitionCounter = 0;                      //�ظ�������
    motor3_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //��װ�ظ���ʱ��,ʹ�ܺ��¸����ڸ���arr,������������
    HAL_TIM_PWM_Init(&motor3_pwm_handle);                              //��ʼ��
    /* PWMͨ������ */
    tim_oc_struct.OCMode       = TIM_OCMODE_PWM1;       //��ʱ��ģʽ
    tim_oc_struct.Pulse        = 0;                     //�Ƚ�ֵ,��ֵ����ȷ��ռ�ձ�
    tim_oc_struct.OCPolarity   = TIM_OCPOLARITY_HIGH;   //�������
    tim_oc_struct.OCNPolarity  = TIM_OCNPOLARITY_HIGH;  //�����������
    tim_oc_struct.OCFastMode   = TIM_OCFAST_DISABLE;    //����ģʽ״̬
    tim_oc_struct.OCIdleState  = TIM_OCIDLESTATE_RESET; //����״̬ʱ��TIM����Ƚ�����״̬
    tim_oc_struct.OCNIdleState = TIM_OCNIDLESTATE_RESET;//����״̬ʱ��TIM����Ƚ�����״̬
    HAL_TIM_PWM_ConfigChannel(&motor3_pwm_handle, &tim_oc_struct, TIM_CHANNEL_2); //����TIMxͨ��y
    /* ���� */
    HAL_TIM_PWM_Start(&motor3_pwm_handle, TIM_CHANNEL_2);  //������ӦPWMͨ��
    M3_DIR(0);
    M3_EN(0);
    M3_SLEEP(1);
}
/**
* @brief  ���4��ʼ��
* @attention 
*/
void motor4_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_OC_InitTypeDef tim_oc_struct = {0};
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* IO���� */
    /* M4 GPIO Configuration
    M4_STEP  ======> PA1->tim2-ch2
    M4_DIR   ======> PA3
    M4_EN    ======> PA0
    M4_SLEEP ======> PC3
    M4_FAULT ======> PA2
    */
    /* out */
    gpio_init_struct.Pin   = GPIO_PIN_0 | GPIO_PIN_3; //���ź�
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;   //ģʽ
    gpio_init_struct.Pull  = GPIO_PULLUP;           //����������
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;  //IO��������·��Ӧ�ٶ�
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);        //��ʼ��
    gpio_init_struct.Pin   = GPIO_PIN_3;            //���ź�
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);        //��ʼ��
    /* in */
    gpio_init_struct.Pin   = GPIO_PIN_2;            //���ź�
    gpio_init_struct.Mode  = MODE_INPUT;            //ģʽ
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);        //��ʼ��
    /* pwm */
    gpio_init_struct.Pin   = GPIO_PIN_1;            //���ź�
    gpio_init_struct.Mode  = GPIO_MODE_AF_PP;       //ģʽ
    gpio_init_struct.Alternate = GPIO_AF1_TIM2;     //����
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);        //��ʼ��
    /* PWM���� */
    motor4_pwm_handle.Instance               = TIM2;                   //�Ĵ���
    motor4_pwm_handle.Init.Prescaler         = TIM_STEP_PSC_INIT;      //Ԥ��Ƶϵ��
    motor4_pwm_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;     //��������ģʽ
    motor4_pwm_handle.Init.Period            = TIM_STEP_ARR_INIT;      //�Զ���װ��ֵ
    motor4_pwm_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; //ʱ�ӻ���
    motor4_pwm_handle.Init.RepetitionCounter = 0;                      //�ظ�������
    motor4_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //��װ�ظ���ʱ��,ʹ�ܺ��¸����ڸ���arr,������������
    HAL_TIM_PWM_Init(&motor4_pwm_handle);                              //��ʼ��
    /* PWMͨ������ */
    tim_oc_struct.OCMode       = TIM_OCMODE_PWM1;       //��ʱ��ģʽ
    tim_oc_struct.Pulse        = 0;                     //�Ƚ�ֵ,��ֵ����ȷ��ռ�ձ�
    tim_oc_struct.OCPolarity   = TIM_OCPOLARITY_HIGH;   //�������
    tim_oc_struct.OCNPolarity  = TIM_OCNPOLARITY_HIGH;  //�����������
    tim_oc_struct.OCFastMode   = TIM_OCFAST_DISABLE;    //����ģʽ״̬
    tim_oc_struct.OCIdleState  = TIM_OCIDLESTATE_RESET; //����״̬ʱ��TIM����Ƚ�����״̬
    tim_oc_struct.OCNIdleState = TIM_OCNIDLESTATE_RESET;//����״̬ʱ��TIM����Ƚ�����״̬
    HAL_TIM_PWM_ConfigChannel(&motor4_pwm_handle, &tim_oc_struct, TIM_CHANNEL_2); //����TIMxͨ��y
    /* ���� */
    HAL_TIM_PWM_Start(&motor4_pwm_handle, TIM_CHANNEL_2);  //������ӦPWMͨ��
    M4_DIR(0);
    M4_EN(0);
    M4_SLEEP(1);
}
/**
* @brief  ���5��ʼ��
* @attention 
*/
void motor5_init(void)
{
    /* �ṹ���ʼ�� */
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_OC_InitTypeDef tim_oc_struct = {0};
    /* ʱ��ʹ�� */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    /* IO���� */
    /* M5 GPIO Configuration
    M5_STEP  ======> PC8->tim8-ch3
    M5_DIR   ======> PD10
    M5_EN    ======> PD15
    M5_SLEEP ======> PD13
    M5_FAULT ======> PD14
    */
    /* out */
    gpio_init_struct.Pin   = GPIO_PIN_10 | GPIO_PIN_13 | GPIO_PIN_15; //���ź�
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;   //ģʽ
    gpio_init_struct.Pull  = GPIO_PULLUP;           //����������
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;  //IO��������·��Ӧ�ٶ�
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);        //��ʼ��
    /* in */
    gpio_init_struct.Pin   = GPIO_PIN_14;           //���ź�
    gpio_init_struct.Mode  = MODE_INPUT;            //ģʽ
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);        //��ʼ��
    /* pwm */
    gpio_init_struct.Pin   = GPIO_PIN_8;            //���ź�
    gpio_init_struct.Mode  = GPIO_MODE_AF_PP;       //ģʽ
    gpio_init_struct.Alternate = GPIO_AF3_TIM8;     //����
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);        //��ʼ��
    /* PWM���� */
    motor5_pwm_handle.Instance               = TIM8;                   //�Ĵ���
    motor5_pwm_handle.Init.Prescaler         = TIM_STEP_PSC_INIT;      //Ԥ��Ƶϵ��
    motor5_pwm_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;     //��������ģʽ
    motor5_pwm_handle.Init.Period            = TIM_STEP_ARR_INIT;      //�Զ���װ��ֵ
    motor5_pwm_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; //ʱ�ӻ���
    motor5_pwm_handle.Init.RepetitionCounter = 0;                      //�ظ�������
    motor5_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //��װ�ظ���ʱ��,ʹ�ܺ��¸����ڸ���arr,������������
    HAL_TIM_PWM_Init(&motor5_pwm_handle);                              //��ʼ��
    /* PWMͨ������ */
    tim_oc_struct.OCMode       = TIM_OCMODE_PWM1;       //��ʱ��ģʽ
    tim_oc_struct.Pulse        = 0;                     //�Ƚ�ֵ,��ֵ����ȷ��ռ�ձ�
    tim_oc_struct.OCPolarity   = TIM_OCPOLARITY_HIGH;   //�������
    tim_oc_struct.OCNPolarity  = TIM_OCNPOLARITY_HIGH;  //�����������
    tim_oc_struct.OCFastMode   = TIM_OCFAST_DISABLE;    //����ģʽ״̬
    tim_oc_struct.OCIdleState  = TIM_OCIDLESTATE_RESET; //����״̬ʱ��TIM����Ƚ�����״̬
    tim_oc_struct.OCNIdleState = TIM_OCNIDLESTATE_RESET;//����״̬ʱ��TIM����Ƚ�����״̬
    HAL_TIM_PWM_ConfigChannel(&motor5_pwm_handle, &tim_oc_struct, TIM_CHANNEL_3); //����TIMxͨ��y
    /* ���� */
    HAL_TIM_PWM_Start(&motor5_pwm_handle, TIM_CHANNEL_3);  //������ӦPWMͨ��
    M5_DIR(0);
    M5_EN(0);
    M5_SLEEP(1);
}

