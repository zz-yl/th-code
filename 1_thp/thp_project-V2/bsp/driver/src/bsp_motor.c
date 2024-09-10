/**@file   bsp_motor.c
* @brief   电机驱动相关外设配置,包括IO和PWM
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
* @brief  电机1脉冲频率控制
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
    __HAL_TIM_SET_COUNTER(&motor1_pwm_handle, 0);  //计数器重新计数
    __HAL_TIM_SET_COMPARE(&motor1_pwm_handle, TIM_CHANNEL_4, cmp);  //占空比50%
}
/**
* @brief  电机2脉冲频率控制
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
    __HAL_TIM_SET_COUNTER(&motor2_pwm_handle, 0);  //计数器重新计数
    __HAL_TIM_SET_COMPARE(&motor2_pwm_handle, TIM_CHANNEL_4, cmp);  //占空比50%
}
/**
* @brief  电机3脉冲频率控制
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
    __HAL_TIM_SET_COUNTER(&motor3_pwm_handle, 0);  //计数器重新计数
    __HAL_TIM_SET_COMPARE(&motor3_pwm_handle, TIM_CHANNEL_2, cmp);  //占空比50%
}
/**
* @brief  电机4脉冲频率控制
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
    __HAL_TIM_SET_COUNTER(&motor4_pwm_handle, 0);  //计数器重新计数
    __HAL_TIM_SET_COMPARE(&motor4_pwm_handle, TIM_CHANNEL_2, cmp);  //占空比50%
}
/**
* @brief  电机5脉冲频率控制
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
    __HAL_TIM_SET_COUNTER(&motor5_pwm_handle, 0);  //计数器重新计数
    __HAL_TIM_SET_COMPARE(&motor5_pwm_handle, TIM_CHANNEL_3, cmp);  //占空比50%
}
/**
* @brief  电机1初始化
* @attention 
*/
void motor1_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_OC_InitTypeDef tim_oc_struct = {0};
    /* 时钟使能 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    /* IO配置 */
    /* M1 GPIO Configuration
    M1_STEP  ======> PA11->tim1-ch4
    M1_DIR   ======> PA12
    M1_EN    ======> PD3
    M1_SLEEP ======> PD4
    M1_FAULT ======> PD6
    */
    /* out */
    gpio_init_struct.Pin   = GPIO_PIN_12; //引脚号
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;   //模式
    gpio_init_struct.Pull  = GPIO_PULLUP;           //上下拉电阻
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;  //IO口驱动电路响应速度
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);        //初始化
    gpio_init_struct.Pin   = GPIO_PIN_3 | GPIO_PIN_4; //引脚号
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);        //初始化
    /* in */
    gpio_init_struct.Pin   = GPIO_PIN_6;            //引脚号
    gpio_init_struct.Mode  = MODE_INPUT;            //模式
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);        //初始化
    /* pwm */
    gpio_init_struct.Pin   = GPIO_PIN_11;           //引脚号
    gpio_init_struct.Mode  = GPIO_MODE_AF_PP;       //模式
    gpio_init_struct.Alternate = GPIO_AF1_TIM1;     //复用
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);        //初始化
    /* PWM配置 */
    motor1_pwm_handle.Instance               = TIM1;                   //寄存器
    motor1_pwm_handle.Init.Prescaler         = TIM_STEP_PSC_INIT;      //预分频系数
    motor1_pwm_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;     //递增计数模式
    motor1_pwm_handle.Init.Period            = TIM_STEP_ARR_INIT;      //自动重装载值
    motor1_pwm_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; //时钟划分
    motor1_pwm_handle.Init.RepetitionCounter = 0;                      //重复计数器
    motor1_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //重装载更新时机,使能后下个周期更新arr,否则立即更新
    HAL_TIM_PWM_Init(&motor1_pwm_handle);                              //初始化
    /* PWM通道配置 */
    tim_oc_struct.OCMode       = TIM_OCMODE_PWM1;       //定时器模式
    tim_oc_struct.Pulse        = 0;                     //比较值,此值用来确定占空比
    tim_oc_struct.OCPolarity   = TIM_OCPOLARITY_HIGH;   //输出极性
    tim_oc_struct.OCNPolarity  = TIM_OCNPOLARITY_HIGH;  //互补输出极性
    tim_oc_struct.OCFastMode   = TIM_OCFAST_DISABLE;    //快速模式状态
    tim_oc_struct.OCIdleState  = TIM_OCIDLESTATE_RESET; //空闲状态时的TIM输出比较引脚状态
    tim_oc_struct.OCNIdleState = TIM_OCNIDLESTATE_RESET;//空闲状态时的TIM输出比较引脚状态
    HAL_TIM_PWM_ConfigChannel(&motor1_pwm_handle, &tim_oc_struct, TIM_CHANNEL_4); //配置TIMx通道y
    /* 启动 */
    HAL_TIM_PWM_Start(&motor1_pwm_handle, TIM_CHANNEL_4);  //开启对应PWM通道
    M1_DIR(0);
    M1_EN(0);
    M1_SLEEP(1);
}
/**
* @brief  电机2初始化
* @attention 
*/
void motor2_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_OC_InitTypeDef tim_oc_struct = {0};
    /* 时钟使能 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* IO配置 */
    /* M2 GPIO Configuration
    M2_STEP  ======> PB1->tim3-ch4
    M2_DIR   ======> PE9
    M2_EN    ======> PE10
    M2_SLEEP ======> PC5
    M2_FAULT ======> PC4
    */
    /* out */
    gpio_init_struct.Pin   = GPIO_PIN_5; //引脚号
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;   //模式
    gpio_init_struct.Pull  = GPIO_PULLUP;           //上下拉电阻
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;  //IO口驱动电路响应速度
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);        //初始化
    gpio_init_struct.Pin   = GPIO_PIN_9 | GPIO_PIN_10; //引脚号
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);        //初始化
    /* in */
    gpio_init_struct.Pin   = GPIO_PIN_4;            //引脚号
    gpio_init_struct.Mode  = MODE_INPUT;            //模式
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);        //初始化
    /* pwm */
    gpio_init_struct.Pin   = GPIO_PIN_1;            //引脚号
    gpio_init_struct.Mode  = GPIO_MODE_AF_PP;       //模式
    gpio_init_struct.Alternate = GPIO_AF2_TIM3;     //复用
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);        //初始化
    /* PWM配置 */
    motor2_pwm_handle.Instance               = TIM3;                   //寄存器
    motor2_pwm_handle.Init.Prescaler         = TIM_STEP_PSC_INIT;      //预分频系数
    motor2_pwm_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;     //递增计数模式
    motor2_pwm_handle.Init.Period            = TIM_STEP_ARR_INIT;      //自动重装载值
    motor2_pwm_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; //时钟划分
    motor2_pwm_handle.Init.RepetitionCounter = 0;                      //重复计数器
    motor2_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //重装载更新时机,使能后下个周期更新arr,否则立即更新
    HAL_TIM_PWM_Init(&motor2_pwm_handle);                              //初始化
    /* PWM通道配置 */
    tim_oc_struct.OCMode       = TIM_OCMODE_PWM1;       //定时器模式
    tim_oc_struct.Pulse        = 0;                     //比较值,此值用来确定占空比
    tim_oc_struct.OCPolarity   = TIM_OCPOLARITY_HIGH;   //输出极性
    tim_oc_struct.OCNPolarity  = TIM_OCNPOLARITY_HIGH;  //互补输出极性
    tim_oc_struct.OCFastMode   = TIM_OCFAST_DISABLE;    //快速模式状态
    tim_oc_struct.OCIdleState  = TIM_OCIDLESTATE_RESET; //空闲状态时的TIM输出比较引脚状态
    tim_oc_struct.OCNIdleState = TIM_OCNIDLESTATE_RESET;//空闲状态时的TIM输出比较引脚状态
    HAL_TIM_PWM_ConfigChannel(&motor2_pwm_handle, &tim_oc_struct, TIM_CHANNEL_4); //配置TIMx通道y
    /* 启动 */
    HAL_TIM_PWM_Start(&motor2_pwm_handle, TIM_CHANNEL_4);  //开启对应PWM通道
    M2_DIR(0);
    M2_EN(0);
    M2_SLEEP(1);
}
/**
* @brief  电机3初始化
* @attention 
*/
void motor3_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_OC_InitTypeDef tim_oc_struct = {0};
    /* 时钟使能 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    /* IO配置 */
    /* M3 GPIO Configuration
    M3_STEP  ======> PB7->tim4-ch2
    M3_DIR   ======> PE4
    M3_EN    ======> PE5
    M3_SLEEP ======> PE6
    M3_FAULT ======> PE2
    */
    /* out */
    gpio_init_struct.Pin   = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6; //引脚号
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;   //模式
    gpio_init_struct.Pull  = GPIO_PULLUP;           //上下拉电阻
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;  //IO口驱动电路响应速度
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);        //初始化
    /* in */
    gpio_init_struct.Pin   = GPIO_PIN_2;            //引脚号
    gpio_init_struct.Mode  = MODE_INPUT;            //模式
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);        //初始化
    /* pwm */
    gpio_init_struct.Pin   = GPIO_PIN_7;            //引脚号
    gpio_init_struct.Mode  = GPIO_MODE_AF_PP;       //模式
    gpio_init_struct.Alternate = GPIO_AF2_TIM4;     //复用
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);        //初始化
    /* PWM配置 */
    motor3_pwm_handle.Instance               = TIM4;                   //寄存器
    motor3_pwm_handle.Init.Prescaler         = TIM_STEP_PSC_INIT;      //预分频系数
    motor3_pwm_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;     //递增计数模式
    motor3_pwm_handle.Init.Period            = TIM_STEP_ARR_INIT;      //自动重装载值
    motor3_pwm_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; //时钟划分
    motor3_pwm_handle.Init.RepetitionCounter = 0;                      //重复计数器
    motor3_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //重装载更新时机,使能后下个周期更新arr,否则立即更新
    HAL_TIM_PWM_Init(&motor3_pwm_handle);                              //初始化
    /* PWM通道配置 */
    tim_oc_struct.OCMode       = TIM_OCMODE_PWM1;       //定时器模式
    tim_oc_struct.Pulse        = 0;                     //比较值,此值用来确定占空比
    tim_oc_struct.OCPolarity   = TIM_OCPOLARITY_HIGH;   //输出极性
    tim_oc_struct.OCNPolarity  = TIM_OCNPOLARITY_HIGH;  //互补输出极性
    tim_oc_struct.OCFastMode   = TIM_OCFAST_DISABLE;    //快速模式状态
    tim_oc_struct.OCIdleState  = TIM_OCIDLESTATE_RESET; //空闲状态时的TIM输出比较引脚状态
    tim_oc_struct.OCNIdleState = TIM_OCNIDLESTATE_RESET;//空闲状态时的TIM输出比较引脚状态
    HAL_TIM_PWM_ConfigChannel(&motor3_pwm_handle, &tim_oc_struct, TIM_CHANNEL_2); //配置TIMx通道y
    /* 启动 */
    HAL_TIM_PWM_Start(&motor3_pwm_handle, TIM_CHANNEL_2);  //开启对应PWM通道
    M3_DIR(0);
    M3_EN(0);
    M3_SLEEP(1);
}
/**
* @brief  电机4初始化
* @attention 
*/
void motor4_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_OC_InitTypeDef tim_oc_struct = {0};
    /* 时钟使能 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* IO配置 */
    /* M4 GPIO Configuration
    M4_STEP  ======> PA1->tim2-ch2
    M4_DIR   ======> PA3
    M4_EN    ======> PA0
    M4_SLEEP ======> PC3
    M4_FAULT ======> PA2
    */
    /* out */
    gpio_init_struct.Pin   = GPIO_PIN_0 | GPIO_PIN_3; //引脚号
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;   //模式
    gpio_init_struct.Pull  = GPIO_PULLUP;           //上下拉电阻
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;  //IO口驱动电路响应速度
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);        //初始化
    gpio_init_struct.Pin   = GPIO_PIN_3;            //引脚号
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);        //初始化
    /* in */
    gpio_init_struct.Pin   = GPIO_PIN_2;            //引脚号
    gpio_init_struct.Mode  = MODE_INPUT;            //模式
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);        //初始化
    /* pwm */
    gpio_init_struct.Pin   = GPIO_PIN_1;            //引脚号
    gpio_init_struct.Mode  = GPIO_MODE_AF_PP;       //模式
    gpio_init_struct.Alternate = GPIO_AF1_TIM2;     //复用
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);        //初始化
    /* PWM配置 */
    motor4_pwm_handle.Instance               = TIM2;                   //寄存器
    motor4_pwm_handle.Init.Prescaler         = TIM_STEP_PSC_INIT;      //预分频系数
    motor4_pwm_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;     //递增计数模式
    motor4_pwm_handle.Init.Period            = TIM_STEP_ARR_INIT;      //自动重装载值
    motor4_pwm_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; //时钟划分
    motor4_pwm_handle.Init.RepetitionCounter = 0;                      //重复计数器
    motor4_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //重装载更新时机,使能后下个周期更新arr,否则立即更新
    HAL_TIM_PWM_Init(&motor4_pwm_handle);                              //初始化
    /* PWM通道配置 */
    tim_oc_struct.OCMode       = TIM_OCMODE_PWM1;       //定时器模式
    tim_oc_struct.Pulse        = 0;                     //比较值,此值用来确定占空比
    tim_oc_struct.OCPolarity   = TIM_OCPOLARITY_HIGH;   //输出极性
    tim_oc_struct.OCNPolarity  = TIM_OCNPOLARITY_HIGH;  //互补输出极性
    tim_oc_struct.OCFastMode   = TIM_OCFAST_DISABLE;    //快速模式状态
    tim_oc_struct.OCIdleState  = TIM_OCIDLESTATE_RESET; //空闲状态时的TIM输出比较引脚状态
    tim_oc_struct.OCNIdleState = TIM_OCNIDLESTATE_RESET;//空闲状态时的TIM输出比较引脚状态
    HAL_TIM_PWM_ConfigChannel(&motor4_pwm_handle, &tim_oc_struct, TIM_CHANNEL_2); //配置TIMx通道y
    /* 启动 */
    HAL_TIM_PWM_Start(&motor4_pwm_handle, TIM_CHANNEL_2);  //开启对应PWM通道
    M4_DIR(0);
    M4_EN(0);
    M4_SLEEP(1);
}
/**
* @brief  电机5初始化
* @attention 
*/
void motor5_init(void)
{
    /* 结构体初始化 */
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_OC_InitTypeDef tim_oc_struct = {0};
    /* 时钟使能 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    /* IO配置 */
    /* M5 GPIO Configuration
    M5_STEP  ======> PC8->tim8-ch3
    M5_DIR   ======> PD10
    M5_EN    ======> PD15
    M5_SLEEP ======> PD13
    M5_FAULT ======> PD14
    */
    /* out */
    gpio_init_struct.Pin   = GPIO_PIN_10 | GPIO_PIN_13 | GPIO_PIN_15; //引脚号
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;   //模式
    gpio_init_struct.Pull  = GPIO_PULLUP;           //上下拉电阻
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;  //IO口驱动电路响应速度
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);        //初始化
    /* in */
    gpio_init_struct.Pin   = GPIO_PIN_14;           //引脚号
    gpio_init_struct.Mode  = MODE_INPUT;            //模式
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);        //初始化
    /* pwm */
    gpio_init_struct.Pin   = GPIO_PIN_8;            //引脚号
    gpio_init_struct.Mode  = GPIO_MODE_AF_PP;       //模式
    gpio_init_struct.Alternate = GPIO_AF3_TIM8;     //复用
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);        //初始化
    /* PWM配置 */
    motor5_pwm_handle.Instance               = TIM8;                   //寄存器
    motor5_pwm_handle.Init.Prescaler         = TIM_STEP_PSC_INIT;      //预分频系数
    motor5_pwm_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;     //递增计数模式
    motor5_pwm_handle.Init.Period            = TIM_STEP_ARR_INIT;      //自动重装载值
    motor5_pwm_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; //时钟划分
    motor5_pwm_handle.Init.RepetitionCounter = 0;                      //重复计数器
    motor5_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //重装载更新时机,使能后下个周期更新arr,否则立即更新
    HAL_TIM_PWM_Init(&motor5_pwm_handle);                              //初始化
    /* PWM通道配置 */
    tim_oc_struct.OCMode       = TIM_OCMODE_PWM1;       //定时器模式
    tim_oc_struct.Pulse        = 0;                     //比较值,此值用来确定占空比
    tim_oc_struct.OCPolarity   = TIM_OCPOLARITY_HIGH;   //输出极性
    tim_oc_struct.OCNPolarity  = TIM_OCNPOLARITY_HIGH;  //互补输出极性
    tim_oc_struct.OCFastMode   = TIM_OCFAST_DISABLE;    //快速模式状态
    tim_oc_struct.OCIdleState  = TIM_OCIDLESTATE_RESET; //空闲状态时的TIM输出比较引脚状态
    tim_oc_struct.OCNIdleState = TIM_OCNIDLESTATE_RESET;//空闲状态时的TIM输出比较引脚状态
    HAL_TIM_PWM_ConfigChannel(&motor5_pwm_handle, &tim_oc_struct, TIM_CHANNEL_3); //配置TIMx通道y
    /* 启动 */
    HAL_TIM_PWM_Start(&motor5_pwm_handle, TIM_CHANNEL_3);  //开启对应PWM通道
    M5_DIR(0);
    M5_EN(0);
    M5_SLEEP(1);
}

