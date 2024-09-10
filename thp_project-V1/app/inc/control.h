/**@file   control.h
* @brief   ¿ØÖÆ
* @author  ³Â×¿ÕÜ
* @date    2023/8/30
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef CONTROL_H_
#define CONTROL_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_cfg.h"
#include "sys_cfg.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**
* @enum 	CTRL_COLOR 
* @brief  	LEDÑÕÉ«¿ØÖÆ
*/
typedef enum
{
    CTRL_BLACK,   ///< ºÚ
    CTRL_RED,     ///< ºì
    CTRL_GREEN,   ///< ÂÌ
    CTRL_BLUE,    ///< À¶
    CTRL_YELLOW,  ///< »Æ
    CTRL_PURPLE,  ///< ×Ï
    CTRL_CYAN,    ///< Çà
    CTRL_WHITE,   ///< °×
}CTRL_COLOR;

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void CtrlLed(uint16_t color);
void ControlRun(void);




void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_Pin_13
#define LED_GPIO_Port GPIOC
#define SW1_Pin GPIO_Pin_0
#define SW1_GPIO_Port GPIOC
#define SW2_Pin GPIO_Pin_1
#define SW2_GPIO_Port GPIOC
#define SW3_Pin GPIO_Pin_2
#define SW3_GPIO_Port GPIOC
#define SW4_Pin GPIO_Pin_3
#define SW4_GPIO_Port GPIOC
#define DRDY_Pin GPIO_Pin_4
#define DRDY_GPIO_Port GPIOC
#define DRDY_EXTI_IRQn EXTI4_IRQn
#define CS_Pin GPIO_Pin_5
#define CS_GPIO_Port GPIOC
#define SYNC_Pin GPIO_Pin_0
#define SYNC_GPIO_Port GPIOB
#define power_on_Pin GPIO_Pin_1
#define power_on_GPIO_Port GPIOB
#define OUT1_Pin GPIO_Pin_12
#define OUT1_GPIO_Port GPIOB
#define OUT2_Pin GPIO_Pin_13
#define OUT2_GPIO_Port GPIOB
#define OUT3_Pin GPIO_Pin_14
#define OUT3_GPIO_Port GPIOB
#define OUT4_Pin GPIO_Pin_15
#define OUT4_GPIO_Port GPIOB
#define TX_EN_Pin GPIO_Pin_8
#define TX_EN_GPIO_Port GPIOC
#define SW6_Pin GPIO_Pin_9
#define SW6_GPIO_Port GPIOC
#define SW5_Pin GPIO_Pin_8
#define SW5_GPIO_Port GPIOA
#define IN1_Pin GPIO_Pin_8
#define IN1_GPIO_Port GPIOB
#define IN2_Pin GPIO_Pin_9
#define IN2_GPIO_Port GPIOB


#define RECEIVENUM      25
#define RECEIVENUM_COM      1000
#define SENDNUM         9
#define M_min           0

#define M1_min          0
#define M1_max          LengthMax[MotorType]
#define M2_min          0
#define M2_max          LengthMax[MotorType]
#define M3_min          0
#define M3_max          LengthMax[MotorType]
#define M4_min          0
#define M4_max          LengthMax[MotorType]

typedef struct s_RobotParamf
{
    float lh ;
    float ll ;
    float ls ;
    float lg1;
    float lg2;
    float e  ;

    float hg ;
    float ang;
    float grm;

    float min;
    float max;

    float dtm_model[4];

    float dtm[4];
} RobotParamf;

struct motor{
  uint8_t motor_id;
  enum action{reset,move, readpos,removeprotect,readprotect} curaction;
  __IO float curpos;
  __IO float tarpos;
  float stoppos;
  uint32_t timeout;
  uint32_t timeout_limit; 
  uint8_t flag_move;
  __IO uint8_t flag_protect;
  uint8_t flag_error;
  uint8_t flag_errorstop;
  uint8_t flag_poserror;
  uint8_t vel_dir;              //0 back 1 forward
  uint8_t vel_value;
  float offset;
  float gain;
  
};

struct communication{
  float len_m1;
  float len_m2;
  float len_m3;
  float len_m4;
  float tcp_x;
  float tcp_y;
  float tcp_z;
  float tcp_rx;
  float tcp_ry;
  float tcp_rz;
  uint8_t tcp_cmd;
  uint8_t clear_cmd;
  enum cmd_action{cmd_test,cmd_cal,cmd_stopmove,cmd_move,cmd_removeprotect,cmd_reset,cmd_stepmove,cmd_checkmove,cmd_fault} nextaction;
  __IO uint8_t communication_receivebuf[RECEIVENUM_COM];
};

extern adc_channel_data adc_data;
extern struct motor motor1;
extern struct motor motor2;
extern struct motor motor3;
extern struct motor motor4;
extern struct communication commu;
extern RobotParamf robot_para;
void releasebrake();
void activebrake();
void calibration();
void ClearProtect();

void CtrlSysInit(void);

#endif /* CONTROL_H_ */
