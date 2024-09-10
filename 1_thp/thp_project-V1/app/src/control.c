/**@file   control.c
* @brief   控制
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "control.h"
#include "stdlib.h"
#include "math.h"
#include "device.h"
#include "motor.h"
#include "interp_table.h"
#include "comm.h"
#include "crc_check.h"
#include "memory.h"

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

RobotParamf robot_para = 
{
    .lh   = 24.7,
    .ll   = 150.0,
    .ls   = 35.0,
    .lg1  = 28.2,
    .lg2  = 20.3,
    .e    = 14.21156685,
    .hg   = 68.0,
    .ang  = 140.0,
    .grm  = 90.0,
    .min  = 0.0,
    .max  = 90.0,
//    .dtm_model[4]
    .dtm[0] = 166.62,
    .dtm[1] = 166.62,
    .dtm[2] = 166.62,
    .dtm[3] = 166.62,
};

adc_channel_data adc_data;
struct motor motor1;
struct motor motor2;
struct motor motor3;
struct motor motor4;
struct communication commu;
__IO uint8_t cmd_com[30];
uint32_t motorvalue;
int stepcounter1;
int stepcounter2;
int stepcounter3;
int stepcounter4;

double voltage_mm;
float M1_error;
float M2_error;
float M3_error;
float M4_error;

uint32_t motorcal[600];

uint8_t Paremeter[200];

int32_t LED_Counter;

void ClearProtect()
{
  uint8_t cmd[4];
  cmd[0]=0x01;
  cmd[1]=0x0E;
  cmd[2]=0x52;
    cmd[3] = GetCRC8Reverse(cmd, 3, 0x00);
  motor1.curaction=removeprotect;
  motor2.curaction=removeprotect;
  motor3.curaction=removeprotect;
  motor4.curaction=removeprotect;
  Usart1Send(cmd,4);
  cmd[0]=0x02;
    cmd[3] = GetCRC8Reverse(cmd, 3, 0x00);
  Usart2Send(cmd,4);
  cmd[0]=0x03;
    cmd[3] = GetCRC8Reverse(cmd, 3, 0x00);
  Usart3Send(cmd,4);
  cmd[0]=0x04;
    cmd[3] = GetCRC8Reverse(cmd, 3, 0x00);
  Uart4Send(cmd,4);
  
}
void StopMotor1()
{
    MotorSpeedCtrl(0, 1);
    motor1.flag_move = 0;
}

void StopMotor2()
{
    MotorSpeedCtrl(0, 2);
    motor2.flag_move = 0;
}

void StopMotor3()
{
    MotorSpeedCtrl(0, 3);
    motor3.flag_move = 0;
}

void StopMotor4()
{
    MotorSpeedCtrl(0, 4);
    motor4.flag_move = 0;
}
void StopMotor()
{
    StopMotor1();
    StopMotor2();
    StopMotor3();
    StopMotor4();
}

void EnableMotor(uint8_t enable)
{
  uint8_t cmd[4];
  cmd[0]=0x01;
  cmd[1]=0xF3;
  cmd[2]=enable;
  cmd[3] = GetCRC8Reverse(cmd, 3, 0x00);
  Usart1Send(cmd,4);
  cmd[0]=0x02;
    cmd[3] = GetCRC8Reverse(cmd, 3, 0x00);
  Usart2Send(cmd,4);
  cmd[0]=0x03;
    cmd[3] = GetCRC8Reverse(cmd, 3, 0x00);
  Usart3Send(cmd,4);
  cmd[0]=0x04;
    cmd[3] = GetCRC8Reverse(cmd, 3, 0x00);
  Uart4Send(cmd,4);
}

void SetZeroPoint()
{
  uint8_t cmd[4];
  cmd[0]=0x01;
  cmd[1]=0x0A;
  cmd[2]=0x6D;
  cmd[3] = GetCRC8Reverse(cmd, 3, 0x00);
  Usart1Send(cmd,4);
  
  cmd[0]=0x02;          //ID
    cmd[3] = GetCRC8Reverse(cmd, 3, 0x00);
  Usart2Send(cmd,4);
  cmd[0]=0x03;          //ID
    cmd[3] = GetCRC8Reverse(cmd, 3, 0x00);
  Usart3Send(cmd,4);
  cmd[0]=0x04;  
    cmd[3] = GetCRC8Reverse(cmd, 3, 0x00);
  Uart4Send(cmd,4);
}

void ReadProtect()
{
  uint8_t cmd[3];
  cmd[0]=0x01;
  cmd[1]=0x3E;
  cmd[2] = GetCRC8Reverse(cmd, 2, 0x00);
  motor1.curaction=readprotect;
  Usart1Send(cmd,3);
  
  cmd[0]=0x02;          //ID
    cmd[2] = GetCRC8Reverse(cmd, 2, 0x00);
  motor2.curaction=readprotect;
  Usart2Send(cmd,3);
  cmd[0]=0x03;          //ID
    cmd[2] = GetCRC8Reverse(cmd, 2, 0x00);
  motor3.curaction=readprotect;
  Usart3Send(cmd,3);
  cmd[0]=0x04;  
    cmd[2] = GetCRC8Reverse(cmd, 2, 0x00);
  motor4.curaction=readprotect;
  Uart4Send(cmd,3);
  
}

void MoveMotorPos()
{
  __IO double movelen;
  uint32_t pos;
  uint8_t cmd[9];
  
  movelen=motor1.curpos-motor1.tarpos;
  cmd[0]=0x01;
  cmd[1]=0xFD;
  if(movelen>0)
    cmd[2]=0x00;        //方向
  else
  {
    cmd[2]=0x10;   
    movelen=-movelen;
  }
  pos=(uint32_t)(movelen*25600.0);              //12800.0
  cmd[3]=0x60;          //速度
  cmd[4]=0x00;          //加速度
  cmd[5]=(uint8_t)(pos>>16);
  cmd[6]=(uint8_t)(pos>>8);
  cmd[7]=(uint8_t)(pos);
  cmd[8] = GetCRC8Reverse(cmd, 8, 0x00);
  motor1.curaction=move;
  Usart1Send(cmd,9);
  
  movelen=motor2.curpos-motor2.tarpos;
  
  cmd[0]=0x02;          //ID
  cmd[1]=0xFD;
  if(movelen>0)
    cmd[2]=0x00;        //方向
  else
  {
    cmd[2]=0x10;   
    movelen=-movelen;
  }
  pos=(uint32_t)(movelen*25600.0);
  cmd[3]=0x60;          //速度
  cmd[4]=0x00;          //加速度
  cmd[5]=(uint8_t)(pos>>16);
  cmd[6]=(uint8_t)(pos>>8);
  cmd[7]=(uint8_t)(pos);
  cmd[8] = GetCRC8Reverse(cmd, 8, 0x00);
  motor2.curaction=move;
  Usart2Send(cmd,9);
  
  movelen=motor3.curpos-motor3.tarpos;
  
  cmd[0]=0x03;          //ID
  cmd[1]=0xFD;
  if(movelen>0)
    cmd[2]=0x00;        //方向
  else
  {
    cmd[2]=0x10;   
    movelen=-movelen;
  }
  pos=(uint32_t)(movelen*25600.0);
  cmd[3]=0x60;          //速度
  cmd[4]=0x00;          //加速度
  cmd[5]=(uint8_t)(pos>>16);
  cmd[6]=(uint8_t)(pos>>8);
  cmd[7]=(uint8_t)(pos);
  cmd[8] = GetCRC8Reverse(cmd, 8, 0x00);
  motor3.curaction=move;
  
  Usart3Send(cmd,9);
  
  movelen=motor4.curpos-motor4.tarpos;
  
  cmd[0]=0x04;          //ID
  cmd[1]=0xFD;
  if(movelen>0)
    cmd[2]=0x00;        //方向
  else
  {
    cmd[2]=0x10;   
    movelen=-movelen;
  }
  pos=(uint32_t)(movelen*25600.0);
  cmd[3]=0x60;          //速度
  cmd[4]=0x00;          //加速度
  cmd[5]=(uint8_t)(pos>>16);
  cmd[6]=(uint8_t)(pos>>8);
  cmd[7]=(uint8_t)(pos);
  cmd[8] = GetCRC8Reverse(cmd, 8, 0x00);
  motor4.curaction=move;
  Uart4Send(cmd,9);
}

void calibration()
{
    uint16_t i = 0;

    commu.nextaction=cmd_cal;
    CtrlLed(CTRL_PURPLE);
    OutData.init_pos1 = 0;
    OutData.init_pos2 = 0;
    OutData.init_pos3 = 0;
    OutData.init_pos4 = 0;
    
    releasebrake();
    EnableMotor(1);
    DelayMs(1000);
//    CommCtrlMotorPos(100, 30, 1);
//    CommCtrlMotorPos(100, 30, 2);
//    CommCtrlMotorPos(100, 30, 3);
//    CommCtrlMotorPos(100, 30, 4);
    MotorSpeedCtrl(30, 1);
    MotorSpeedCtrl(30, 2);
    MotorSpeedCtrl(30, 3);
    MotorSpeedCtrl(30, 4);

    DelayMs(10000);
    StopMotor();
    DelayMs(1000);
    ClearProtect();
    DelayMs(10);
    SetZeroPoint();
    DelayMs(10);
    
//    CommCtrlMotorPos(-100, 30, 1);
//    CommCtrlMotorPos(-100, 30, 2);
//    CommCtrlMotorPos(-100, 30, 3);
//    CommCtrlMotorPos(-100, 30, 4);
    MotorSpeedCtrl(-30, 1);
    MotorSpeedCtrl(-30, 2);
    MotorSpeedCtrl(-30, 3);
    MotorSpeedCtrl(-30, 4);

    DelayMs(10000);
    CommGetMotorPos(1);
    CommGetMotorPos(2);
    CommGetMotorPos(3);
    CommGetMotorPos(4);
    DelayMs(10);
    if(OutData.e_pos1 < -65)
    {
        MotorType = SYS_MOTOR100;
    }
    else
    {
        MotorType = SYS_MOTOR60;
    }
    STMFLASH_Write(FLASH_MID_ADDR,(uint32_t*)&MotorType,1);
    
    StopMotor();
    DelayMs(1000);
    ClearProtect();
    DelayMs(10);
    SetZeroPoint();
    DelayMs(10);
    
    CommCtrlMotorPos(0.3, 30, 1, 1);
    CommCtrlMotorPos(0.3, 30, 2, 1);
    CommCtrlMotorPos(0.3, 30, 3, 1);
    CommCtrlMotorPos(0.3, 30, 4, 1);
    DelayMs(3000);
    
    while((InFilterData.pos1 > 0x7F1000) || (InFilterData.pos2 > 0x7F1000)
        || (InFilterData.pos3 > 0x7F1000) || (InFilterData.pos4 > 0x7F1000))
//    while(InFilterData.Pos1 > 0x7F1000)
    {
        if(InFilterData.pos1 > 0x7F1000)
        {
            CommCtrlMotorPos(0.1, 20, 1, 1);
        }
        if(InFilterData.pos2 > 0x7F1000)
        {
            CommCtrlMotorPos(0.1, 20, 2, 1);
        }
        if(InFilterData.pos3 > 0x7F1000)
        {
            CommCtrlMotorPos(0.1, 20, 3, 1);
        }
        if(InFilterData.pos4 > 0x7F1000)
        {
            CommCtrlMotorPos(0.1, 20, 4, 1);
        }
        DelayMs(800);
    }
    for(i=0; i<30;i++)
    {
        DeviceList.x_pos1[i]=InFilterData.pos1;
        DeviceList.x_pos2[i]=InFilterData.pos2;
        DeviceList.x_pos3[i]=InFilterData.pos3;
        DeviceList.x_pos4[i]=InFilterData.pos4;
        CommCtrlMotorPos(0.1, 20, 1, 1);
        CommCtrlMotorPos(0.1, 20, 2, 1);
        CommCtrlMotorPos(0.1, 20, 3, 1);
        CommCtrlMotorPos(0.1, 20, 4, 1);
        DelayMs(750);
        CommGetMotorPos(1);
        CommGetMotorPos(2);
        CommGetMotorPos(3);
        CommGetMotorPos(4);
        DelayMs(750);
    }
    for(;i<TableMax[MotorType]-31;i++)
    {
        DeviceList.x_pos1[i]=InFilterData.pos1;
        DeviceList.x_pos2[i]=InFilterData.pos2;
        DeviceList.x_pos3[i]=InFilterData.pos3;
        DeviceList.x_pos4[i]=InFilterData.pos4;
        CommCtrlMotorPos(1, 20, 1, 1);
        CommCtrlMotorPos(1, 20, 2, 1);
        CommCtrlMotorPos(1, 20, 3, 1);
        CommCtrlMotorPos(1, 20, 4, 1);
        DelayMs(1500);
        CommGetMotorPos(1);
        CommGetMotorPos(2);
        CommGetMotorPos(3);
        CommGetMotorPos(4);
        DelayMs(1500);
    }
    for(;i<TableMax[MotorType];i++)
    {
        DeviceList.x_pos1[i]=InFilterData.pos1;
        DeviceList.x_pos2[i]=InFilterData.pos2;
        DeviceList.x_pos3[i]=InFilterData.pos3;
        DeviceList.x_pos4[i]=InFilterData.pos4;
        CommCtrlMotorPos(0.1, 20, 1, 1);
        CommCtrlMotorPos(0.1, 20, 2, 1);
        CommCtrlMotorPos(0.1, 20, 3, 1);
        CommCtrlMotorPos(0.1, 20, 4, 1);
        DelayMs(750);
        CommGetMotorPos(1);
        CommGetMotorPos(2);
        CommGetMotorPos(3);
        CommGetMotorPos(4);
        DelayMs(750);
    }
    for(i=0;i<150;i++)
    {
        motorcal[i]     = (uint32_t)DeviceList.x_pos1[i];
        motorcal[150+i] = (uint32_t)DeviceList.x_pos2[i];
        motorcal[300+i] = (uint32_t)DeviceList.x_pos3[i];
        motorcal[450+i] = (uint32_t)DeviceList.x_pos4[i];
    }
    activebrake();
    CtrlSysInit();
    SetZeroPoint();
    DelayMs(10);
    STMFLASH_Write(FLASH_ADDR,(uint32_t*)motorcal,600);
    CtrlLed(CTRL_GREEN);
    commu.nextaction=cmd_reset;
}
uint8_t pos_check(float M1_pos,float M2_pos,float M3_pos,float M4_pos)
{
  uint8_t error=0;
  if((M1_pos<M1_min) ||M1_pos>M1_max)
  {
    motor1.flag_poserror=1;
    error=1;
  }
  if((M2_pos<M2_min) ||M2_pos>M2_max)
  {
    motor2.flag_poserror=1;
    error=1;
  }
  
  if((M3_pos<M3_min) ||M3_pos>M3_max)
  {
    motor3.flag_poserror=1;
    error=1;
  }
  
  if((M4_pos<M4_min) ||M4_pos>M4_max)
  {
    motor4.flag_poserror=1;
    error=1;
  }
  return error;
}
void activebrake()
{
  GPIO_WriteBit(OUT1_GPIO_Port,OUT1_Pin,Bit_SET);
}
void releasebrake()
{
  GPIO_WriteBit(OUT1_GPIO_Port,OUT1_Pin,Bit_RESET);
}

float AlarmPos1 = 0;
float AlarmPos2 = 0;
float AlarmPos3 = 0;
float AlarmPos4 = 0;
void AlarmPos(void)
{
//    static uint8_t cnt = 0;
//    
//    AlarmPos1 = OutData.m_pos1 - OutData.pos1;
//    AlarmPos2 = OutData.m_pos2 - OutData.pos2;
//    AlarmPos3 = OutData.m_pos3 - OutData.pos3;
//    AlarmPos4 = OutData.m_pos4 - OutData.pos4;
//    if((fabs(AlarmPos1) > 10) || (fabs(AlarmPos2) > 10) || (fabs(AlarmPos3) > 10) || (fabs(AlarmPos4) > 10))
//    {
//        if(cnt > 10)
//        {
//            cnt = 0;
//            POWER_OFF;
//            CtrlLed(CTRL_RED);
//            commu.nextaction = cmd_fault;
//        }
//        else
//        {
//            cnt++;
//        }
//    }
//    else
//    {
//        cnt = 0;
//    }
}

void ControlRun(void)
{
    static uint16_t cnt = 0;
    
  if(GPIO_ReadInputDataBit(IN1_GPIO_Port,IN1_Pin)==0)
  {
    calibration();
  }
  else
  {
//    MemRead();
  }

  commu.nextaction=cmd_reset;
  //green
  CtrlLed(CTRL_GREEN);
  EnableMotor(1);
  
 // MX_IWDG_Init();
  activebrake();

  while (1)
  {
    switch (commu.nextaction)
    {
    case cmd_removeprotect:
      {
        ClearProtect();
        motor1.flag_protect=0;
        motor2.flag_protect=0;
        motor3.flag_protect=0;
        motor4.flag_protect=0;
          CtrlSysInit();
        commu.nextaction=cmd_checkmove;
        break;
      }
    case cmd_stopmove:
      {
        if(motor1.flag_protect||motor2.flag_protect||motor3.flag_protect||motor4.flag_protect)
        {
          //yellow
          CtrlLed(CTRL_YELLOW);
        }
        else
        {
          //green
          CtrlLed(CTRL_GREEN);
        }
        motor1.flag_move=0;
        motor2.flag_move=0;
        motor3.flag_move=0;
        motor4.flag_move=0;
        StopMotor();
        cnt++;
        if(cnt == 12)
        {
            StopMotor();
        }
        if(cnt > 15)
        {
            cnt = 0;
            
            activebrake();
            commu.nextaction=cmd_checkmove;
        }
        break;
      }
    case cmd_move:
      {
        if(motor1.flag_protect||motor2.flag_protect||motor3.flag_protect||motor4.flag_protect)
        {
            //yellow
            CtrlLed(CTRL_YELLOW);
            cnt++;
            if(cnt > 140)
            {
                StopMotor();
            }
            if(cnt > 150)
            {
                cnt = 0;
                commu.nextaction=cmd_reset;
            }
            break;
        }
        else
        {
          //blue
          CtrlLed(CTRL_BLUE);
        }
        cnt++;
        if(cnt > 2)
        {
            cnt = 0;
            if(pos_check(commu.len_m1,commu.len_m2,commu.len_m3,commu.len_m4)==0)
            {
                releasebrake();
                motor1.flag_move=1;
                motor2.flag_move=1;
                motor3.flag_move=1;
                motor4.flag_move=1;
                motor1.tarpos=commu.len_m1;
                motor2.tarpos=commu.len_m2;
                motor3.tarpos=commu.len_m3;
                motor4.tarpos=commu.len_m4;
#ifdef POS_CONTROL
                CommCtrlMotorPos(motor1.tarpos - OutData.m_pos1, 15, 1, 1);
                CommCtrlMotorPos(motor2.tarpos - OutData.m_pos2, 15, 2, 1);
                CommCtrlMotorPos(motor3.tarpos - OutData.m_pos3, 15, 3, 1);
                CommCtrlMotorPos(motor4.tarpos - OutData.m_pos4, 15, 4, 1);
#endif
                
                commu.nextaction=cmd_stepmove;
            }
        }
        break;
        
      }
    case cmd_stepmove:
      {
        releasebrake();
//          motor1.flag_protect = 1;
        if(motor1.flag_protect||motor2.flag_protect||motor3.flag_protect||motor4.flag_protect)
        {
          //yellow
          CtrlLed(CTRL_YELLOW);
          
            cnt++;
            if(cnt > 140)
            {
                StopMotor();
            }
            if(cnt > 150)
            {
                cnt = 0;
                commu.nextaction=cmd_reset;
            }
            
          break;
        }
        else
        {
          //blue
          CtrlLed(CTRL_BLUE);
        }
        
#ifdef POS_CONTROL
        if(stepcounter1%25==0)
        {
            ReadProtect();
        }
        else
        {
            CommGetMotorPos(1);
            CommGetMotorPos(2);
            CommGetMotorPos(3);
            CommGetMotorPos(4);
        }
        M1_error=fabs(motor1.tarpos-motor1.curpos);
        M2_error=fabs(motor2.tarpos-motor2.curpos);  
        M3_error=fabs(motor3.tarpos-motor3.curpos);
        M4_error=fabs(motor4.tarpos-motor4.curpos);
        if(M1_error<0.02)
        {
            motor1.flag_move=0;
        }
        if(M2_error<0.02)
        {
            motor2.flag_move=0;
        }
        if(M3_error<0.02)
        {
            motor3.flag_move=0;
        }
        if(M4_error<0.02)
        {
            motor4.flag_move=0;
        }
#else
        if(stepcounter1 % 10 == 0)
        {
            if(Motor1PosCtrl(motor1.tarpos))
            {
                motor1.flag_move = 0;
            }
            else
            {
                motor1.flag_move = 1;
            }
            if(Motor2PosCtrl(motor2.tarpos))
            {
                motor2.flag_move = 0;
            }
            else
            {
                motor2.flag_move = 1;
            }
            if(Motor3PosCtrl(motor3.tarpos))
            {
                motor3.flag_move = 0;
            }
            else
            {
                motor3.flag_move = 1;
            }
            if(Motor4PosCtrl(motor4.tarpos))
            {
                motor4.flag_move = 0;
            }
            else
            {
                motor4.flag_move = 1;
            }
        }
        else
        {
            if(stepcounter1%25==0)
            {
                ReadProtect();
            }
            else
            {
                CommGetMotorPos(1);
                CommGetMotorPos(2);
                CommGetMotorPos(3);
                CommGetMotorPos(4);
            }
        }
#endif
        AlarmPos();
        
        if((motor1.flag_move==0)&&(motor2.flag_move==0)&&(motor3.flag_move==0)&&(motor4.flag_move==0))
        {
            motor1.stoppos=motor1.curpos;
            motor2.stoppos=motor2.curpos;
            motor3.stoppos=motor3.curpos;
            motor4.stoppos=motor4.curpos;
            stepcounter1 = 0;
            StopMotor();
            cnt++;
            if(cnt > 10)
            {
                cnt = 0;
                activebrake();
                commu.nextaction=cmd_checkmove;
            }
        }
        stepcounter1++;
        break;
      }
      
    case cmd_checkmove:
      {
        activebrake();
        if(motor1.flag_protect||motor2.flag_protect||motor3.flag_protect||motor4.flag_protect)
        {
          //yellow
          CtrlLed(CTRL_YELLOW);
        }
        else
        {
          //green
          CtrlLed(CTRL_GREEN);
        }
        cnt++;
        if(cnt == 1)
        {
            ReadProtect();
        }
        if(cnt > 2)
        {
            cnt = 0;
            CommGetMotorPos(1);
            CommGetMotorPos(2);
            CommGetMotorPos(3);
            CommGetMotorPos(4);
        }
        
        AlarmPos();
        commu.nextaction=cmd_checkmove;
        break;
      }
      
    case cmd_reset:
      {
            cnt++;
            if(cnt == 50)
            {
                ReadProtect();
            }
            if(cnt > 100)
            {
                cnt = 0;
                CommGetMotorPos(1);
                CommGetMotorPos(2);
                CommGetMotorPos(3);
                CommGetMotorPos(4);
            }
            break;
      }
    default:
      break;
      
    }

    DelayMs(2);
    
  }
}

/**
* @brief  灯光控制
*/
void CtrlLed(uint16_t color)
{
    switch(color)
    {
        case CTRL_RED   : OUT2_ON; OUT3_OFF; OUT4_OFF; break;
        case CTRL_GREEN : OUT2_OFF; OUT3_OFF; OUT4_ON; break;
        case CTRL_BLUE  : OUT2_OFF; OUT3_ON; OUT4_OFF; break;
        case CTRL_YELLOW: OUT2_ON; OUT3_OFF; OUT4_ON;  break;
        case CTRL_PURPLE: OUT2_ON; OUT3_ON; OUT4_OFF;  break;
        case CTRL_CYAN  : OUT2_OFF; OUT3_ON; OUT4_ON;  break;
        case CTRL_WHITE : OUT2_ON; OUT3_ON; OUT4_ON;   break;
        default: OUT2_OFF; OUT3_OFF; OUT4_OFF; break;
    }
}
/**
* @brief  控制系统初始化
*/
void CtrlSysInit(void)
{
    POWER_ON;
    DelayMs(1000);
    EnableMotor(1);
    DelayMs(5);
    EnableMotor(1);
    DelayMs(5);
    SetZeroPoint();
    DelayMs(5);
    SetZeroPoint();
    OutData.init_pos1 = OutFilterData.pos1;
    OutData.init_pos2 = OutFilterData.pos2;
    OutData.init_pos3 = OutFilterData.pos3;
    OutData.init_pos4 = OutFilterData.pos4;
    OutData.m_pos1 = OutFilterData.pos1;
    OutData.m_pos2 = OutFilterData.pos2;
    OutData.m_pos3 = OutFilterData.pos3;
    OutData.m_pos4 = OutFilterData.pos4;
    CtrlLed(CTRL_GREEN);
}

