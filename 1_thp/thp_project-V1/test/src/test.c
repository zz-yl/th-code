/**@file   test.c
* @brief   测试
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "test.h"
#include "motor.h"
#include "control.h"
#include "comm.h"
#include "crc_check.h"
#include "comm_msg.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/


/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

TEST_CTRL TestCtrl = 
{
    .Data1 = 20,
    .ArrA = {0x01, 0xFD, 0x10, 0x60, 0x00, 0x00, 0x00, 0x00, 0x6B},
    .ArrLen = 9,
};

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/
uint32_t Reverse(uint32_t value)
{
  value = ((value & 0xAAAAAAAA) >> 1) | ((value & 0x55555555) << 1);
  value = ((value & 0xCCCCCCCC) >> 2) | ((value & 0x33333333) << 2);
  value = ((value & 0xF0F0F0F0) >> 4) | ((value & 0x0F0F0F0F) << 4);
//  value = ((value & 0xFF00FF00) >> 8) | ((value & 0x00FF00FF) << 8);
//  value = (value >> 16) | (value << 16);
  return value;
}
uint8_t crcTable[300] = {0};
void GenerateTable(uint32_t polynomial, bool reflectIn, bool reflectOut)
{
    for (int byte = 0; byte < 256; ++byte)
    {
        uint32_t crc = (reflectIn ? (Reverse((uint32_t)byte)) : byte);

        for (int bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ polynomial;
            }
            else
            {
                crc <<= 1;
            }
        }
        crcTable[byte] = (reflectOut ? Reverse(crc) : crc);
    }
}
//void GenerateTable(uint32_t polynomial)
//{
//	for (int byte = 0; byte < 256; ++byte)
//	 {
//		uint32_t crc = byte;
//		
//		for (int bit = 8; bit > 0; --bit)
//		{
//			if (crc & 0x80)
//			{
//				  crc = (crc << 1) ^ polynomial;
//			}
//			else
//			{
//				  crc <<= 1;
//			}
//		}
//		crcTable[byte] = crc;
//	 }
//}
void TestCrc(void)
{
    GenerateTable(0x31, 1, 1);
}

void MotorTest1(uint8_t dir)
{
    uint8_t arr[] = {0x02, 0xFD, 0x10, 0x60, 0x00, 0x23, 0x28, 0x00, 0x6B};
    
    if(dir)
        arr[2] = 0x10;
    else
        arr[2] = 0;
    Usart2Send(arr, 9);
    arr[0] = 0x03;
    Usart3Send(arr, 9);
}
void TestUartMotorSpeed(void)
{
    static uint16_t cnt = 0;
    
    if(cnt % TestCtrl.Data2 == 0)
    {
        TestCtrl.Ctrl = -TestCtrl.Ctrl;
    }
    if(cnt % 2 == 0)
    {
        MotorSpeedCtrl(TestCtrl.Ctrl, 1);
    }
    else
    {
        CommGetMotorPos(1);
    }
    
    cnt++;
}
void TestUartMotorPos(void)
{
    static uint16_t cnt = 0;
    
    if(cnt % TestCtrl.Data2 == 0)
    {
        TestCtrl.Ctrl = -TestCtrl.Ctrl;
        CommCtrlMotorPos(TestCtrl.Ctrl, TestCtrl.Data1, 1, TestCtrl.Data3);
    }
    else
    {
        CommGetMotorPos(1);
    }
    
    cnt++;
}
void TestUartMotorRun(uint8_t mode)
{
    static uint16_t cnt = 0;
    
    if(mode)
    {
        if(cnt % TestCtrl.Data2 == 0)
        {
            DelayMs(10);
            commu.nextaction=cmd_stopmove;
            DelayMs(50);
            commu.nextaction=cmd_removeprotect;
            DelayMs(500);
            commu.nextaction=cmd_test;
            DelayMs(50);
            TestCtrl.Ctrl = -TestCtrl.Ctrl;
            CommCtrlMotorPos(TestCtrl.Ctrl, TestCtrl.Data1, 1, TestCtrl.Data3);
            CommCtrlMotorPos(TestCtrl.Ctrl, TestCtrl.Data1, 2, TestCtrl.Data3);
            CommCtrlMotorPos(TestCtrl.Ctrl, TestCtrl.Data1, 3, TestCtrl.Data3);
            CommCtrlMotorPos(TestCtrl.Ctrl, TestCtrl.Data1, 4, TestCtrl.Data3);
        }
        else
        {
            CommGetMotorPos(1);
        }
    }
    else
    {
        if(cnt % TestCtrl.Data2 == 0)
        {
            DelayMs(10);
            commu.nextaction=cmd_stopmove;
            DelayMs(50);
            commu.nextaction=cmd_removeprotect;
            DelayMs(500);
            commu.nextaction=cmd_test;
            DelayMs(50);
            TestCtrl.Ctrl = -TestCtrl.Ctrl;
            MotorSpeedCtrl(TestCtrl.Ctrl, 1);
            MotorSpeedCtrl(TestCtrl.Ctrl, 2);
            MotorSpeedCtrl(TestCtrl.Ctrl, 3);
            MotorSpeedCtrl(TestCtrl.Ctrl, 4);
        }
        else
        {
            CommGetMotorPos(1);
        }
    }

    cnt++;
}
void TestXYZ(void)
{
    static uint8_t i = 0;
    
    float pos[5][5] = 
    {
        {25, 25, 25, 25},
        {25.68, 1.733, 25.68, 1.733},
        {89.332, 71.559, 89.332, 71.559},
        {71.559, 89.332, 71.559, 89.332},
        {1.733, 25.68, 1.733, 25.68}
    };
    
    if((commu.nextaction == cmd_reset) || (commu.nextaction == cmd_checkmove))
    {
        commu.len_m1 = pos[i][0];
        commu.len_m2 = pos[i][1];
        commu.len_m3 = pos[i][2];
        commu.len_m4 = pos[i][3];
        i++;
        if(i > 4)
        {
            i = 1;
        }
        commu.nextaction = cmd_move;
    }
    else
    {
        return;
    }
}
/**
* @brief  测试功能运行
* @attention 
*/
void TestRun(void)
{
    switch(TestCtrl.State)
    {
        case 0:
            if((TestCtrl.Id > 0) && (TestCtrl.Id < IO_END))
            {
                BspIOCtrl(TestCtrl.Id, TestCtrl.Ctrl); TestCtrl.Id=0;
            }
            else
            {
                switch(TestCtrl.Id)
                {
                    case 41: TestCtrl.Ctrl = GetCRC8(TestCtrl.ArrA, TestCtrl.ArrLen, 0x00); break;
                    case 42: TestCtrl.Ctrl = GetCRC8Reverse(TestCtrl.ArrA, TestCtrl.ArrLen, 0x00); break;
                    case 43: TestCrc(); break;
                    case 44: STMFLASH_Write(0x080E0000,(uint32_t*)TestCtrl.ArrA,TestCtrl.ArrLen); break;
                    case 45: STMFLASH_Write(0x080E0000,(uint32_t*)TestCtrl.ArrA,TestCtrl.ArrLen); break;
                    
                    case 51: Usart1Send(TestCtrl.ArrA, TestCtrl.ArrLen); break;
                    case 52: Usart2Send(TestCtrl.ArrA, TestCtrl.ArrLen); break;
                    case 53: Usart3Send(TestCtrl.ArrA, TestCtrl.ArrLen); break;
                    case 54: Uart4Send(TestCtrl.ArrA, TestCtrl.ArrLen); break;
                    case 55: MotorSpeedCtrl(TestCtrl.Ctrl, 1); break;
                    case 56: MotorSpeedCtrl(TestCtrl.Ctrl, 2); break;
                    case 57: MotorSpeedCtrl(TestCtrl.Ctrl, 3); break;
                    case 58: MotorSpeedCtrl(TestCtrl.Ctrl, 4); break;
                    case 61: MotorTest1(TestCtrl.Ctrl); break;
                    case 71: CtrlLed(TestCtrl.Ctrl); break;
                    case 72: CommTxFillMsg(TestCtrl.Ctrl); break;
                    case 81: calibration(); break;
                    case 91: CommCtrlMotorPos(TestCtrl.Ctrl, TestCtrl.Data1, 1, 1); break;
                    case 92: CommCtrlMotorPos(TestCtrl.Ctrl, TestCtrl.Data1, 2, 1); break;
                    case 93: CommCtrlMotorPos(TestCtrl.Ctrl, TestCtrl.Data1, 3, 1); break;
                    case 94: CommCtrlMotorPos(TestCtrl.Ctrl, TestCtrl.Data1, 4, 1); break;
                    case 95: CommGetMotorPos(1); break;
                    case 96: CommGetMotorPos(2); break;
                    case 97: CommGetMotorPos(3); break;
                    case 98: CommGetMotorPos(4); break;
                    case 101: MotorSpeedCtrl(TestCtrl.Ctrl, 1);
                              MotorSpeedCtrl(TestCtrl.Ctrl, 2);
                              MotorSpeedCtrl(TestCtrl.Ctrl, 3);
                              MotorSpeedCtrl(TestCtrl.Ctrl, 4);
                              break;
                    case 102: CommCtrlMotorPos(TestCtrl.Ctrl, TestCtrl.Data1, 1, 1);
                              CommCtrlMotorPos(TestCtrl.Ctrl, TestCtrl.Data1, 2, 1);
                              CommCtrlMotorPos(TestCtrl.Ctrl, TestCtrl.Data1, 3, 1);
                              CommCtrlMotorPos(TestCtrl.Ctrl, TestCtrl.Data1, 4, 1);
                              break;
                    default:break;
                }
                TestCtrl.Id=0;
            }
            break;
        case 1:
            switch(TestCtrl.Id)
            {
                case 1: TestUartMotorSpeed(); break;
                case 2: TestUartMotorPos(); break;
                case 3: TestUartMotorRun(TestCtrl.Data4); break;
                case 11: Motor1PosCtrl(TestCtrl.Ctrl); break;
                case 12: Motor2PosCtrl(TestCtrl.Ctrl); break;
                case 13: Motor3PosCtrl(TestCtrl.Ctrl); break;
                case 14: Motor4PosCtrl(TestCtrl.Ctrl); break;
                case 15: Motor1PosCtrl(TestCtrl.Ctrl);
                         Motor2PosCtrl(TestCtrl.Ctrl);
                         Motor3PosCtrl(TestCtrl.Ctrl);
                         Motor4PosCtrl(TestCtrl.Ctrl);
                case 21: TestXYZ(); break;
                         break;
                default:break;
            }
            break;
        default:break;
    }
    
}
