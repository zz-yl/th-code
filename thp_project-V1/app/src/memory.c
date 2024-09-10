/**@file   memory.c
* @brief   ´æ´¢Ä£¿é
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "memory.h"
#include "sys_cfg.h"
#include "interp_table.h"

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
* @brief  Ð´SN
* @attention 
*/
void MemSNWrite(uint8_t *data, uint8_t len)
{
    uint8_t i = 0;
    uint8_t buf[32] = {0};
    
    for(i=0; i<15; i++)
    {
        buf[i]=SN[i];
    }
    STMFLASH_Write(FLASH_PAR_ADDR, (uint32_t*)buf, 4);
}
/**
* @brief  ´æ´¢Êý¾Ý¶ÁÈ¡
* @attention 
*/
void MemRead(void)
{
    uint16_t i = 0;
    uint8_t buf[50] = {0};
    uint32_t buf_mm[600];
    
    MotorType = STMFLASH_ReadWord(FLASH_MID_ADDR);
    if(MotorType > SYS_MOTOR100)
    {
        MotorType = SYS_MOTOR60;
    }
    STMFLASH_Read(FLASH_ADDR,(uint32_t*)buf_mm,600);
    for(int i=0;i<150;i++)
    {
        DeviceList.x_pos1[i] = buf_mm[i];
        DeviceList.x_pos2[i] = buf_mm[150+i];
        DeviceList.x_pos3[i] = buf_mm[300+i];
        DeviceList.x_pos4[i] = buf_mm[450+i];
    }
    switch(MotorType)
    {
        case SYS_MOTOR60:
            for(i=0; i<TableMax[MotorType]; i++)
            {
                DeviceList.y_pos[i] = DeviceList.y_pos_60[i];
            }
            break;
        case SYS_MOTOR100:
            for(i=0; i<TableMax[MotorType]; i++)
            {
                DeviceList.y_pos[i] = DeviceList.y_pos_100[i];
            }
            break;
        default: break;
    }
    
    STMFLASH_Read(FLASH_PAR_ADDR, (uint32_t*)buf, 10);
    for(i=0; i<15; i++)
    {
        SN[i] = buf[i];
    }
}
