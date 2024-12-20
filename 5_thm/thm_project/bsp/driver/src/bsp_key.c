/**@file   bsp_key.c
* @brief   key驱动
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_key.h"
#include "bsp_cfg.h"
#include "control.h"
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

key_data_t key_data;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  key按下
* @attention 
*/
static void key_down(key_id_t id)
{
    switch(id)
    {
        case KEY_ID1:  break;
        case KEY_ID2:  break;
        case KEY_ID3:  break;
        case KEY_ID4:  break;
        default: break;
    }
}
/**
* @brief  key抬起
* @attention 
*/
static void key_up(key_id_t id)
{
    uint8_t led_state = 0;
    
    switch(id)
    {
        case KEY_ID1: //所有灯全亮或全灭
            led_state = led_data.state[0];
            led_state = !led_state;
            led_ctrl_all(led_state, 0xFF);
            break;
        case KEY_ID2:  break;
        case KEY_ID3:  break;
        case KEY_ID4:  //保存当前led状态
            stmflash_write(MEM_ADDR, (uint32_t *)&led_data, sizeof(led_data));
            break;
        default: break;
    }
}
/**
* @brief  key获取状态
* @attention 
*/
static uint8_t key_get_state(key_id_t id)
{
    uint8_t ret = 0;
    
    switch(id)
    {
        case KEY_ID1: ret = IO_KEY1; break;
        case KEY_ID2: ret = IO_KEY2; break;
        case KEY_ID3: ret = IO_KEY3; break;
        case KEY_ID4: ret = IO_KEY4; break;
        default: break;
    }
    
    return ret;
}
/**
* @brief  key扫描
* @attention 
*/
static void key_scan(void)
{
    uint8_t i = 0;
    
    for(i=0; i<KEY_NUM; i++)
    {
        key_data.buf[i] <<= 1;
        key_data.buf[i] |= key_get_state(i);
        if(key_data.buf[i] == 0xFF)
        {
            key_data.state[i] = 1;
        }
        if(key_data.buf[i] == 0x00)
        {
            key_data.state[i] = 0;
        }
    }
}
/**
* @brief  key运行
* @attention 
*/
void key_run(void)
{
    uint8_t i = 0;
    
    key_scan();
    
    for(i=0; i<KEY_NUM; i++)
    {
        if(key_data.state_last[i] != key_data.state[i])
        {
            key_data.state_last[i] = key_data.state[i];
            if(key_data.state[i])  //按下
            {
                key_down(i);
            }
            else  //抬起
            {
                key_up(i);
            }
        }
    }
    
}
