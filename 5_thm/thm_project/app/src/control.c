/**@file   control.c
* @brief   简单外部设备控制
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "control.h"
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

led_data_t led_data;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/
/**
* @brief  led控制
*/
void led_ctrl(led_id_t id, uint8_t ctrl)
{
    switch(id)
    {
        case LED_ID_1:  IO_LED1(ctrl);  break;
        case LED_ID_2:  IO_LED2(ctrl);  break;
        case LED_ID_3:  IO_LED3(ctrl);  break;
        case LED_ID_4:  IO_LED4(ctrl);  break;
        case LED_ID_5:  IO_LED5(ctrl);  break;
        case LED_ID_6:  IO_LED6(ctrl);  break;
        case LED_ID_7:  IO_LED7(ctrl);  break;
        case LED_ID_8:  IO_LED8(ctrl);  break;
        case LED_ID_9:  IO_LED9(ctrl);  break;
        case LED_ID_10: IO_LED10(ctrl); break;
        case LED_ID_11: IO_LED11(ctrl); break;
        case LED_ID_12: IO_LED12(ctrl); break;
        case LED_ID_13: IO_LED13(ctrl); break;
        case LED_ID_14: IO_LED14(ctrl); break;
        case LED_ID_15: IO_LED15(ctrl); break;
        case LED_ID_16: IO_LED16(ctrl); break;
        case LED_ID_17: IO_LED17(ctrl); break;
        case LED_ID_18: IO_LED18(ctrl); break;
        default: break;
    }
}
/**
* @brief  led翻转
*/
void led_toggle(led_id_t id)
{
    switch(id)
    {
        case LED_ID_1:  IO_LED1_TOGGLE;  break;
        case LED_ID_2:  IO_LED2_TOGGLE;  break;
        case LED_ID_3:  IO_LED3_TOGGLE;  break;
        case LED_ID_4:  IO_LED4_TOGGLE;  break;
        case LED_ID_5:  IO_LED5_TOGGLE;  break;
        case LED_ID_6:  IO_LED6_TOGGLE;  break;
        case LED_ID_7:  IO_LED7_TOGGLE;  break;
        case LED_ID_8:  IO_LED8_TOGGLE;  break;
        case LED_ID_9:  IO_LED9_TOGGLE;  break;
        case LED_ID_10: IO_LED10_TOGGLE; break;
        case LED_ID_11: IO_LED11_TOGGLE; break;
        case LED_ID_12: IO_LED12_TOGGLE; break;
        case LED_ID_13: IO_LED13_TOGGLE; break;
        case LED_ID_14: IO_LED14_TOGGLE; break;
        case LED_ID_15: IO_LED15_TOGGLE; break;
        case LED_ID_16: IO_LED16_TOGGLE; break;
        case LED_ID_17: IO_LED17_TOGGLE; break;
        case LED_ID_18: IO_LED18_TOGGLE; break;
        default: break;
    }
}
void led_ctrl_all(uint8_t state, uint16_t frq)
{
    uint16_t id = 0;
    
    for(id=LED_ID_1; id<LED_NUM; id++)
    {
        led_data.state[id] = state;
        if(frq != 0xFF)  //频率可控
        {
            led_data.frq[id] = frq;
        }
    }
}
/**
* @brief  led控制运行
*/
void led_run(void)
{
    uint16_t id = 0;
    
    for(id=LED_ID_1; id<LED_NUM; id++)
    {
        if(led_data.frq[id])
        {
            led_data.cnt[id] = FRQ_TIMER / 2 / led_data.frq[id];

        }
        else
        {
            led_data.cnt[id] = 0;
        }
    }
}
/**
* @brief  led中断扫描
*/
void led_scan(void)
{
    uint16_t id = 0;
    
    for(id=LED_ID_1; id<LED_NUM; id++)
    {
        if(led_data.state[id])
        {
            if(led_data.cnt[id])
            {
                led_data.tim[id]++;
                if(led_data.tim[id] > led_data.cnt[id])
                {
                    led_data.tim[id] = 0;
                    led_toggle(id);
                }
            }
            else
            {
                led_ctrl(id, 1);
            }
        }
        else
        {
            led_ctrl(id, 0);
        }
    }
}
