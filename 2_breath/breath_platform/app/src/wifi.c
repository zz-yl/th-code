/**@file   wifi.c
* @brief   wifi¿ØÖÆ
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "wifi.h"
#include "comm.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/


/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

wifi_ctrl_t wifi_ctrl;

uint8_t str1[] = "AT+CWJAP=\"Health_5G_108\",\"th123456\"";
uint8_t str2[] = "AT+MQTTUSERCFG=0,1,\"TH-B100001\",\"espressif\",\"123456789\",0,0,\"/STM32/BreathCurve\"";
uint8_t str3[] = "AT+MQTTCONN=0,\"192.168.1.14\",1883,1";
uint8_t str4[] = "AT+MQTTSUB=0,\"/STM32/BreathCurve\",1";
uint8_t str5[] = "AT+MQTTPUB=0,\"/STM32/BreathCurve\",\"-1000;-201;0;0;0;0;00000000000000000000000000000000000000000000000000000000\",1,0";
uint8_t str6[] = "AT+MQTTCLEAN=0";

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief wifiÔËÐÐ
* @attention 
*/
void wifi_run(void)
{
    static uint16_t tim = 0;
    
    switch(wifi_ctrl.state)
    {
        case WIFI_RESET: 
//            if()
//            {
//                
//            }
            break;
        case WIFI_CONNECT: comm_wifi_send(str2, sizeof(str1)-1); break;
        case WIFI_MQTT: comm_wifi_send(str3, sizeof(str1)-1); break;
        case WIFI_TCP: comm_wifi_send(str4, sizeof(str1)-1); break;
        case WIFI_SEND: comm_wifi_send(str5, sizeof(str1)-1); break;
        default: break;
    }
    
    tim++;
    if(tim > 100)
    {
        tim = 0;
        switch(wifi_ctrl.state)
        {
            case WIFI_RESET: comm_wifi_send(str1, sizeof(str1)-1); break;
            case WIFI_CONNECT: comm_wifi_send(str2, sizeof(str1)-1); break;
            case WIFI_MQTT: comm_wifi_send(str3, sizeof(str1)-1); break;
            case WIFI_TCP: comm_wifi_send(str4, sizeof(str1)-1); break;
            case WIFI_SEND: comm_wifi_send(str5, sizeof(str1)-1); break;
            default: break;
        }
    }
}
