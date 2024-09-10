/**@file   crc_check.h
* @brief   crc校验,使用查表法,减少计算量
* @author  陈卓哲
* @date    2023/9/5
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef CRC_CHECK_H_
#define CRC_CHECK_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include <stdint.h>

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

uint8_t crc_get_crc8(uint8_t *data, uint8_t len, uint8_t start);
uint8_t crc_get_crc8_reverse(uint8_t *data, uint8_t len, uint8_t start);
uint16_t crc_get_crc16(uint8_t *Ptr, uint8_t Len);

#endif /* CRC_CHECK_H_ */
