/**@file   bsp_cfg.h
* @brief   bsp½Ó¿Ú
* @author  ³Â×¿ÕÜ
* @date    2023/10/26
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef BSP_CFG_H_
#define BSP_CFG_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "bsp_mpu.h"
#include "bsp_io.h"
#include "bsp_timer.h"
#include "bsp_usart.h"
#include "bsp_spi.h"
#include "bsp_qspi.h"
#include "bsp_ads131m0x.h"
#include "bsp_ads131m08.h"
#include "bsp_i2c.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

//#ifndef   __WEAK
//  #define __WEAK        __attribute__((weak))
//#endif

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void spi1_receive(uint8_t *data, uint16_t len);
void spi2_receive(uint8_t *data, uint16_t len);
void spi3_receive(uint8_t *data, uint16_t len);
void spi4_receive(uint8_t *data, uint16_t len);
void spi6_receive(uint8_t *data, uint16_t len);
void qspi_receive(uint8_t *data, uint16_t len);
void uart3_receive(uint8_t *data, uint16_t len);
void uart4_receive(uint8_t *data, uint16_t len);
void uart5_receive(uint8_t *data, uint16_t len);
void uart6_receive(uint8_t *data, uint16_t len);
void uart7_receive(uint8_t *data, uint16_t len);
void uart8_receive(uint8_t *data, uint16_t len);
void Error_Handler(void);

#endif /* BSP_CFG_H_ */
