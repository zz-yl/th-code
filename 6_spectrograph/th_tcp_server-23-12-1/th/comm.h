/**@file   comm.h
* @brief   
* @author  ��׿��
* @date    2023/11/28
* @version 1.00.0.0
**************************************************************************************************/

#ifndef COMM_H_
#define COMM_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

//#include "bsp_sys.h"
#include "./SYSTEM/sys/sys.h"
#include "protocol.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

extern cmd_t comm_cmd;
extern msg_t comm_msg;

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

void comm_run(void);

#endif /* COMM_H_ */
