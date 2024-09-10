/**@file   queue.h
* @brief   ����
* @author  ��׿��
* @date    2023/9/8
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef QUEUE_H_
#define QUEUE_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include <stdint.h>
#include <stdio.h>

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define QUEUE_MAX_SIZE 0xFFFF  ///< ������󳤶�

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @enum    QUEUE_STATE
* @brief   ����״̬
*/
typedef enum
{
	QUEUE_SUCCESS,
    QUEUE_ERR,
    QUEUE_EMPTY,
    QUEUE_FULL,
    QUEUE_FREE,
    QUEUE_BUSY
}QUEUE_STATE;
/**
* @struct  QUEUE_LIST
* @brief   ���Զ���
*/
typedef struct
{
	void *data;     //����
	uint16_t front; //����
	uint16_t rear;  //��β
    uint16_t size;  //�����ܳ���
    QUEUE_STATE state;  //����״̬
}QUEUE_LIST;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

uint16_t QueueIsEmpty(QUEUE_LIST *qp);
uint16_t QueueIsFull(QUEUE_LIST *qp);
QUEUE_STATE QueueInit(QUEUE_LIST *qp);
QUEUE_STATE QueueDelete(QUEUE_LIST *qp);
uint16_t QueueLength(QUEUE_LIST *qp);
void QueueClear(QUEUE_LIST *qp);
uint16_t QueueDeleteData(QUEUE_LIST *qp, uint16_t len);

QUEUE_STATE QueueInputU8(QUEUE_LIST *qp, uint8_t data);
QUEUE_STATE QueueOutputU8(QUEUE_LIST *qp, uint8_t *data);
QUEUE_STATE QueueInputFl(QUEUE_LIST *qp, float data);
QUEUE_STATE QueueOutputFl(QUEUE_LIST *qp, float *data);

uint16_t QueueInputDataU8(QUEUE_LIST *qp, uint8_t *data, uint16_t len);
uint16_t QueueOutputDataU8(QUEUE_LIST *qp, uint8_t *data, uint16_t len);
uint16_t QueueReadDataU8(QUEUE_LIST *qp, uint8_t *data, uint16_t len);
uint16_t QueueInputDataFl(QUEUE_LIST *qp, float *data, uint16_t len);
uint16_t QueueOutputDataFl(QUEUE_LIST *qp, float *data, uint16_t len);

#endif /* QUEUE_H_ */
