/**@file   data_queue.h
* @brief   队列
* @author  陈卓哲
* @date    2023/9/8
* @section
* -# 
*
* @version 1.00.0.0
**************************************************************************************************/

#ifndef DATA_QUEUE_H_
#define DATA_QUEUE_H_

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include <stdint.h>
#include <stdio.h>

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/

#define QUEUE_MAX_SIZE 0xFFFF  ///< 队列最大长度

/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/

/**
* @enum    queue_state_t
* @brief   队列状态
*/
typedef enum
{
	QUEUE_SUCCESS,
    QUEUE_ERR,
    QUEUE_EMPTY,
    QUEUE_FULL,
    QUEUE_FREE,
    QUEUE_BUSY
}queue_state_t;
/**
* @struct  queue_list
* @brief   线性队列
*/
typedef struct
{
	void *data;     //数据
	uint16_t front; //队首
	uint16_t rear;  //队尾
    uint16_t size;  //数组总长度
    queue_state_t state;  //队列状态
}queue_list_t;

/**************************************************************************************************
*                                      GLOBAL VARIABLES
**************************************************************************************************/

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

uint16_t queue_is_empty(queue_list_t *qp);
uint16_t queue_is_full(queue_list_t *qp);
queue_state_t queue_init(queue_list_t *qp, void *data, uint16_t size);
uint16_t queue_length(queue_list_t *qp);
void queue_clear(queue_list_t *qp);
queue_state_t queue_delete_single(queue_list_t *qp);
uint16_t queue_delete(queue_list_t *qp, uint16_t len);

queue_state_t queue_input_u8_single(queue_list_t *qp, uint8_t data);
queue_state_t queue_output_u8_single(queue_list_t *qp, uint8_t *data);
queue_state_t queue_input_fl_single(queue_list_t *qp, float data);
queue_state_t queue_output_fl_single(queue_list_t *qp, float *data);

uint16_t queue_input_u8(queue_list_t *qp, uint8_t *data, uint16_t len);
uint16_t queue_output_u8(queue_list_t *qp, uint8_t *data, uint16_t len);
uint16_t queue_read_u8(queue_list_t *qp, uint8_t *data, uint16_t len);
uint16_t queue_input_fl(queue_list_t *qp, float *data, uint16_t len);
uint16_t queue_output_fl(queue_list_t *qp, float *data, uint16_t len);

#endif /* DATA_QUEUE_H_ */
