/**@file   data_queue.c
* @brief   队列
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "data_queue.h"

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
* @brief  队列初始化
*/
queue_state_t queue_init(queue_list_t *qp, void *data, uint16_t size)
{
    if(qp == NULL)
    {
        qp->state = QUEUE_ERR;
        return qp->state;
    }
    qp->state = QUEUE_BUSY;
    if(data == NULL)
    {
        qp->state = QUEUE_ERR;
        return qp->state;
    }
    if((size == 0) || (size > QUEUE_MAX_SIZE))
    {
        qp->state = QUEUE_ERR;
        return qp->state;
    }
    qp->data = data;
    qp->size = size;

    queue_clear(qp);

    return QUEUE_SUCCESS;
}
/**
* @brief  判断队列是否空,空返回1
*/
uint16_t queue_is_empty(queue_list_t *qp)
{
    return (qp->front == qp->rear);
}
/**
* @brief  判断队列是否满,满返回1
*/
uint16_t queue_is_full(queue_list_t *qp)
{
    return ((qp->rear + 1) % qp->size == qp->front);
}
/**
* @brief  获取当前队列数据长度
*/
uint16_t queue_length(queue_list_t *qp)
{
    return ((qp->rear + qp->size - qp->front) % qp->size);
}
/**
* @brief  队列数据清空
*/
void queue_clear(queue_list_t *qp)
{
    qp->front = 0;
    qp->rear  = 0;
    qp->state = QUEUE_FREE;
}
/**
* @brief  删除一个数据
*/
queue_state_t queue_delete_single(queue_list_t *qp)
{
    if(queue_is_empty(qp))
        return QUEUE_EMPTY;

    qp->front = (qp->front + 1) % qp->size;

    return QUEUE_SUCCESS;
}
/**
* @brief  删除多个数据
*/
uint16_t queue_delete(queue_list_t *qp, uint16_t len)
{
    uint16_t count = 0;
    
    qp->state = QUEUE_BUSY;
    while(len--)
    {
        if(queue_delete_single(qp) != QUEUE_SUCCESS)
        {
            return count;
        }
        count++;
    }
    qp->state = QUEUE_FREE;
    
    return count;
}
/*U8类型************************************************************************************************/
/**
* @brief  入队
*/
queue_state_t queue_input_u8_single(queue_list_t *qp, uint8_t data)
{
    if(qp == NULL)
        return QUEUE_FULL;
    if(queue_is_full(qp))
        return QUEUE_FULL;

    *((uint8_t *)qp->data + qp->rear) = data;
    qp->rear = (qp->rear + 1) % qp->size;

    return QUEUE_SUCCESS;
}
/**
* @brief  出队
*/
queue_state_t queue_output_u8_single(queue_list_t *qp, uint8_t *data)
{
    if(qp == NULL)
        return QUEUE_FULL;
    if(queue_is_empty(qp))
        return QUEUE_EMPTY;

    *data = *((uint8_t *)qp->data + qp->front);
    qp->front = (qp->front + 1) % qp->size;

    return QUEUE_SUCCESS;
}
/**
* @brief  多数据入队
* @attention 返回值:成功入队数据个数
*/
uint16_t queue_input_u8(queue_list_t *qp, uint8_t *data, uint16_t len)
{
    uint16_t count = 0;
    
    if(qp->state != QUEUE_FREE)
    {
        return 0;
    }
    qp->state = QUEUE_BUSY;
    while(len--)
    {
        if(queue_input_u8_single(qp, *(data++)) != QUEUE_SUCCESS)
        {
            qp->state = QUEUE_FREE;
            return count;
        }
        count++;
    }
    qp->state = QUEUE_FREE;
    
    return count;
}
/**
* @brief  多数据出队
* @attention 返回值:成功出队数据个数
*/
uint16_t queue_output_u8(queue_list_t *qp, uint8_t *data, uint16_t len)
{
    uint16_t count = 0;
    
    if(qp->state != QUEUE_FREE)
    {
        return 0;
    }
    qp->state = QUEUE_BUSY;
    while(count < len)
    {
        if(queue_output_u8_single(qp, &data[count]) != QUEUE_SUCCESS)  //出队
        {
            qp->state = QUEUE_FREE;
            return count;
        }
        count++;
    }
    qp->state = QUEUE_FREE;
    
    return count;
}
/**
* @brief  多数据读取
* @attention 读取数据时不出队,返回值:成功读取数据个数
*/
uint16_t queue_read_u8(queue_list_t *qp, uint8_t *data, uint16_t len)
{
    uint16_t count = 0;
    uint16_t pre = qp->front;
    
    if(qp == NULL)
    {
        return 0;
    }

    while((queue_is_empty(qp) != 1) && (count < len))
    {
        data[count] = *((uint8_t *)qp->data + pre);
        pre = (pre + 1) % qp->size;
        count++;
    }
    
    return count;
}
/*float类型************************************************************************************************/
/**
* @brief  入队
*/
queue_state_t queue_input_fl_single(queue_list_t *qp, float data)
{
    if(qp == NULL)
        return QUEUE_FULL;
    if(queue_is_full(qp))
        return QUEUE_FULL;
    
    *((float *)qp->data + qp->rear) = data;
    qp->rear = (qp->rear + 1) % qp->size;

    return QUEUE_SUCCESS;
}
/**
* @brief  出队
*/
queue_state_t queue_output_fl_single(queue_list_t *qp, float *data)
{
    if(qp == NULL)
        return QUEUE_FULL;
    if(queue_is_empty(qp))
        return QUEUE_EMPTY;

    *data = *((float *)qp->data + qp->front);
    qp->front = (qp->front + 1) % qp->size;

    return QUEUE_SUCCESS;
}
/**
* @brief  多数据入队
* @attention 返回值:成功入队数据个数
*/
uint16_t queue_input_fl(queue_list_t *qp, float *data, uint16_t len)
{
    uint16_t count = 0;
    
    if(qp->state != QUEUE_FREE)
    {
        return 0;
    }
    qp->state = QUEUE_BUSY;
    while(len--)
    {
        if(queue_input_fl_single(qp, *(data++)) != QUEUE_SUCCESS)
        {
            qp->state = QUEUE_FREE;
            return count;
        }
        count++;
    }
    qp->state = QUEUE_FREE;
    
    return count;
}
/**
* @brief  多数据出队
* @attention 返回值:成功出队数据个数
*/
uint16_t queue_output_fl(queue_list_t *qp, float *data, uint16_t len)
{
    uint16_t count = 0;
    
    if(qp->state != QUEUE_FREE)
    {
        return 0;
    }
    qp->state = QUEUE_BUSY;
    while(count < len)
    {
        if(queue_output_fl_single(qp, &data[count]) != QUEUE_SUCCESS)  //出队
        {
            qp->state = QUEUE_FREE;
            return count;
        }
        count++;
    }
    qp->state = QUEUE_FREE;
    
    return count;
}
