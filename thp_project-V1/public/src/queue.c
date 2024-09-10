/**@file   queue.c
* @brief   队列
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "queue.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/


/**************************************************************************************************
*                                      data TYPES
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
QUEUE_STATE QueueInit(QUEUE_LIST *qp)
{
    if(qp == NULL)
    {
        return QUEUE_ERR;
    }
    if((qp->size == 0) || (qp->size > QUEUE_MAX_SIZE))
    {
        qp = NULL;
        return QUEUE_ERR;
    }

    QueueClear(qp);

    return QUEUE_SUCCESS;
}
/**
* @brief  判断队列是否空,空返回1
*/
uint16_t QueueIsEmpty(QUEUE_LIST *qp)
{
    return (qp->front == qp->rear);
}
/**
* @brief  判断队列是否满,满返回1
*/
uint16_t QueueIsFull(QUEUE_LIST *qp)
{
    return ((qp->rear + 1) % qp->size == qp->front);
}
/**
* @brief  删除一个数据
*/
QUEUE_STATE QueueDelete(QUEUE_LIST *qp)
{
    if(QueueIsEmpty(qp))
        return QUEUE_EMPTY;

    qp->front = (qp->front + 1) % qp->size;

    return QUEUE_SUCCESS;
}
/**
* @brief  获取当前队列数据长度
*/
uint16_t QueueLength(QUEUE_LIST *qp)
{
    return ((qp->rear + qp->size - qp->front) % qp->size);
}
/**
* @brief  队列数据清空
*/
void QueueClear(QUEUE_LIST *qp)
{
    qp->front = 0;
    qp->rear  = 0;
    qp->state = QUEUE_FREE;
}
/**
* @brief  删除多个数据
*/
uint16_t QueueDeleteData(QUEUE_LIST *qp, uint16_t len)
{
    uint16_t count = 0;
    
    while(len--)
    {
        if(QueueDelete(qp) != QUEUE_SUCCESS)
        {
            return count;
        }
        count++;
    }
    
    return count;
}
/*U8类型************************************************************************************************/
/**
* @brief  入队
*/
QUEUE_STATE QueueInputU8(QUEUE_LIST *qp, uint8_t data)
{
    if(qp == NULL)
        return QUEUE_FULL;
    if(QueueIsFull(qp))
        return QUEUE_FULL;

    *((uint8_t *)qp->data + qp->rear) = data;
    qp->rear = (qp->rear + 1) % qp->size;

    return QUEUE_SUCCESS;
}
/**
* @brief  出队
*/
QUEUE_STATE QueueOutputU8(QUEUE_LIST *qp, uint8_t *data)
{
    if(qp == NULL)
        return QUEUE_FULL;
    if(QueueIsEmpty(qp))
        return QUEUE_EMPTY;

    *data = *((uint8_t *)qp->data + qp->front);
    qp->front = (qp->front + 1) % qp->size;

    return QUEUE_SUCCESS;
}
/**
* @brief  多数据入队
* @attention 返回值:成功入队数据个数
*/
uint16_t QueueInputDataU8(QUEUE_LIST *qp, uint8_t *data, uint16_t len)
{
    uint16_t count = 0;
    
    if(qp->state == QUEUE_BUSY)
    {
        return 0;
    }
    qp->state = QUEUE_BUSY;
    while(len--)
    {
        if(QueueInputU8(qp, *(data++)) != QUEUE_SUCCESS)
        {
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
uint16_t QueueOutputDataU8(QUEUE_LIST *qp, uint8_t *data, uint16_t len)
{
    uint16_t count = 0;
    
    if(qp->state == QUEUE_BUSY)
    {
        return 0;
    }
    qp->state = QUEUE_BUSY;
    while(count < len)
    {
        if(QueueOutputU8(qp, &data[count]) != QUEUE_SUCCESS)  //出队
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
uint16_t QueueReadDataU8(QUEUE_LIST *qp, uint8_t *data, uint16_t len)
{
    uint16_t count = 0;
    uint16_t pre = qp->front;
    
    if(qp == NULL)
    {
        return 0;
    }

    while((QueueIsEmpty(qp) != 1) && (count < len))
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
QUEUE_STATE QueueInputFl(QUEUE_LIST *qp, float data)
{
    if(qp == NULL)
        return QUEUE_FULL;
    if(QueueIsFull(qp))
        return QUEUE_FULL;
    
    *((float *)qp->data + qp->rear) = data;
    qp->rear = (qp->rear + 1) % qp->size;

    return QUEUE_SUCCESS;
}
/**
* @brief  出队
*/
QUEUE_STATE QueueOutputFl(QUEUE_LIST *qp, float *data)
{
    if(qp == NULL)
        return QUEUE_FULL;
    if(QueueIsEmpty(qp))
        return QUEUE_EMPTY;

    *data = *((float *)qp->data + qp->front);
    qp->front = (qp->front + 1) % qp->size;

    return QUEUE_SUCCESS;
}
/**
* @brief  多数据入队
* @attention 返回值:成功入队数据个数
*/
uint16_t QueueInputDataFl(QUEUE_LIST *qp, float *data, uint16_t len)
{
    uint16_t count = 0;
    
    if(qp->state == QUEUE_BUSY)
    {
        return 0;
    }
    qp->state = QUEUE_BUSY;
    while(len--)
    {
        if(QueueInputFl(qp, *(data++)) != QUEUE_SUCCESS)
        {
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
uint16_t QueueOutputDataFl(QUEUE_LIST *qp, float *data, uint16_t len)
{
    uint16_t count = 0;
    
    if(qp->state == QUEUE_BUSY)
    {
        return 0;
    }
    qp->state = QUEUE_BUSY;
    while(count < len)
    {
        if(QueueOutputFl(qp, &data[count]) != QUEUE_SUCCESS)  //出队
        {
            qp->state = QUEUE_FREE;
            return count;
        }
        count++;
    }
    qp->state = QUEUE_FREE;
    
    return count;
}
