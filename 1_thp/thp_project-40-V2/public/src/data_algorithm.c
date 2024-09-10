/**@file   data_algorithm.c
* @brief   数据算法
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "data_algorithm.h"
#include "string.h"

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
* @brief  滑动窗口滤波
*/
float filter_moving_average(queue_list_t *queue, float in)
{
    float    sum = 0;
    uint16_t i = 0;
    uint16_t len = 0;

    if (queue_is_full(queue)) {
        queue_delete_single(queue);
    }
    queue_input_fl_single(queue, in);

    len = queue_length(queue);
    for (i = 0; i < len; i++) {
        sum += *((float *)queue->data + (queue->front+i) % queue->size);
    }
    
    return sum /= len;
}
/**
* @brief  中位数滤波
*/
float filter_median(queue_list_t *queue, float in)
{
    uint16_t i = 0;
    uint16_t j = 0;
    uint16_t len = 0;
    float arr[20];
    float temp = 0;
    if (queue_is_full(queue)) {
        queue_delete_single(queue);
    }
    queue_input_fl_single(queue, in);
    len = queue_length(queue);
    
    memcpy(arr, queue->data, len * sizeof(float));
    for(i = 0;i < len;i++){
        for(j = 0;j < len -1-i;j++){
            if(arr[j] > arr[j+1]){
                temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
            }
        }
    }
    
    if(len%2)
        return arr[len/2];
    else
        return (arr[len/2]+arr[len/2 -1])/2;            
}
/**
* @brief  低通滤波
* 截止频率公式 u_o(k) = u_o(k-1)+Ts*2*Pi*fc*(u_i(k)-u_o(k-1))
*/
float filter_low_pass(float filter, float data, float per_now)
{
    float per_filter = 0;
    
    per_filter = 1 - per_now;
    filter = (filter * per_filter)+ (data * per_now);

    return filter;
}
/**
* @brief     一维分段线性插值
*            将离散点用直线连接,并返回输入值x0对应的y0
* @param[in] x0:输入x
* @param[in] x_list:x坐标插值表
* @param[in] y_list:y坐标插值表
* @param[in] size:插值表大小
* @param[in] limit:是否有边界限制,0:无边界;1:限制边界
* @retval    y0
*/
float interp_1d(float x0, const float *x_list, const float *y_list, uint16_t size, uint8_t limit)
{
    uint16_t i = 0;
    float k_val = 0;  //斜率k
    float b_val = 0;  //截距b

    if(x_list[0] < x_list[1])  //x递增
    {
        if(limit)  //边界限制
        {
            if(x0 <= x_list[0])
            {
                return y_list[0];
            }
            else if(x0 >= x_list[size-1])
            {
                return y_list[size-1];
            }
        }
        //查找X所对应的区间
        for(i=1; i<size-1; i++)
        {
            if(x0 <= x_list[i])
            {
                break;
            }
        }
    }
    else  //x递减
    {
        if(limit)  //边界限制
        {
            if(x0 >= x_list[0])
            {
                return y_list[0];
            }
            else if(x0 <= x_list[size-1])
            {
                return y_list[size-1];
            }
        }
        //查找x所对应的区间
        for(i=1; i<size-1; i++)
        {
            if(x0 >= x_list[i])
            {
                break;
            }
        }
    }
    if((x_list[i] - x_list[i - 1]) == 0)  //避免除0
    {
        return 0;
    }
    k_val = (y_list[i] - y_list[i - 1]) / (x_list[i] - x_list[i - 1]);  //计算斜率k
    b_val = y_list[i] - (x_list[i] * k_val);  //计算截距b
    
    return (x0 * k_val + b_val);  //y=kx+b
}
/**
* @brief 二维线性插值
*/
double interp_2d(double x1, double *x1_list, uint16_t x1_len, double x2, double *x2_list, uint16_t x2_len, double y_list[][11])
{
    uint16_t i = 0, j = 0;
    double k1 = 0;  //斜率k1
    double k2 = 0;  //斜率k2
    double b2 = 0;  //截距b2
    double b1 = 0;  //截距b1
    double y1 = 0;  //y1
    double y2 = 0;  //y2
    double ret = 0;  //结果

    //查找x1所对应的区间
    for(i=1; i<x1_len-1; i++)
    {
        if(x1 < x1_list[i])
        {
            break;
        }
    }
    //查找x2所对应的区间
    for(j=1; j<x2_len-1; j++)
    {
        if(x2 < x2_list[j])
        {
            break;
        }
    }

    k1 = (y_list[i][j-1] - y_list[i-1][j-1]) / (x1_list[i] - x1_list[i - 1]);  //计算斜率k
    b1 = y_list[i][j-1] - (x1_list[i] * k1);  //计算截距b1
    k2 = (y_list[i][j] - y_list[i-1][j]) / (x1_list[i] - x1_list[i - 1]);  //计算斜率k
    b2 = y_list[i][j] - (x1_list[i] * k2);  //计算截距b2
    y1 = x1 * k1 + b1;  //y=kx+b
    y2 = x1 * k2 + b2;  //y=kx+b
    ret = y1 + (y2 - y1) * ((x2 - x2_list[j-1]) / (x2_list[j] - x2_list[j-1]));
    
    return ret;
}
