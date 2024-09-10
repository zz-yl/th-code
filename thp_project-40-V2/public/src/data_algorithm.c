/**@file   data_algorithm.c
* @brief   �����㷨
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
* @brief  ���������˲�
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
* @brief  ��λ���˲�
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
* @brief  ��ͨ�˲�
* ��ֹƵ�ʹ�ʽ u_o(k) = u_o(k-1)+Ts*2*Pi*fc*(u_i(k)-u_o(k-1))
*/
float filter_low_pass(float filter, float data, float per_now)
{
    float per_filter = 0;
    
    per_filter = 1 - per_now;
    filter = (filter * per_filter)+ (data * per_now);

    return filter;
}
/**
* @brief     һά�ֶ����Բ�ֵ
*            ����ɢ����ֱ������,����������ֵx0��Ӧ��y0
* @param[in] x0:����x
* @param[in] x_list:x�����ֵ��
* @param[in] y_list:y�����ֵ��
* @param[in] size:��ֵ���С
* @param[in] limit:�Ƿ��б߽�����,0:�ޱ߽�;1:���Ʊ߽�
* @retval    y0
*/
float interp_1d(float x0, const float *x_list, const float *y_list, uint16_t size, uint8_t limit)
{
    uint16_t i = 0;
    float k_val = 0;  //б��k
    float b_val = 0;  //�ؾ�b

    if(x_list[0] < x_list[1])  //x����
    {
        if(limit)  //�߽�����
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
        //����X����Ӧ������
        for(i=1; i<size-1; i++)
        {
            if(x0 <= x_list[i])
            {
                break;
            }
        }
    }
    else  //x�ݼ�
    {
        if(limit)  //�߽�����
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
        //����x����Ӧ������
        for(i=1; i<size-1; i++)
        {
            if(x0 >= x_list[i])
            {
                break;
            }
        }
    }
    if((x_list[i] - x_list[i - 1]) == 0)  //�����0
    {
        return 0;
    }
    k_val = (y_list[i] - y_list[i - 1]) / (x_list[i] - x_list[i - 1]);  //����б��k
    b_val = y_list[i] - (x_list[i] * k_val);  //����ؾ�b
    
    return (x0 * k_val + b_val);  //y=kx+b
}
/**
* @brief ��ά���Բ�ֵ
*/
double interp_2d(double x1, double *x1_list, uint16_t x1_len, double x2, double *x2_list, uint16_t x2_len, double y_list[][11])
{
    uint16_t i = 0, j = 0;
    double k1 = 0;  //б��k1
    double k2 = 0;  //б��k2
    double b2 = 0;  //�ؾ�b2
    double b1 = 0;  //�ؾ�b1
    double y1 = 0;  //y1
    double y2 = 0;  //y2
    double ret = 0;  //���

    //����x1����Ӧ������
    for(i=1; i<x1_len-1; i++)
    {
        if(x1 < x1_list[i])
        {
            break;
        }
    }
    //����x2����Ӧ������
    for(j=1; j<x2_len-1; j++)
    {
        if(x2 < x2_list[j])
        {
            break;
        }
    }

    k1 = (y_list[i][j-1] - y_list[i-1][j-1]) / (x1_list[i] - x1_list[i - 1]);  //����б��k
    b1 = y_list[i][j-1] - (x1_list[i] * k1);  //����ؾ�b1
    k2 = (y_list[i][j] - y_list[i-1][j]) / (x1_list[i] - x1_list[i - 1]);  //����б��k
    b2 = y_list[i][j] - (x1_list[i] * k2);  //����ؾ�b2
    y1 = x1 * k1 + b1;  //y=kx+b
    y2 = x1 * k2 + b2;  //y=kx+b
    ret = y1 + (y2 - y1) * ((x2 - x2_list[j-1]) / (x2_list[j] - x2_list[j-1]));
    
    return ret;
}
