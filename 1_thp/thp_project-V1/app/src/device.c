/**@file   device.c
* @brief   设备
* @version 1.00.0.0
**************************************************************************************************/

/**************************************************************************************************
*                                      INCLUDE FILES
**************************************************************************************************/

#include "device.h"
#include "data_algorithm.h"
#include "interp_table.h"

#include "control.h"

/**************************************************************************************************
*                                      MACROS DEFINE
**************************************************************************************************/


/**************************************************************************************************
*                                      DATA TYPES
**************************************************************************************************/


/**************************************************************************************************
*                                      VARIABLES
**************************************************************************************************/

VOLTAGE_DATA VolData;
DEVICE_IO InData        = {0};
DEVICE_IO OutData       = {0};
DEVICE_IO InFilterData  = {0};
DEVICE_IO OutFilterData = {0};

/**************************************************************************************************
*                                      FUNCTION PROTOTYPES
**************************************************************************************************/

/**
* @brief  电压采集
* @attention 
*/
static void DevVoltage(void)
{
    readData(&adc_data);
    adc_data.channel0 &= 0x7FFFFF;  //去负号
    adc_data.channel1 &= 0x7FFFFF;  //去负号
    adc_data.channel2 &= 0x7FFFFF;  //去负号
    adc_data.channel3 &= 0x7FFFFF;  //去负号
    adc_data.channel4 &= 0x7FFFFF;  //去负号
    adc_data.channel5 &= 0x7FFFFF;  //去负号
    adc_data.channel6 &= 0x7FFFFF;  //去负号
    VolData.adc_out0 = (float)adc_data.channel0 * 1.2 / 0x7FFFFF;
    VolData.adc_out1 = (float)adc_data.channel1 * 1.2 / 0x7FFFFF;
    VolData.adc_out2 = (float)adc_data.channel2 * 1.2 / 0x7FFFFF;
    VolData.adc_out3 = (float)adc_data.channel3 * 1.2 / 0x7FFFFF;
    VolData.adc_out4 = (float)adc_data.channel4 * 1.2 / 0x7FFFFF;
    VolData.adc_out5 = (float)adc_data.channel5 * 1.2 / 0x7FFFFF;
    VolData.adc_out6 = (float)adc_data.channel6 * 1.2 / 0x7FFFFF;
    
//    InData.pos1 = VolData.adc_out0;
//    InData.pos2 = VolData.adc_out1;
//    InData.pos3 = VolData.adc_out2;
//    InData.pos4 = VolData.adc_out3;
    InData.pos1 = adc_data.channel0;
    InData.pos2 = adc_data.channel1;
    InData.pos3 = adc_data.channel2;
    InData.pos4 = adc_data.channel3;
}
/**
* @brief  数据滤波
* @attention 
*/
static void DevFilter(void)
{
    InFilterData.pos1 = LowPassFilter(InFilterData.pos1, InData.pos1);
    InFilterData.pos2 = LowPassFilter(InFilterData.pos2, InData.pos2);
    InFilterData.pos3 = LowPassFilter(InFilterData.pos3, InData.pos3);
    InFilterData.pos4 = LowPassFilter(InFilterData.pos4, InData.pos4);
}
/**
* @brief  数据转换
* @attention 
*/
static void DevConversion(void)
{
    float data = 0;
    float fdata = 0;
    /* p1 */
    data = Interp1DCore(InData.pos1, DeviceList.x_pos1, DeviceList.y_pos, TableMax[MotorType], LIMIT_UEN);
    data = Limit(data, LengthMax[MotorType], 0);  //上下限
    //滤波
    fdata = Interp1DCore(InFilterData.pos1, DeviceList.x_pos1, DeviceList.y_pos, TableMax[MotorType], LIMIT_UEN);
    fdata = Limit(fdata, LengthMax[MotorType], 0);  //上下限
    
    OutData.pos1 = data;
    OutFilterData.pos1 = fdata;
    /* p2 */
    data = Interp1DCore(InData.pos2, DeviceList.x_pos2, DeviceList.y_pos, TableMax[MotorType], LIMIT_UEN);
    data = Limit(data, LengthMax[MotorType], 0);  //上下限
    //滤波
    fdata = Interp1DCore(InFilterData.pos2, DeviceList.x_pos2, DeviceList.y_pos, TableMax[MotorType], LIMIT_UEN);
    fdata = Limit(fdata, LengthMax[MotorType], 0);  //上下限
    
    OutData.pos2 = data;
    OutFilterData.pos2 = fdata;
    /* p3 */
    data = Interp1DCore(InData.pos3, DeviceList.x_pos3, DeviceList.y_pos, TableMax[MotorType], LIMIT_UEN);
    data = Limit(data, LengthMax[MotorType], 0);  //上下限
    //滤波
    fdata = Interp1DCore(InFilterData.pos3, DeviceList.x_pos3, DeviceList.y_pos, TableMax[MotorType], LIMIT_UEN);
    fdata = Limit(fdata, LengthMax[MotorType], 0);  //上下限
    
    OutData.pos3 = data;
    OutFilterData.pos3 = fdata;
    /* p4 */
    data = Interp1DCore(InData.pos4, DeviceList.x_pos4, DeviceList.y_pos, TableMax[MotorType], LIMIT_UEN);
    data = Limit(data, LengthMax[MotorType], 0);  //上下限
    //滤波
    fdata = Interp1DCore(InFilterData.pos4, DeviceList.x_pos4, DeviceList.y_pos, TableMax[MotorType], LIMIT_UEN);
    fdata = Limit(fdata, LengthMax[MotorType], 0);  //上下限
    
    OutData.pos4 = data;
    OutFilterData.pos4 = fdata;

    motor1.curpos = OutData.m_pos1;
    motor2.curpos = OutData.m_pos2;
    motor3.curpos = OutData.m_pos3;
    motor4.curpos = OutData.m_pos4;
}
/**
* @brief  设备运行
* @attention 
*/
void DevRun(void)
{
    DevVoltage();
    DevConversion();
    DevFilter();
}
