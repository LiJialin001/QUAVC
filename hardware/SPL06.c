/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：SPL06.c
  * 摘    要：本文件用以驱动SPL06气压计，获取气压信息，并解算出高度
  *
  * 当前版本：V1.0
  * 作    者：北京中科浩电科技有限公司研发部
  *          北京寒水教育科技有限公司
  * 完成日期：    
  * 修改说明：
  *
  *
  * 历史版本：
  *
  *
  *******************************************************************************/

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
SPL06驱动可按如下方式使用：
1.调用 SPL06_Init() 函数，以初始化硬件设备；
2.固定频率调用 UpdateSPL06Info() 函数，以更新气压值和高度值；
*/
#ifndef SPL06_C
#define SPL06_C


//外部文件引用
#include "SPL06.h"
#include "i2c.h"
#include <math.h>
#include "delay.h"

//宏定义区
#define PRS_CFG                 0x06
#define TMP_CFG                 0x07
#define MEAS_CFG                0x08
#define SPL06_REST_VALUE        0x09
#define PRODUCT_ID              0X0D

//(1 / 5.25588f) Pressure factor
#define CONST_PF                0.1902630958    

// Fixed Temperature. ASL is a function of pressure and temperature,
// but as the temperature changes so much 
//(blow a little towards the flie and watch it drop 5 degrees)
// it corrupts the ASL estimates.
#define FIX_TEMP                25     
#define SPL06_Check             I2C_Read_Byte(HW_ADR, 0x0D)

//Extern引用


//私有函数区
void GetRawTemp(void);
void GetRawPressure(void);
void SetRate(uint8_t u8_Sensor, uint8_t u8_OverSmpl, uint8_t u8_SmplRate);
void SelectMode(uint8_t mode);
void CalcParam(void);

//私有变量区
SPL06Manager_t g_SPL06Manager;

/******************************************************************************
  * 函数名称：SPL06_Init
  * 函数描述：SPL06-01 初始化函数
  * 输    入：void
  * 输    出：vodd
  * 返    回：void
  * 备    注：null   
  *    
  *
******************************************************************************/
void SPL06_Init(void)
{   
    if(SPL06_Check == 0x10)
    {
        g_SPL06Manager.Check = true;
    }else
    {
        g_SPL06Manager.Check = false;
    }
    
    g_SPL06Manager.i32RawPressure = 0;
    g_SPL06Manager.i32RawTemperature = 0;
    g_SPL06Manager.u8Chip_id = 0x34;

    CalcParam();

    SetRate(PRESSURE_SENSOR, 128, 32);   
    SetRate(TEMPERATURE_SENSOR, 32, 8);
    SelectMode(CONTINUOUS_P_AND_T);
    delay_ms(100);
    
    UpdateSPL06Info();
    g_SPL06Manager.fGround_Alt = g_SPL06Manager.fALT;
}

/******************************************************************************
  * 函数名称:SetRate
  * 函数描述:设置温度传感器的每秒采样次数以及过采样率
  * 输    入:
  * uint8_t u8_OverSmpl:过采样率,最大值为128
  * uint8_t u8_SmplRate:每秒采样次数(Hz),最大值为128
  * uint8_t u8_Sensor  :传感器选择
  *                     0:气压计
  *                     1:温度计
  * 输    出:void
  * 返    回:void
  * 备    注:null
  *
  *
******************************************************************************/
void SetRate(uint8_t u8_Sensor, uint8_t u8_SmplRate, uint8_t u8_OverSmpl)
{
    uint8_t u8Reg = 0;
    int32_t i32KPkT = 0;
    switch(u8_SmplRate)
    {
        case 2:
            u8Reg |= (1 << 5);
            break;
        case 4:
            u8Reg |= (2 << 5);
            break;
        case 8:
            u8Reg |= (3 << 5);
            break;
        case 16:
            u8Reg |= (4 << 5);
            break;
        case 32:
            u8Reg |= (5 << 5);
            break;
        case 64:
            u8Reg |= (6 << 5);
            break;
        case 128:
            u8Reg |= (7 << 5);
            break;
        case 1:
        default:
            break;
    }
    
    switch(u8_OverSmpl)
    {
        case 2:
            u8Reg |= 1;
            i32KPkT = 1572864;
            break;
        case 4:
            u8Reg |= 2;
            i32KPkT = 3670016;
            break;
        case 8:
            u8Reg |= 3;
            i32KPkT = 7864320;
            break;
        case 16:
            i32KPkT = 253952;
            u8Reg |= 4;
            break;
        case 32:
            i32KPkT = 516096;
            u8Reg |= 5;
            break;
        case 64:
            i32KPkT = 1040384;
            u8Reg |= 6;
            break;
        case 128:
            i32KPkT = 2088960;
            u8Reg |= 7;
            break;
        case 1:
        default:
            i32KPkT = 524288;
            break;
    }

    if(u8_Sensor == 0)
    {
        g_SPL06Manager.i32KP = i32KPkT;
        I2C_Write_Byte(HW_ADR, 0x06, u8Reg);
        if(u8_OverSmpl > 8)
        {
            u8Reg = I2C_Read_Byte(HW_ADR, 0x09);
            I2C_Write_Byte(HW_ADR, 0x09, u8Reg | 0x04);
        }
    }
    
    if(u8_Sensor == 1)
    {
        g_SPL06Manager.i32KT = i32KPkT;
        
        //Using mems temperature
        I2C_Write_Byte(HW_ADR, 0x07, u8Reg|0x80);  
        
        if(u8_OverSmpl > 8)
        {
            u8Reg = I2C_Read_Byte(HW_ADR, 0x09);
            I2C_Write_Byte(HW_ADR, 0x09, u8Reg | 0x08);
        }
    }
}

/******************************************************************************
  * 函数名称：CalcParam
  * 函数描述：获取校准参数
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null   
  *    
  *
******************************************************************************/
void CalcParam(void)
{
    g_SPL06Manager.Param.i16C0 = 204;
    g_SPL06Manager.Param.i16C1 = -261;
    g_SPL06Manager.Param.i32C00 = 80469;
    g_SPL06Manager.Param.i32C10 = -54769;
    g_SPL06Manager.Param.i16C01 = -2803;
    g_SPL06Manager.Param.i16C11 = 1226;
    g_SPL06Manager.Param.i16C20 = -10787;
    g_SPL06Manager.Param.i16C21 = 183;
    g_SPL06Manager.Param.i16C30 = -1603; 
}

/******************************************************************************
  * 函数名称：SelectMode
  * 函数描述：Select node for the continuously measurement
  * 输    入：
  * uint8_t mode:模式选择
  *              1:气压模式;
  *              2:温度模式; 
  *              3:气压和温度模式;
  * 输    出：
  * 返    回：
  * 备    注：
  *    
  *
******************************************************************************/
void SelectMode(uint8_t mode)
{
    I2C_Write_Byte(HW_ADR, 0x08, mode + 4);
}

/******************************************************************************
  * 函数名称：GetRawTemp
  * 函数描述：获取温度的原始值，并转换成32Bits整数
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null    
  *    
  *
******************************************************************************/
void GetRawTemp(void)
{
    uint8_t u8Data[3] = {0};
    
    u8Data[0] = I2C_Read_Byte(HW_ADR, 0x03);
    u8Data[1] = I2C_Read_Byte(HW_ADR, 0x04);
    u8Data[2] = I2C_Read_Byte(HW_ADR, 0x05);

    g_SPL06Manager.i32RawTemperature = (int32_t)u8Data[0] << 16 | \
                                          (int32_t)u8Data[1] << 8  | \
                                          (int32_t)u8Data[2];
    
    g_SPL06Manager.i32RawTemperature = (g_SPL06Manager.i32RawTemperature & 0x800000)   ? \
                                          (0xFF000000 | g_SPL06Manager.i32RawTemperature) : \
                                          (g_SPL06Manager.i32RawTemperature);
}

/******************************************************************************
  * 函数名称：GetRawPressure
  * 函数描述：获取压力原始值，并转换成32bits整数
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null      
  *    
  *
******************************************************************************/
void GetRawPressure(void)
{
    uint8_t u8Data[3] = {0};
    
    u8Data[0] = I2C_Read_Byte(HW_ADR, 0x00);
    u8Data[1] = I2C_Read_Byte(HW_ADR, 0x01);
    u8Data[2] = I2C_Read_Byte(HW_ADR, 0x02);
    
    g_SPL06Manager.i32RawPressure = (int32_t)u8Data[0] << 16 | \
                                       (int32_t)u8Data[1] << 8  | \
                                       (int32_t)u8Data[2];
    g_SPL06Manager.i32RawPressure = (g_SPL06Manager.i32RawPressure & 0x800000)   ? \
                                       (0xFF000000 | g_SPL06Manager.i32RawPressure) : \
                                       (g_SPL06Manager.i32RawPressure);
}

/******************************************************************************
  * 函数名称：GetTemp
  * 函数描述：在获取原始值的基础上，返回浮点校准后的温度值
  * 输    入：void
  * 输    出：void
  * 返    回：float
  * 备    注：null   
  *    
  *
******************************************************************************/
float GetTemp(void)
{
    float fTCompensate = 0;
    float fTsc = 0;

    fTsc = g_SPL06Manager.i32RawTemperature / (float)g_SPL06Manager.i32KT;
    fTCompensate =  g_SPL06Manager.Param.i16C0 * 0.5 + \
                    g_SPL06Manager.Param.i16C1 * fTsc;
    
    return fTCompensate;
}

/******************************************************************************
 * 函数名称：GetTemp
 * 函数描述：在获取原始值的基础上，返回浮点校准后的温度值
 * 输    入：void
 * 输    出：void
 * 返    回：float
 * 备    注：null
 *
 *
 ******************************************************************************/
float GetSPL06Temp()
{
  return GetTemp();
}

/******************************************************************************
  * 函数名称：GetSPL06Press
  * 函数描述：在获取原始值的基础上，返回浮点校准后的压力值
  * 输    入：void
  * 输    出：void
  * 返    回：float
  * 备    注：null   
  *    
  *
******************************************************************************/
float GetSPL06Press(void)
{
    float fTsc = 0;
    float fPsc = 0;
    float fqua2 = 0;
    float fqua3 = 0;
    float fPCompensate = 0;

    fTsc = g_SPL06Manager.i32RawTemperature / (float)g_SPL06Manager.i32KT;
    fPsc = g_SPL06Manager.i32RawPressure / (float)g_SPL06Manager.i32KP;
    
    fqua2 = g_SPL06Manager.Param.i32C10 \
           + fPsc * (g_SPL06Manager.Param.i16C20 \
           + fPsc* g_SPL06Manager.Param.i16C30);
    fqua3 = fTsc * fPsc * (g_SPL06Manager.Param.i16C11 \
           + fPsc * g_SPL06Manager.Param.i16C21);
    
    fPCompensate = g_SPL06Manager.Param.i32C00 \
                   + fPsc * fqua2 + fTsc * g_SPL06Manager.Param.i16C01\
                   + fqua3;
    
    return fPCompensate;
}

/******************************************************************************
  * 函数名称：UpdateSPL06Info
  * 函数描述：更新气压计高度信息
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null     
  *    
  *
******************************************************************************/
void UpdateSPL06Info()
{
    GetRawTemp();
    GetRawPressure();
    
    g_SPL06Manager.fPressure = GetSPL06Press();

    /* tropospheric properties (0-11km) for standard atmosphere */
    /* temperature at base height in Kelvin, [K] = [°C] + 273.15 */
    const double T1 = 15.0 + 273.15;

    /* temperature gradient in degrees per metre */    
    const double a = -6.5 / 1000;    
    
    /* gravity constant in m / s/s */
    const double g = 9.80665;    
    
    /* ideal gas constant in J/kg/K */
    const double R = 287.05;    
    
    /* current pressure at MSL in kPa */
    double p1 = 101325.0 / 1000.0;

    /* measured pressure in kPa */
    double p = g_SPL06Manager.fPressure / 1000.0;

    //Altitude = (((exp((-(a * R) / g) * log((p / p1)))) * T1) - T1) / a;
    g_SPL06Manager.fALT = (((exp((-(a * R) / g) * log((p / p1)))) * T1) - T1) / a;
    g_SPL06Manager.fRelative_Alt = (int16_t)((int16_t)(g_SPL06Manager.fALT * 1000) - (int16_t)(g_SPL06Manager.fGround_Alt * 1000))/1000.0f;
}

void ResetAlt()
{
    g_SPL06Manager.fGround_Alt = g_SPL06Manager.fALT;
}

#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
