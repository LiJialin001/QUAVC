/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�SPL06.c
  * ժ    Ҫ�����ļ���������SPL06��ѹ�ƣ���ȡ��ѹ��Ϣ����������߶�
  *
  * ��ǰ�汾��V1.0
  * ��    �ߣ������пƺƵ�Ƽ����޹�˾�з���
  *          ������ˮ�����Ƽ����޹�˾
  * ������ڣ�    
  * �޸�˵����
  *
  *
  * ��ʷ�汾��
  *
  *
  *******************************************************************************/

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
SPL06�����ɰ����·�ʽʹ�ã�
1.���� SPL06_Init() �������Գ�ʼ��Ӳ���豸��
2.�̶�Ƶ�ʵ��� UpdateSPL06Info() �������Ը�����ѹֵ�͸߶�ֵ��
*/
#ifndef SPL06_C
#define SPL06_C


//�ⲿ�ļ�����
#include "SPL06.h"
#include "i2c.h"
#include <math.h>
#include "delay.h"

//�궨����
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

//Extern����


//˽�к�����
void GetRawTemp(void);
void GetRawPressure(void);
void SetRate(uint8_t u8_Sensor, uint8_t u8_OverSmpl, uint8_t u8_SmplRate);
void SelectMode(uint8_t mode);
void CalcParam(void);

//˽�б�����
SPL06Manager_t g_SPL06Manager;

/******************************************************************************
  * �������ƣ�SPL06_Init
  * ����������SPL06-01 ��ʼ������
  * ��    �룺void
  * ��    ����vodd
  * ��    �أ�void
  * ��    ע��null   
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
  * ��������:SetRate
  * ��������:�����¶ȴ�������ÿ����������Լ���������
  * ��    ��:
  * uint8_t u8_OverSmpl:��������,���ֵΪ128
  * uint8_t u8_SmplRate:ÿ���������(Hz),���ֵΪ128
  * uint8_t u8_Sensor  :������ѡ��
  *                     0:��ѹ��
  *                     1:�¶ȼ�
  * ��    ��:void
  * ��    ��:void
  * ��    ע:null
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
  * �������ƣ�CalcParam
  * ������������ȡУ׼����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null   
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
  * �������ƣ�SelectMode
  * ����������Select node for the continuously measurement
  * ��    �룺
  * uint8_t mode:ģʽѡ��
  *              1:��ѹģʽ;
  *              2:�¶�ģʽ; 
  *              3:��ѹ���¶�ģʽ;
  * ��    ����
  * ��    �أ�
  * ��    ע��
  *    
  *
******************************************************************************/
void SelectMode(uint8_t mode)
{
    I2C_Write_Byte(HW_ADR, 0x08, mode + 4);
}

/******************************************************************************
  * �������ƣ�GetRawTemp
  * ������������ȡ�¶ȵ�ԭʼֵ����ת����32Bits����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null    
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
  * �������ƣ�GetRawPressure
  * ������������ȡѹ��ԭʼֵ����ת����32bits����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null      
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
  * �������ƣ�GetTemp
  * �����������ڻ�ȡԭʼֵ�Ļ����ϣ����ظ���У׼����¶�ֵ
  * ��    �룺void
  * ��    ����void
  * ��    �أ�float
  * ��    ע��null   
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
 * �������ƣ�GetTemp
 * �����������ڻ�ȡԭʼֵ�Ļ����ϣ����ظ���У׼����¶�ֵ
 * ��    �룺void
 * ��    ����void
 * ��    �أ�float
 * ��    ע��null
 *
 *
 ******************************************************************************/
float GetSPL06Temp()
{
  return GetTemp();
}

/******************************************************************************
  * �������ƣ�GetSPL06Press
  * �����������ڻ�ȡԭʼֵ�Ļ����ϣ����ظ���У׼���ѹ��ֵ
  * ��    �룺void
  * ��    ����void
  * ��    �أ�float
  * ��    ע��null   
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
  * �������ƣ�UpdateSPL06Info
  * ����������������ѹ�Ƹ߶���Ϣ
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null     
  *    
  *
******************************************************************************/
void UpdateSPL06Info()
{
    GetRawTemp();
    GetRawPressure();
    
    g_SPL06Manager.fPressure = GetSPL06Press();

    /* tropospheric properties (0-11km) for standard atmosphere */
    /* temperature at base height in Kelvin, [K] = [��C] + 273.15 */
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

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
