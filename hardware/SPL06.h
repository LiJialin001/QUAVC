/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�SPL06.h
  * ժ    Ҫ��
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
#ifndef SPL06_H
#define SPL06_H
//�ⲿ�ļ�����
#include "stdint.h"
#include "stdbool.h"

//�궨����
#define HW_ADR_L                    0x76//SDO LOW
#define HW_ADR                      HW_ADR_L<<1
#define CONTINUOUS_PRESSURE         1
#define CONTINUOUS_TEMPERATURE      2
#define CONTINUOUS_P_AND_T          3
#define PRESSURE_SENSOR             0
#define TEMPERATURE_SENSOR          1
#define OFFSET_COUNT                30

//���ݽṹ����
typedef struct
{    
    int16_t i16C0;
    int16_t i16C1;
    int32_t i32C00;
    int32_t i32C10;
    int16_t i16C01;
    int16_t i16C11;
    int16_t i16C20;
    int16_t i16C21;
    int16_t i16C30;       
}SPL06Param_t;

typedef struct
{    
    SPL06Param_t Param;
    uint8_t u8Chip_id;
    int32_t i32RawPressure;
    int32_t i32RawTemperature;
    int32_t i32KP;
    int32_t i32KT;
    
    float fGround_Alt;
    float fALT;                  //height above sea level        
    float fRelative_Alt;
    
    float fTemperature;
    float fPressure;
    float fLast_Pressure;
    
    float fOffset;
    bool Check;
}SPL06Manager_t;

//Extern����
extern SPL06Manager_t g_SPL06Manager;

//��������
extern void ResetAlt(void);
void  SPL06_Init(void);
float GetSPL06Temp(void);
float GetSPL06Press(void);
void  UpdateSPL06Info(void);

#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
