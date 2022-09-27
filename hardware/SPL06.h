/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：SPL06.h
  * 摘    要：
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
#ifndef SPL06_H
#define SPL06_H
//外部文件引用
#include "stdint.h"
#include "stdbool.h"

//宏定义区
#define HW_ADR_L                    0x76//SDO LOW
#define HW_ADR                      HW_ADR_L<<1
#define CONTINUOUS_PRESSURE         1
#define CONTINUOUS_TEMPERATURE      2
#define CONTINUOUS_P_AND_T          3
#define PRESSURE_SENSOR             0
#define TEMPERATURE_SENSOR          1
#define OFFSET_COUNT                30

//数据结构声明
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

//Extern引用
extern SPL06Manager_t g_SPL06Manager;

//函数声明
extern void ResetAlt(void);
void  SPL06_Init(void);
float GetSPL06Temp(void);
float GetSPL06Press(void);
void  UpdateSPL06Info(void);

#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
