/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：pid.h
  * 摘    要：
  *
  * 当前版本：V1.0
  * 作    者：北京中科浩电科技有限公司研发部 
  * 完成日期：    
  * 修改说明：
  * 
  *
  * 历史版本：
  *
  *
  *******************************************************************************/
#ifndef __PID_H
#define __PID_H
//外部文件引用
#include <stdbool.h>
#include "stdint.h"

//宏定义区


//数据结构声明
typedef enum
{
    emPID_Pitch_Spd = 0,
    emPID_Roll_Spd,
    emPID_Yaw_Spd,
    emPID_Pitch_Pos,
    emPID_Roll_Pos,
    emPID_Yaw_Pos,
    emPID_Height_Spd,
    emPID_Height_Pos,
    emPID_AUX1,
    emPID_AUX2,
    emPID_AUX3,
    emPID_AUX4,
    emPID_AUX5,
    
    emNum_Of_PID_List,
}emPID_list_t;

typedef struct
{
    float kp;           //< proportional gain
    float ki;           //< integral gain
    float kd;           //< derivative gain
    float out;
    float Err;
    float desired;     //< set point
    float measured;

    float Err_LimitHigh;
    float Err_LimitLow;
    
    float offset;      //
    float prevError;    //< previous error
    float integ;        //< integral

    float IntegLimitHigh;       //< integral limit
    float IntegLimitLow;

    float OutLimitHigh;
    float OutLimitLow;
}PIDInfo_t;

//Extern引用
extern PIDInfo_t PIDGroup[emNum_Of_PID_List];

//函数声明
void ResetPID(void);
void ClacCascadePID(PIDInfo_t* pidRate, PIDInfo_t* pidAngE, const float dt);  //串级PID
void UpdatePID(PIDInfo_t* pid, const float dt);  //PID
extern void PID_Init(void);
#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
