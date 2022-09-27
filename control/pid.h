/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�pid.h
  * ժ    Ҫ��
  *
  * ��ǰ�汾��V1.0
  * ��    �ߣ������пƺƵ�Ƽ����޹�˾�з��� 
  * ������ڣ�    
  * �޸�˵����
  * 
  *
  * ��ʷ�汾��
  *
  *
  *******************************************************************************/
#ifndef __PID_H
#define __PID_H
//�ⲿ�ļ�����
#include <stdbool.h>
#include "stdint.h"

//�궨����


//���ݽṹ����
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

//Extern����
extern PIDInfo_t PIDGroup[emNum_Of_PID_List];

//��������
void ResetPID(void);
void ClacCascadePID(PIDInfo_t* pidRate, PIDInfo_t* pidAngE, const float dt);  //����PID
void UpdatePID(PIDInfo_t* pid, const float dt);  //PID
extern void PID_Init(void);
#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
