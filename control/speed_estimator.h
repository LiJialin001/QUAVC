/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�speed_estimator.h
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
#ifndef __STATE_ESTIMATOR_H
#define __STATE_ESTIMATOR_H
//�ⲿ�ļ�����
#include "driverlib.h"

//
typedef enum
{
    FIX_RESET = 0,
    FIX_WORKING,
}_wz_fus_sta_enum;
extern _wz_fus_sta_enum wz_fus_sta;

//��������
void UpdateAltSpeed(float dt);
void ResetAltSpeed(void);         

//======ANOTC=====
void WZ_Obs_Calcu(float dT_s);
void WZ_Est_Calcu(float dT_s);
void WZ_Fix_Calcu(float dT_s);
void WZ_Fus_Reset(void);

extern float est_wz_height;
#endif 

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
