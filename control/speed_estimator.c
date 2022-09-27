/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�speesd_estimator.c
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

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
obs:�۲�ֵ

*/
//�ⲿ�ļ�����
#include "speed_estimator.h"
#include "height_control.h"
#include "SPL06.h"
#include "control.h"
#include "myMath.h"


//�궨����
#define VELOCITY_LIMIT        (130.f)    /*�ٶ��޷� ��λcm/s*/

//Extern����


//˽�к�����
float applyDeadbandf(float value, float deadband);

//===================ANOTC===================
//#include "Ano_OF.h"

/*
#define FIX_A1 0.8f
#define FIX_B1 0.5f
#define FIX_C1    0.2f
*/


#define FIX_A1 15.0f
#define FIX_B1 0.5f
#define FIX_C1 0.2f


#define FIX_A2 1.6f
#define FIX_B2 0.5f
#define FIX_C2 0.2f

//wz_fus struct
float fix_rat[3];
_wz_fus_sta_enum wz_fus_sta;

float obs_wz_velocity[2];
float obs_wz_height[2];

float obs_wz_hasl;
float obs_wz_hasl_gnd;

float obs_wz_velocity_ref;
float obs_wz_height_ref;
    
float est_wz_velocity;
float est_wz_height;

float fix_wz_velocity;
float fix_wz_height;
float fix_wz_acceleration;

/*
void WZ_Obs_Calcu(float dT_s)//����OBS���ݸ�������
{
    //height[1]
    
    obs_wz_hasl = (g_SPL06Manager.fRelative_Alt);//cm
    
    //
    if(height_init_f!=0)
    {
        //
        height_init_f --;
        //
        obs_wz_hasl_gnd = obs_wz_hasl;
    }
    else
    {
        obs_wz_height[0] = obs_wz_hasl - obs_wz_hasl_gnd;
    }
    
    //�����ش��ĸ߶�����
    //obs_wz_height[1] = ANO_OF.ALT;
    
    //velocity ZKHD:������ѹ�Ƶ��ٶȺͼ�����ٶ�
    obs_wz_velocity[0] = ((obs_wz_height[0] - height_old[0])/dT_s);
    obs_wz_velocity[1] = ((obs_wz_height[1] - height_old[1])/dT_s);
    
    //
    height_old[0] = obs_wz_height[0];
    height_old[1] = obs_wz_height[1];
    
    obs_wz_velocity_ref = obs_wz_velocity[0];
    //
    fix_rat[0] = FIX_A1;
    fix_rat[1] = FIX_B1;
    fix_rat[2] = FIX_C1;
    
}
*/

///******************************************************************************
//  * �������ƣ�WZ_Est_Calcu
//  * ����������Z���ٶȺ�λ�ó���Ԥ��
//  * ��    �룺float dt����λ����ʱ��
//  * ��    ����void
//  * ��    �أ�void 
//  * ��    ע��WZ_Est_Calcu  �� 
//  *           W��world ��������ϵ
//  *           Z��Z�᷽��
//  *           Est��EstimatorԤ��
//  *           Calc:Cal
//******************************************************************************/
void WZ_Est_Calcu(float dT_s)//����������ݸ�������
{
    //����ԭʼ���ٶȼ���Ϣ���ֺ�õ�Ԥ��Z���ٶ�
    est_wz_velocity += HeightInfo.Z_Acc * dT_s;
    
    //����Ԥ������Z���ٶ���Ϣ���ֺ�õ�Ԥ��Z��߶�
    est_wz_height += est_wz_velocity * dT_s;
}

///******************************************************************************
//  * �������ƣ�WZ_Fix_Calcu
//  * ����������Z���ٶȺ�λ�ó���Ԥ��
//  * ��    �룺float dt����λ����ʱ��
//  * ��    ����void
//  * ��    �أ�void 
//  * ��    ע��WZ_Est_Calcu  �� 
//  *           W��world ��������ϵ
//  *           Z��Z�᷽��
//  *           Est��EstimatorԤ��
//  *           Calc:Cal
//******************************************************************************/
void WZ_Fix_Calcu(float dT_s)//�����������
{
  static float tmp = 0;
    //==calcu
    //
    fix_rat[0] = FIX_A1;
    fix_rat[1] = FIX_B1;
    fix_rat[2] = FIX_C1;
    
    obs_wz_height_ref = g_SPL06Manager.fRelative_Alt;
    obs_wz_velocity_ref = -(g_SPL06Manager.fRelative_Alt - tmp)/dT_s;
    obs_wz_velocity_ref = (obs_wz_velocity_ref > 500) ? 500 : ((obs_wz_velocity_ref < -500 ) ? -500 : obs_wz_velocity_ref);
    tmp = g_SPL06Manager.fRelative_Alt;
    
    fix_wz_height = fix_rat[0] *((float)obs_wz_height_ref - est_wz_height);
    
    fix_wz_velocity = fix_rat[1] *((float)obs_wz_velocity_ref - est_wz_velocity);
    //
    fix_wz_acceleration += fix_rat[2] *(fix_wz_velocity) * dT_s;
    fix_wz_acceleration = (fix_wz_acceleration > 50) ? 50 : ((fix_wz_acceleration < -50 ) ? -50 : fix_wz_acceleration);
    //
    //==fix
    //
    est_wz_height += fix_wz_height * dT_s;    
    //
    est_wz_velocity += (fix_wz_acceleration + fix_wz_velocity ) *dT_s;
    //==
    HeightInfo.Z_Speed  = est_wz_velocity;
    HeightInfo.Z_Postion = est_wz_height;
    
    //
    if(wz_fus_sta == FIX_RESET || g_FMUflg.unlock == 0)
    {
        WZ_Fus_Reset();
        //
        wz_fus_sta = FIX_WORKING;
    }
}

void WZ_Fus_Reset()
{
    obs_wz_height_ref =    obs_wz_velocity_ref = 0;
    est_wz_height = 0;
}
/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
