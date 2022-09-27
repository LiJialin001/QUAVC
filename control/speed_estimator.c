/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：speesd_estimator.c
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

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
obs:观测值

*/
//外部文件引用
#include "speed_estimator.h"
#include "height_control.h"
#include "SPL06.h"
#include "control.h"
#include "myMath.h"


//宏定义区
#define VELOCITY_LIMIT        (130.f)    /*速度限幅 单位cm/s*/

//Extern引用


//私有函数区
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
void WZ_Obs_Calcu(float dT_s)//跟随OBS数据更新周期
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
    
    //光流回传的高度数据
    //obs_wz_height[1] = ANO_OF.ALT;
    
    //velocity ZKHD:计算气压计的速度和激光的速度
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
//  * 函数名称：WZ_Est_Calcu
//  * 函数描述：Z轴速度和位置初步预估
//  * 输    入：float dt：单位运行时间
//  * 输    出：void
//  * 返    回：void 
//  * 备    注：WZ_Est_Calcu  中 
//  *           W：world 世界坐标系
//  *           Z：Z轴方向
//  *           Est：Estimator预估
//  *           Calc:Cal
//******************************************************************************/
void WZ_Est_Calcu(float dT_s)//跟随惯性数据更新周期
{
    //根据原始加速度计信息积分后得到预估Z轴速度
    est_wz_velocity += HeightInfo.Z_Acc * dT_s;
    
    //根据预估到的Z轴速度信息积分后得到预估Z轴高度
    est_wz_height += est_wz_velocity * dT_s;
}

///******************************************************************************
//  * 函数名称：WZ_Fix_Calcu
//  * 函数描述：Z轴速度和位置初步预估
//  * 输    入：float dt：单位运行时间
//  * 输    出：void
//  * 返    回：void 
//  * 备    注：WZ_Est_Calcu  中 
//  *           W：world 世界坐标系
//  *           Z：Z轴方向
//  *           Est：Estimator预估
//  *           Calc:Cal
//******************************************************************************/
void WZ_Fix_Calcu(float dT_s)//跟随控制周期
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
/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
