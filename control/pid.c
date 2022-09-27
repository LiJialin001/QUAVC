/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：pid.c
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
PID驱动使用方法如下：
1.构建一个PIDInfo_t结构体，将所需要控制的数据存放进去；
2.调UpdatePID函数，计算PID输出结果
3.可以直接调用ClacCascadePID直接计算串级PID


*/
//外部文件引用
#include "pid.h"
#include "myMath.h"    
#include "imu.h"
//宏定义区



//Extern引用
extern float ResetYawAngle;
extern Attitude_t g_Attitude;    //当前角度姿态值
extern float PIDGroup_desired_yaw_pos_tmp;
//私有函数区



//私有变量区
/*PID工程变量*/
PIDInfo_t PIDGroup[emNum_Of_PID_List];


/******************************************************************************
  * 函数名称：ResetPID
  * 函数描述：复位PID
  * 输    入：PID结构体指针
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *    
  *
******************************************************************************/
void ResetPID(void)
{
    for(int i = 0; i < emNum_Of_PID_List; i++)
    {
        PIDGroup[i].integ = 0;
        PIDGroup[i].prevError = 0;
        PIDGroup[i].out = 0;
        PIDGroup[i].offset = 0;
        PIDGroup[i].Err = 0;
        PIDGroup[i].desired = 0;     //< set point
        PIDGroup[i].measured = 0;
        PIDGroup[i].out = 0;
    }
    
    ResetYawAngle = g_Attitude.yaw;
    PIDGroup_desired_yaw_pos_tmp= 0;
    ResetAttitude();
}

/******************************************************************************
  * 函数名称：UpdatePID
  * 函数描述：计算PID相关值
  * 输    入：PIDInfo_t* pid：要计算的PID结构体指针
              float dt：单位运行时间
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *    
  *
******************************************************************************/
void UpdatePID(PIDInfo_t* pid, const float dt)
{
    float deriv;
    
    pid->Err = pid->desired - pid->measured + pid->offset; //当前角度与实际角度的误差

    if(pid->Err_LimitHigh != 0 && pid->Err_LimitLow != 0)
    {
        pid->Err = LIMIT(pid->Err, pid->Err_LimitLow, pid->Err_LimitHigh);
    }
    
    pid->integ += pid->Err * dt;    
    
    if(pid->IntegLimitHigh != 0 && pid->IntegLimitLow != 0)
    {
        pid->integ = LIMIT(pid->integ, pid->IntegLimitLow, pid->IntegLimitHigh);
    }
    
    //deriv = (pid->Err - pid->prevError)/dt;
    deriv = -(pid->measured - pid->prevError)/dt;
    
    pid->out = pid->kp * pid->Err + pid->ki * pid->integ + pid->kd * deriv;//PID输出
    
    if(pid->OutLimitHigh != 0 && pid->OutLimitLow != 0)
    {
        pid->out = LIMIT(pid->out, pid->OutLimitLow, pid->OutLimitHigh);
    }
    
    pid->prevError = pid->measured;//pid->Err;  微分先行（变式）用法
}

/******************************************************************************
  * 函数名称：ClacCascadePID
  * 函数描述：计算串级PID
  * 输    入：PIDInfo_t* pidRate：PID速度环
              PIDInfo_t* pidAngE：PID角度环
              const float dt：单位运行时间
  * 输    出：void
  * 返    回：void
  * 备    注：null    
  *    
  *
******************************************************************************/
void ClacCascadePID(PIDInfo_t* pidRate, PIDInfo_t* pidAngE, const float dt)  //串级PID
{     
    UpdatePID(pidAngE, dt);    //先计算外环
    pidRate->desired = pidAngE->out;
    UpdatePID(pidRate, dt);   
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/


void PID_Init(void)
{
    PIDGroup[emPID_Pitch_Pos].kp   = 15.0f;
    PIDGroup[emPID_Pitch_Pos].ki   = 1.5f;
    PIDGroup[emPID_Pitch_Pos].kd   = 0.7f;
    PIDGroup[emPID_Pitch_Pos].IntegLimitHigh = 75;
    PIDGroup[emPID_Pitch_Pos].IntegLimitLow = -75;
    PIDGroup[emPID_Pitch_Pos].OutLimitHigh = 200;
    PIDGroup[emPID_Pitch_Pos].OutLimitLow = -200;
    
    
    PIDGroup[emPID_Pitch_Spd].kp   = 1.8f;
    PIDGroup[emPID_Pitch_Spd].ki   = 1.0f;
    PIDGroup[emPID_Pitch_Spd].kd   = 0.12f;
    PIDGroup[emPID_Pitch_Spd].IntegLimitHigh = 50;
    PIDGroup[emPID_Pitch_Spd].IntegLimitLow = -50;
    PIDGroup[emPID_Pitch_Spd].OutLimitHigh = 300;
    PIDGroup[emPID_Pitch_Spd].OutLimitLow = -300;
    
    
    PIDGroup[emPID_Roll_Pos].kp    = 15.0f;
    PIDGroup[emPID_Roll_Pos].ki    = 1.5f;
    PIDGroup[emPID_Roll_Pos].kd    = 0.7f;
    PIDGroup[emPID_Roll_Pos].IntegLimitHigh = 75;
    PIDGroup[emPID_Roll_Pos].IntegLimitLow = -75;
    PIDGroup[emPID_Roll_Pos].OutLimitHigh = 200;
    PIDGroup[emPID_Roll_Pos].OutLimitLow = -200;
    
    PIDGroup[emPID_Roll_Spd].kp    = 1.8f;
    PIDGroup[emPID_Roll_Spd].ki    = 1.0f;
    PIDGroup[emPID_Roll_Spd].kd    = 0.12f;
    PIDGroup[emPID_Roll_Spd].IntegLimitHigh = 50;
    PIDGroup[emPID_Roll_Spd].IntegLimitLow = -50;
    PIDGroup[emPID_Roll_Spd].OutLimitHigh = 300;
    PIDGroup[emPID_Roll_Spd].OutLimitLow = -300;
    
    
    
    PIDGroup[emPID_Yaw_Pos].kp     = 8.0f;
    
    PIDGroup[emPID_Yaw_Spd].kp     = 3.0f;
    PIDGroup[emPID_Yaw_Spd].kd     = 0.00f;
    PIDGroup[emPID_Yaw_Spd].OutLimitHigh = 200;
    PIDGroup[emPID_Yaw_Spd].OutLimitLow = -200;


//====
    //PIDGroup[emPID_Roll_Spd].IntegLimitHigh = 50;
    //PIDGroup[emPID_Roll_Spd].IntegLimitLow = -50;

    //初始化UAV相关信息
//    g_UAVinfo.UAV_Mode = Altitude_Hold;
}