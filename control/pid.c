/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�pid.c
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
PID����ʹ�÷������£�
1.����һ��PIDInfo_t�ṹ�壬������Ҫ���Ƶ����ݴ�Ž�ȥ��
2.��UpdatePID����������PID������
3.����ֱ�ӵ���ClacCascadePIDֱ�Ӽ��㴮��PID


*/
//�ⲿ�ļ�����
#include "pid.h"
#include "myMath.h"    
#include "imu.h"
//�궨����



//Extern����
extern float ResetYawAngle;
extern Attitude_t g_Attitude;    //��ǰ�Ƕ���ֵ̬
extern float PIDGroup_desired_yaw_pos_tmp;
//˽�к�����



//˽�б�����
/*PID���̱���*/
PIDInfo_t PIDGroup[emNum_Of_PID_List];


/******************************************************************************
  * �������ƣ�ResetPID
  * ������������λPID
  * ��    �룺PID�ṹ��ָ��
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
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
  * �������ƣ�UpdatePID
  * ��������������PID���ֵ
  * ��    �룺PIDInfo_t* pid��Ҫ�����PID�ṹ��ָ��
              float dt����λ����ʱ��
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void UpdatePID(PIDInfo_t* pid, const float dt)
{
    float deriv;
    
    pid->Err = pid->desired - pid->measured + pid->offset; //��ǰ�Ƕ���ʵ�ʽǶȵ����

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
    
    pid->out = pid->kp * pid->Err + pid->ki * pid->integ + pid->kd * deriv;//PID���
    
    if(pid->OutLimitHigh != 0 && pid->OutLimitLow != 0)
    {
        pid->out = LIMIT(pid->out, pid->OutLimitLow, pid->OutLimitHigh);
    }
    
    pid->prevError = pid->measured;//pid->Err;  ΢�����У���ʽ���÷�
}

/******************************************************************************
  * �������ƣ�ClacCascadePID
  * �������������㴮��PID
  * ��    �룺PIDInfo_t* pidRate��PID�ٶȻ�
              PIDInfo_t* pidAngE��PID�ǶȻ�
              const float dt����λ����ʱ��
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null    
  *    
  *
******************************************************************************/
void ClacCascadePID(PIDInfo_t* pidRate, PIDInfo_t* pidAngE, const float dt)  //����PID
{     
    UpdatePID(pidAngE, dt);    //�ȼ����⻷
    pidRate->desired = pidAngE->out;
    UpdatePID(pidRate, dt);   
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/


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

    //��ʼ��UAV�����Ϣ
//    g_UAVinfo.UAV_Mode = Altitude_Hold;
}