/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�height_control.c
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
�߶ȿ�������


*/
//�ⲿ�ļ�����
#include "height_control.h"
#include "speed_estimator.h"
#include "control.h"
#include "SPL06.h"
#include "imu.h"
#include "mpu6050.h"
#include "myMath.h"
#include "math.h"
//#include "Remote.h"
#include "pid.h"

//�궨����
#define IN_RANGE(value, min, max)        ( (value)>(min) && (value)<(max) )
#define THROTTLE_BASE 600

//Extern����
extern SPL06Manager_t g_SPL06Manager;


//˽�к�����

//˽�б�����
HeightInfo_t HeightInfo;
float dt2 = 0;
bool Acc_Enable_Flag = false;


#define MAX_EXP_WZ_VEL_UP 200
#define MAX_EXP_WZ_VEL_DW 150
#define MAX_EXP_WZ_ACC    500

//wz_ctrl struct
static float exp_vel_transition[4];
static float exp_vel_d;

static float exp_acc;
static float fb_acc;
static float exp_vel;
static float fb_vel;
float exp_hei;
static float fb_hei;

//pid struct
static float hei_err,vel_err,acc_err,acc_err_i,acc_out,wz_out;

#define H_KP 100.0f
#define V_KP 5.0f
#define V_KD 0.05f
#define A_KP 0.4f
#define A_KI 0.6f


//fc����˼�Ƿɿ�
//extern uint8_t fc_state
//uint8_t fc_state_take_off = 0;
height_task_mode_e  task_flag = on_the_ground;
uint32_t on_the_ground_count;
uint32_t take_off_ready_count;
uint32_t take_off_count;
uint32_t keep_altitude_count;
uint32_t touch_down_count;



void mode_control(float dT_s)
{
  
    //״̬������
    if(task_flag == on_the_ground)
    {
        on_the_ground_count ++; //ÿ��0.02s    
    }

    if(task_flag == take_off_ready)
    {
        take_off_ready_count ++; //ÿ��0.02s    
    }     
    
    if(task_flag == keep_altitude)  
    {  
        keep_altitude_count ++; //ÿ��0.02s
    }
    
    
    //ģʽ�л�
    if(task_flag == on_the_ground && on_the_ground_count>= 100) //2s
    {
        task_flag = take_off_ready;
        HeightInfo.Thr = 200;
    }
    else if(task_flag == take_off_ready && take_off_ready_count>= 100) //2s
    {
        task_flag = take_off;
        est_wz_height = 0.0f;
        ResetAlt();        
    }
    else if(task_flag == take_off && fb_hei >= 0.8f) //3s
    {
        task_flag = keep_altitude;
    }
    else if(task_flag == keep_altitude && keep_altitude_count > 250) //5s
    {
       task_flag = touch_down;
    }    
    else if(task_flag == touch_down && fb_hei <= 0.15f)   //3s
    {
       task_flag = stop;
    }
    
    
    //���ݷ���ģʽ�ж��ٶ�=�趨ֵ
    if(task_flag == take_off)
    {
        exp_vel_transition[0] = 0.3f;
        exp_hei = 1.0f;
    }
    else if(task_flag == keep_altitude)
    {
        exp_vel_transition[0] = 0;
        exp_hei = 1.0f;
    }
    else if(task_flag == touch_down)
    {
        exp_vel_transition[0] = -0.3f; 
        exp_hei = 0.0f;
    }
    else
    {
        exp_vel_transition[0] = 0;
    }
    
    
}

void ALT_Ctrl(float dT_s)
{
    //==input calculate
    //fb = feedback ���·�����Ϣ
    fb_vel = HeightInfo.Z_Speed;
    fb_hei = HeightInfo.Z_Postion;
    fb_acc = HeightInfo.Z_Acc;
            
    //����������Χ������������ٶ�
    if(exp_vel_transition[0] > 0)
    {
        exp_vel_transition[1] = exp_vel_transition[0] * MAX_EXP_WZ_VEL_UP;
    }
    else
    {
        exp_vel_transition[1] = exp_vel_transition[0] * MAX_EXP_WZ_VEL_DW;
    }
    
    //�������������ٶ�����
    float tmp = MAX_EXP_WZ_ACC * dT_s;
    
    //���������ٶ�����
    exp_vel_d = (exp_vel_transition[1] - exp_vel_transition[2]);
    
    //�����ٶ������޷�
    if(exp_vel_d > tmp)
    {
        exp_vel_d = tmp;
    }
    else if(exp_vel_d < -tmp)
    {
        exp_vel_d = -tmp;
    }
    
    //�����ٶ�Ϊ�����ٶ��������൱�ڼ��ٶȻ���
    exp_vel_transition[2] += exp_vel_d;
    
    //�����ٶ�LPF
    exp_vel_transition[3] += 0.2f *(exp_vel_transition[2] - exp_vel_transition[3]);
    
    //==exp_val state
    //
//    if(g_UAVinfo.UAV_Mode >= Altitude_Hold && fc_state_take_off != 0 )
//    {
        //���յ������ٶ�Ϊexp_vel
        exp_vel = exp_vel_transition[3];
        
        //�����߶�Ϊ�����ٶȵĻ���
        //exp_hei += exp_vel * dT_s;
        
        //�����߶��޷�
        //if(exp_hei > fb_hei+150)
        //{
        //    exp_hei = fb_hei+150;
        //}
        //else if(exp_hei < fb_hei-150)
        //{
        //    exp_hei = fb_hei-150;
        //}
//    }
//    else
//   {
//        exp_vel = 0;
//        exp_hei = fb_hei;
 
//    }
    
    //==ctrl
    //�߶���� = �����߶���� - �������
    hei_err = (exp_hei - fb_hei);
    
    
    //�ٶ���� = (Kp * �߶���� + �����ٶ�) - (�����ٶ� + �������ٶ� * kd)
    //Ϊ�δ˴��ٶ���Ϊ��vel_err = exp_vel - fb_vel;
    //PD���ƣ����������������ٶ�
    vel_err = ((H_KP * hei_err + exp_vel) - (fb_vel + V_KD *fb_acc));
    
    //�������ٶ� = Kp * �ٶ�����P���ƣ������������������ٶ�
    exp_acc = (V_KP * vel_err);
    
    //���ٶ���� = �������ٶ� - �������ٶ�
    //�Լ��ٶȽ���PI����
    acc_err = exp_acc - fb_acc;
    acc_err_i += A_KI * acc_err * dT_s;
    acc_err_i = (acc_err_i > 600)?600:((acc_err_i<0)?0:acc_err_i);
    
    //�������Ϊ Kp * ���ٶ�����ֵ + ���ٶȻ���ֵ��PI���ƣ�
    acc_out = A_KP * exp_acc;
    wz_out = acc_out + acc_err_i;
    
    //����޷�
    wz_out = (wz_out > 900)?900:((wz_out < 0)?0:wz_out);
    
    if(task_flag == touch_down && HeightInfo.Thr < 450)
    {
        HeightInfo.Thr = 450;
    }
    HeightInfo.Thr = (uint16_t)wz_out;

/*            
    //unlock state
    //����ɻ�δ�������򽫼��ٶȵĻ��������㣬�߶�����ֵ�뷴��ֵ��ͬ
    if(g_FMUflg.unlock == 0)
    {
        acc_err_i = 0;
        exp_hei = fb_hei;
        fc_state_take_off = 0;
         for(int i = 0;i<4;i++)
        {
          exp_vel_transition[i] = 0;
        }
    }
    else
    {
        if(g_UAVinfo.UAV_Mode >= Altitude_Hold)
        {
            //������λ��״̬�л�Ϊ���
            if(exp_vel_transition[0]>0)
            {
                fc_state_take_off = 1;
            }
        }
        else//g_UAVinfo.UAV_Mode < Altitude_Hold
        {
            fc_state_take_off = 1;
        }
    }
*/
}


/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
