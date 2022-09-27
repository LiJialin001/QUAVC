/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：height_control.c
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
高度控制驱动


*/
//外部文件引用
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

//宏定义区
#define IN_RANGE(value, min, max)        ( (value)>(min) && (value)<(max) )
#define THROTTLE_BASE 600

//Extern引用
extern SPL06Manager_t g_SPL06Manager;


//私有函数区

//私有变量区
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


//fc的意思是飞控
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
  
    //状态机计数
    if(task_flag == on_the_ground)
    {
        on_the_ground_count ++; //每次0.02s    
    }

    if(task_flag == take_off_ready)
    {
        take_off_ready_count ++; //每次0.02s    
    }     
    
    if(task_flag == keep_altitude)  
    {  
        keep_altitude_count ++; //每次0.02s
    }
    
    
    //模式切换
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
    
    
    //根据飞行模式判断速度=设定值
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
    //fb = feedback 更新反馈信息
    fb_vel = HeightInfo.Z_Speed;
    fb_hei = HeightInfo.Z_Postion;
    fb_acc = HeightInfo.Z_Acc;
            
    //超过死区范围，则计算期望速度
    if(exp_vel_transition[0] > 0)
    {
        exp_vel_transition[1] = exp_vel_transition[0] * MAX_EXP_WZ_VEL_UP;
    }
    else
    {
        exp_vel_transition[1] = exp_vel_transition[0] * MAX_EXP_WZ_VEL_DW;
    }
    
    //计算最大的期望速度增量
    float tmp = MAX_EXP_WZ_ACC * dT_s;
    
    //计算期望速度增量
    exp_vel_d = (exp_vel_transition[1] - exp_vel_transition[2]);
    
    //期望速度增量限幅
    if(exp_vel_d > tmp)
    {
        exp_vel_d = tmp;
    }
    else if(exp_vel_d < -tmp)
    {
        exp_vel_d = -tmp;
    }
    
    //期望速度为叠加速度增量，相当于加速度积分
    exp_vel_transition[2] += exp_vel_d;
    
    //期望速度LPF
    exp_vel_transition[3] += 0.2f *(exp_vel_transition[2] - exp_vel_transition[3]);
    
    //==exp_val state
    //
//    if(g_UAVinfo.UAV_Mode >= Altitude_Hold && fc_state_take_off != 0 )
//    {
        //最终的期望速度为exp_vel
        exp_vel = exp_vel_transition[3];
        
        //期望高度为期望速度的积分
        //exp_hei += exp_vel * dT_s;
        
        //期望高度限幅
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
    //高度误差 = 期望高度误差 - 反馈误差
    hei_err = (exp_hei - fb_hei);
    
    
    //速度误差 = (Kp * 高度误差 + 期望速度) - (反馈速度 + 反馈加速度 * kd)
    //为何此处速度误差不为：vel_err = exp_vel - fb_vel;
    //PD控制，快速收敛到期望速度
    vel_err = ((H_KP * hei_err + exp_vel) - (fb_vel + V_KD *fb_acc));
    
    //期望加速度 = Kp * 速度误差，纯P控制，快速收敛到期望加速度
    exp_acc = (V_KP * vel_err);
    
    //加速度误差 = 期望加速度 - 反馈加速度
    //对加速度进行PI控制
    acc_err = exp_acc - fb_acc;
    acc_err_i += A_KI * acc_err * dT_s;
    acc_err_i = (acc_err_i > 600)?600:((acc_err_i<0)?0:acc_err_i);
    
    //最终输出为 Kp * 加速度期望值 + 加速度积分值（PI控制）
    acc_out = A_KP * exp_acc;
    wz_out = acc_out + acc_err_i;
    
    //输出限幅
    wz_out = (wz_out > 900)?900:((wz_out < 0)?0:wz_out);
    
    if(task_flag == touch_down && HeightInfo.Thr < 450)
    {
        HeightInfo.Thr = 450;
    }
    HeightInfo.Thr = (uint16_t)wz_out;

/*            
    //unlock state
    //如果飞机未解锁，则将加速度的积分量置零，高度期望值与反馈值相同
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
            //超过中位，状态切换为起飞
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


/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
