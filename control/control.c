/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：control.c
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
飞行函数自动调用，在轮训系统中调用
1.FlightPidControl函数3ms时间调用一次
2.MotorControl函数3ms调用一次

*/
//外部文件引用
#include "driverlib.h"
#include "control.h"
#include "pid.h"
#include "math.h"
#include "led.h"
//#include "remote.h"
#include "spl06.h"
#include "imu.h"
#include "myMath.h"
#include "motor.h"
#include "speed_estimator.h"
#include "height_control.h"
#include "fmuConfig.h"

//宏定义区
#define EMERGENT    0
#define MOTOR1      motor[0] 
#define MOTOR2      motor[1] 
#define MOTOR3      motor[2] 
#define MOTOR4      motor[3]
#define ClearMotor  memset(motor, 0, sizeof(int16_t) * 4)

//Extern引用



//私有函数区



//私有变量区
int16_t motor[4];
FMUflg_t g_FMUflg;      //系统标志位，包含解锁标志位等
float PIDGroup_desired_yaw_pos_tmp;
float ResetYawAngle = 0;
/******************************************************************************
  * 函数名称：FlightPidControl
  * 函数描述：飞行PID控制函数
  * 输    入：float dt：单位运行时间
  * 输    出：void
  * 返    回：void 
  * 备    注：null  
  *
  *
******************************************************************************/
void FlightPidControl(float dt)
{
/*  
    volatile static uint8_t status = WAITING_1;

   
    switch(status)
    {
        case WAITING_1:
            if(g_FMUflg.unlock)
            {
                status = READY_11;    
            }
            break;
        case READY_11:
            ResetPID();                             //批量复位PID数据，防止上次遗留的数据影响本次控制

            g_Attitude.yaw = 0;
            PIDGroup[emPID_Yaw_Pos].desired = 0;
            PIDGroup[emPID_Yaw_Pos].measured = 0;
            status = PROCESS_31;
            break;
        case PROCESS_31:                //正式进入控制
*/
            
            PIDGroup[emPID_Pitch_Pos].desired = 0.0f;            
            PIDGroup[emPID_Roll_Pos].desired = 0.0f;
            
            PIDGroup[emPID_Roll_Spd].measured = g_MPUManager.gyroX * Gyro_G; //内环测量值 角度/秒
            PIDGroup[emPID_Pitch_Spd].measured = g_MPUManager.gyroY * Gyro_G;
            PIDGroup[emPID_Yaw_Spd].measured = g_MPUManager.gyroZ * Gyro_G;

            PIDGroup[emPID_Pitch_Pos].measured = g_Attitude.pitch; //外环测量值 单位：角度
            PIDGroup[emPID_Roll_Pos].measured = g_Attitude.roll;       
            
            PIDGroup[emPID_Yaw_Pos].measured = 0;
            PIDGroup[emPID_Yaw_Pos].desired = (PIDGroup_desired_yaw_pos_tmp - g_Attitude.yaw + ResetYawAngle);
            if(PIDGroup[emPID_Yaw_Pos].desired >= 180)
            {
                PIDGroup[emPID_Yaw_Pos].desired -= 360;
            }
            else if(PIDGroup[emPID_Yaw_Pos].desired <= -180)
            {
                PIDGroup[emPID_Yaw_Pos].desired += 360;
            }



            
            ClacCascadePID(&PIDGroup[emPID_Roll_Spd],  &PIDGroup[emPID_Roll_Pos],  dt);     //X轴
            ClacCascadePID(&PIDGroup[emPID_Pitch_Spd], &PIDGroup[emPID_Pitch_Pos], dt);     //Y轴
            ClacCascadePID(&PIDGroup[emPID_Yaw_Spd],   &PIDGroup[emPID_Yaw_Pos],   dt);     //Z轴     
            
            
            
/*            break;
        case EXIT_255:                  //退出控制
            ResetPID();

            status = WAITING_1;         //返回等待解锁
          break;
        default:
            status = EXIT_255;
            break;
    }
    
    if(g_FMUflg.unlock == EMERGENT)     //紧急制动
    {
        status = EXIT_255;
    }
*/
}

/******************************************************************************
  * 函数名称：MotorControl
  * 函数描述：更新电机控制逻辑
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void MotorControl(void)
{    
/*    volatile static uint8_t status = WAITING_1;

    if(g_FMUflg.unlock == EMERGENT)
    {
        status = EXIT_255;
    }
    
    switch(status)
    {
        case WAITING_1: 
            
            if(g_FMUflg.unlock)
            {
                g_FMUflg.take_off = 0;    
                g_FMUflg.height_lock = 0; 
                status = WAITING_2;
            }
        case WAITING_2:                               //解锁完成后判断使用者是否开始拨动遥杆进行飞行控制
            if(Remote.thr > 1100 && !g_FMUflg.take_off) //刚解锁时，如果不处于一键起飞并且遥杆处于最低，认为操作者还不想飞行
            {
                status = PROCESS_31;
            }
            else if(g_FMUflg.take_off)
            {
                g_FMUflg.height_lock = 1;
                status = PROCESS_31;                            
            }
            else
            {
                break;
            }    
        case PROCESS_31:
            {
                int16_t temp = 0;
                
                //添加偏移紧急制动
                if(g_Attitude.pitch < -MAX_ISFD_ATTITUDE 
                || g_Attitude.pitch > MAX_ISFD_ATTITUDE
                || g_Attitude.roll  < -MAX_ISFD_ATTITUDE
                || g_Attitude.roll  > MAX_ISFD_ATTITUDE)
                {
                    status = WAITING_1;
                    ResetAlt();
                    ResetPID();
                    ClearMotor;
                    break;
                }
                if(g_UAVinfo.UAV_Mode == Stabilize_Mode)
                {
                    temp = Remote.thr - 1000; 
                }

                if(g_UAVinfo.UAV_Mode == Altitude_Hold)
                {
                    temp = HeightInfo.Thr;
                }
*/              int16_t temp = 0;

                temp = HeightInfo.Thr;
                //temp = 100;
                //将油门值作为基础值给PWM
                MOTOR1 = LIMIT(temp, 0, MOTOR_MAX_INIT_VALUE); 
                MOTOR2 = LIMIT(temp, 0, MOTOR_MAX_INIT_VALUE); 
                MOTOR3 = LIMIT(temp, 0, MOTOR_MAX_INIT_VALUE); 
                MOTOR4 = LIMIT(temp, 0, MOTOR_MAX_INIT_VALUE); 
                
                MOTOR1 += +PIDGroup[emPID_Roll_Spd].out - PIDGroup[emPID_Pitch_Spd].out - PIDGroup[emPID_Yaw_Spd].out;
                MOTOR2 += +PIDGroup[emPID_Roll_Spd].out + PIDGroup[emPID_Pitch_Spd].out + PIDGroup[emPID_Yaw_Spd].out;
                MOTOR3 += -PIDGroup[emPID_Roll_Spd].out + PIDGroup[emPID_Pitch_Spd].out - PIDGroup[emPID_Yaw_Spd].out;
                MOTOR4 += -PIDGroup[emPID_Roll_Spd].out - PIDGroup[emPID_Pitch_Spd].out + PIDGroup[emPID_Yaw_Spd].out;
/*            }
            break;
        case EXIT_255:
            status = WAITING_1;    //返回等待解锁
            ClearMotor;
            break;
        default:
            break;
    }
*/  
    if(task_flag != on_the_ground && task_flag != stop)            
    {
      UpdateMotor(MOTOR4, MOTOR2, MOTOR1, MOTOR3);
    }
    else
    {
      UpdateMotor(0 ,0, 0, 0);    
    }
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
