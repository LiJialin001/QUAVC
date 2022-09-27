/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�control.c
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
���к����Զ����ã�����ѵϵͳ�е���
1.FlightPidControl����3msʱ�����һ��
2.MotorControl����3ms����һ��

*/
//�ⲿ�ļ�����
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

//�궨����
#define EMERGENT    0
#define MOTOR1      motor[0] 
#define MOTOR2      motor[1] 
#define MOTOR3      motor[2] 
#define MOTOR4      motor[3]
#define ClearMotor  memset(motor, 0, sizeof(int16_t) * 4)

//Extern����



//˽�к�����



//˽�б�����
int16_t motor[4];
FMUflg_t g_FMUflg;      //ϵͳ��־λ������������־λ��
float PIDGroup_desired_yaw_pos_tmp;
float ResetYawAngle = 0;
/******************************************************************************
  * �������ƣ�FlightPidControl
  * ��������������PID���ƺ���
  * ��    �룺float dt����λ����ʱ��
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null  
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
            ResetPID();                             //������λPID���ݣ���ֹ�ϴ�����������Ӱ�챾�ο���

            g_Attitude.yaw = 0;
            PIDGroup[emPID_Yaw_Pos].desired = 0;
            PIDGroup[emPID_Yaw_Pos].measured = 0;
            status = PROCESS_31;
            break;
        case PROCESS_31:                //��ʽ�������
*/
            
            PIDGroup[emPID_Pitch_Pos].desired = 0.0f;            
            PIDGroup[emPID_Roll_Pos].desired = 0.0f;
            
            PIDGroup[emPID_Roll_Spd].measured = g_MPUManager.gyroX * Gyro_G; //�ڻ�����ֵ �Ƕ�/��
            PIDGroup[emPID_Pitch_Spd].measured = g_MPUManager.gyroY * Gyro_G;
            PIDGroup[emPID_Yaw_Spd].measured = g_MPUManager.gyroZ * Gyro_G;

            PIDGroup[emPID_Pitch_Pos].measured = g_Attitude.pitch; //�⻷����ֵ ��λ���Ƕ�
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



            
            ClacCascadePID(&PIDGroup[emPID_Roll_Spd],  &PIDGroup[emPID_Roll_Pos],  dt);     //X��
            ClacCascadePID(&PIDGroup[emPID_Pitch_Spd], &PIDGroup[emPID_Pitch_Pos], dt);     //Y��
            ClacCascadePID(&PIDGroup[emPID_Yaw_Spd],   &PIDGroup[emPID_Yaw_Pos],   dt);     //Z��     
            
            
            
/*            break;
        case EXIT_255:                  //�˳�����
            ResetPID();

            status = WAITING_1;         //���صȴ�����
          break;
        default:
            status = EXIT_255;
            break;
    }
    
    if(g_FMUflg.unlock == EMERGENT)     //�����ƶ�
    {
        status = EXIT_255;
    }
*/
}

/******************************************************************************
  * �������ƣ�MotorControl
  * �������������µ�������߼�
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null    
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
        case WAITING_2:                               //������ɺ��ж�ʹ�����Ƿ�ʼ����ң�˽��з��п���
            if(Remote.thr > 1100 && !g_FMUflg.take_off) //�ս���ʱ�����������һ����ɲ���ң�˴�����ͣ���Ϊ�����߻��������
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
                
                //���ƫ�ƽ����ƶ�
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
                //������ֵ��Ϊ����ֵ��PWM
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
            status = WAITING_1;    //���صȴ�����
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

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
