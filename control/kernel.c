/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�kernel.c
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
�������к��������ļ�������ѵ��Ҫ���ܺ���

*/
//�ⲿ�ļ�����
#include "driverlib.h"
#include "kernel.h"
#include "delay.h"
#include "mpu6050.h"
#include "imu.h"
#include "SPL06.h"
#include "motor.h"
#include "speed_estimator.h"
#include "control.h"
#include "height_control.h"

//�궨����
#define RATE_166HZ (166)
#define RATE_333HZ (333)

//Extern������

//˽�к�����
void Update(void);
void UpdateUSBQueue(void);

//˽�б�����
bool Thread_3MS;
bool Thread_8MS;
bool Thread_10MS;
bool Thread_20MS;

/******************************************************************************
  * �������ƣ�TIMER0_B0_ISR
  * ����������TIMER0 B0 �жϷ�����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��1ms����һ��
  *    
  *
******************************************************************************/
// Timer B0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(TIMER0_B0_VECTOR))) TIMER0_B0_ISR(void)
#else
#error Compiler not supported!
#endif
{
    static uint32_t Cnt = 0;

    Cnt++;
    if (Cnt % 3 == 0) //3ms
    {
        Thread_3MS = true;
    }

    if (Cnt % 8 == 0)
    {
        Thread_8MS = true;
    }

    if (Cnt % 20 == 0)
    {
        Thread_20MS = true;
    }
    
    if (Cnt % 10 == 0)
    {
        Thread_10MS = true;
    }
    
    //PollingUSART();
    //PollingLED();
    Timer_B_clearTimerInterrupt(TIMER_B0_BASE);
}

/******************************************************************************
  * �������ƣ�PollingKernel
  * ���������������к���
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע���˺���������B0�жϷ���������
  *    
  *
******************************************************************************/
void PollingKernel()
{

    if (Thread_3MS)
    {
        Thread_3MS = false;
        
        MPU_6050_read();    
        WZ_Est_Calcu(0.003f);
        ATT_Update(&g_MPUManager,&g_Attitude, 0.003f); 
        FlightPidControl(0.003f); 
        MotorControl();
    }

    if (Thread_8MS)
    {
        Thread_8MS = false;

        GetAngle(&g_Attitude);
    }
    
//    if (Thread_10MS)
//    {
//        Thread_10MS = false;
          
          //PollingNRF(); 
//    }
   if (Thread_20MS)
   {
       Thread_20MS = false;
      
       mode_control(0.02f);
       
       //��ѹ�����ݲɼ�OK 
       UpdateSPL06Info();

//        //�����ں�
        WZ_Fix_Calcu(0.02f);

//        //�߶ȿ���
        ALT_Ctrl(0.02f);
       
   }
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/

