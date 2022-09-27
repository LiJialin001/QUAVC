/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�motor.c
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
����������÷�ʽ���£�
1.���ú���UpdateMotorֱ�����������
���PWM��ʼ���Ѿ���main��������ɡ�


*/
//�ⲿ�ļ�����
#include "motor.h"
#include "myMath.h"

//�궨����


//Extern����


//˽�к�����


//˽�б�����


/******************************************************************************
  * �������ƣ�UpdateMotor
  * �����������������ת����������ȡֵ0-300
  * ��    �룺
  * int16_t M1:���1���ȡֵ
  * int16_t M2:���2���ȡֵ
  * int16_t M3:���3���ȡֵ
  * int16_t M4:���4���ȡֵ
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void UpdateMotor(int16_t M1, int16_t M2, int16_t M3, int16_t M4)
{
    M1 = LIMIT(M1, 0, 999);   //���ȡֵ�޷� 0-999
    M2 = LIMIT(M2, 0, 999); 
    M3 = LIMIT(M3, 0, 999);
    M4 = LIMIT(M4, 0, 999);
    
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, M1);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, M2);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, M3);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, M4);
}

/******************************************************************************
  * �������ƣ�Motor_Init
  * ������������ʼ�����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void Motor_Init()
{
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1,GPIO_PIN2);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1,GPIO_PIN3);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1,GPIO_PIN4);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1,GPIO_PIN5);
    
    Timer_A_outputPWMParam param = {0};
    param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod = 1000;
    param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle = 0;
    
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    Timer_A_outputPWM(TIMER_A0_BASE, &param);
    
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;
    Timer_A_outputPWM(TIMER_A0_BASE, &param);
    
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    Timer_A_outputPWM(TIMER_A0_BASE, &param);
    
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    Timer_A_outputPWM(TIMER_A0_BASE, &param);
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
