/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：motor.c
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
电机驱动调用方式如下：
1.调用函数UpdateMotor直接驱动电机；
电机PWM初始化已经在main函数中完成。


*/
//外部文件引用
#include "motor.h"
#include "myMath.h"

//宏定义区


//Extern引用


//私有函数区


//私有变量区


/******************************************************************************
  * 函数名称：UpdateMotor
  * 函数描述：驱动电机转动，电机输出取值0-300
  * 输    入：
  * int16_t M1:电机1输出取值
  * int16_t M2:电机2输出取值
  * int16_t M3:电机3输出取值
  * int16_t M4:电机4输出取值
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *    
  *
******************************************************************************/
void UpdateMotor(int16_t M1, int16_t M2, int16_t M3, int16_t M4)
{
    M1 = LIMIT(M1, 0, 999);   //电机取值限幅 0-999
    M2 = LIMIT(M2, 0, 999); 
    M3 = LIMIT(M3, 0, 999);
    M4 = LIMIT(M4, 0, 999);
    
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, M1);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, M2);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, M3);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, M4);
}

/******************************************************************************
  * 函数名称：Motor_Init
  * 函数描述：初始化电机
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null
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

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
