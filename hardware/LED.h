/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：LED.h
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
#ifndef __LED_H
#define __LED_H
//外部文件引用
#include "driverlib.h"


//宏定义区
#define M1_ON                       P2OUT &= ~GPIO_PIN4;
#define M1_OFF                      P2OUT |= GPIO_PIN4;
#define M1_Toggle                   GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN4);

#define M2_ON                       P2OUT &= ~GPIO_PIN5;
#define M2_OFF                      P2OUT |= GPIO_PIN5;
#define M2_Toggle                   GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN5);

#define M3_ON                       P2OUT &= ~GPIO_PIN6;
#define M3_OFF                      P2OUT |= GPIO_PIN6;
#define M3_Toggle                   GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN6);

#define M4_ON                       P2OUT &= ~GPIO_PIN7;
#define M4_OFF                      P2OUT |= GPIO_PIN7;
#define M4_Toggle                   GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN7);

#define LED_STATUS_OFF              P2OUT |= GPIO_PIN0;
#define LED_STATUS_ON               P2OUT &= ~GPIO_PIN0;
#define LED_STATUS_TOGGLE           GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN0);

#define LED_POWER_OFF               P2OUT |= GPIO_PIN1;
#define LED_POWER_ON                P2OUT &= ~GPIO_PIN1;
#define LED_POWER_TOGGLE            GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);

typedef enum
{
    StatusToggle = 0,                   //状态灯取反
    StatusOn,                       //状态灯亮起
    StatusOff,                      //状态灯熄灭    
    StatusFlash,                    //状态灯闪烁
}emLEDStatus_t;

typedef enum
{
    PowerOn = 0,                        //电源灯亮起
    PowerOff,                       //电源灯熄灭  
    PowerToggle,                    //电源灯取反
    PowerFlash,                     //电源灯闪烁 
}emLEDPower_t;

typedef enum
{
    MotorOn = 0,                        //电机灯亮起
    MotorOff,                       //电机灯熄灭 
    MotorFlash,                     //电机灯闪烁
    MotorClockwiseFlash,            //null
}emLEDMotor_t;

//数据结构声明
typedef struct
{
    uint16_t u16FlashTime;          //闪烁计时,1ms累加一次
    
    emLEDMotor_t   emLEDMotor;
    emLEDPower_t   emLEDPower;
    emLEDStatus_t  emLEDStatus;
}LedManager_t;


//Extern引用
extern LedManager_t g_LedManager;


//函数声明
void LEDInit(void);
void PollingLED(void);

#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
