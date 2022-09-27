/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�LED.h
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
#ifndef __LED_H
#define __LED_H
//�ⲿ�ļ�����
#include "driverlib.h"


//�궨����
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
    StatusToggle = 0,                   //״̬��ȡ��
    StatusOn,                       //״̬������
    StatusOff,                      //״̬��Ϩ��    
    StatusFlash,                    //״̬����˸
}emLEDStatus_t;

typedef enum
{
    PowerOn = 0,                        //��Դ������
    PowerOff,                       //��Դ��Ϩ��  
    PowerToggle,                    //��Դ��ȡ��
    PowerFlash,                     //��Դ����˸ 
}emLEDPower_t;

typedef enum
{
    MotorOn = 0,                        //���������
    MotorOff,                       //�����Ϩ�� 
    MotorFlash,                     //�������˸
    MotorClockwiseFlash,            //null
}emLEDMotor_t;

//���ݽṹ����
typedef struct
{
    uint16_t u16FlashTime;          //��˸��ʱ,1ms�ۼ�һ��
    
    emLEDMotor_t   emLEDMotor;
    emLEDPower_t   emLEDPower;
    emLEDStatus_t  emLEDStatus;
}LedManager_t;


//Extern����
extern LedManager_t g_LedManager;


//��������
void LEDInit(void);
void PollingLED(void);

#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
