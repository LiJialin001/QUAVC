/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�LED.c
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
LED������ʹ�÷�ʽ���£�
g_LedManagerΪLED�ƿ��ƽṹ�壬Ҫ����LED����˸��ֻ��Ҫ���Ĵ˽ṹ���е�
ö��ֵ���ɡ���g_LedManager.emLed_Status
���磺
1.��Ҫ״̬��������Ҫ�������
g_LedManager.emLed_Status = StatusOn;

����ö����Դ����.h�ļ��е�emLED_Status_t

*/
//�ⲿ�ļ�����
#include "LED.h"
#include "control.h"

//�궨����
#define LED_FLASH_FREQ      100


//Extern����
extern FMUflg_t g_FMUflg;    


//˽�к�����



//˽�б�����
LedManager_t g_LedManager;


/******************************************************************************
  * �������ƣ�LEDInit
  * ������������ʼ��LED��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void LEDInit(void)        
{
    P2DIR |= GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7;
    P2OUT |= (GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    
    P2OUT &= ~GPIO_PIN7;
    
    LED_STATUS_OFF;
    LED_POWER_OFF;
    
    g_LedManager.emLEDStatus = StatusOff;
    g_LedManager.emLEDPower = PowerOff;    
}

/******************************************************************************
  * �������ƣ�PollingLED
  * ������������ѯ��ǰ�Ƿ���LED�����������
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *
  *
******************************************************************************/
void PollingLED()
{
    static bool blFlash = false;
    g_LedManager.u16FlashTime++;

    switch(g_LedManager.emLEDStatus)
    {
        case StatusFlash:
            if(g_LedManager.u16FlashTime % LED_FLASH_FREQ == 0)
            {
                LED_STATUS_TOGGLE;
            }
            break;
        case StatusToggle:
            LED_STATUS_TOGGLE;
            break;
        case StatusOn:
            LED_STATUS_ON;
            break;
        case StatusOff:
            LED_STATUS_OFF;
            break;
        default:
            break;
    }
    
    switch(g_LedManager.emLEDPower)
    {
        case PowerOn:
            LED_POWER_ON;
            break;
        case PowerFlash:
            if(g_LedManager.u16FlashTime % LED_FLASH_FREQ == 0)
            {
                LED_POWER_TOGGLE;
            }
            break;
        case PowerOff:
            LED_POWER_OFF
            break;
        case PowerToggle:
            LED_POWER_TOGGLE;
            break;
        default:
            break;
    }
    
        
    switch(g_LedManager.emLEDMotor)
    {
        case MotorFlash:
            if(g_LedManager.u16FlashTime % LED_FLASH_FREQ == 0)
            {
                blFlash = !blFlash;
            }
            
            if(blFlash)
            {
                M1_ON;
                M2_ON;
                M3_ON;
                M4_ON;
            }else
            {
                M1_OFF;
                M2_OFF;
                M3_OFF;
                M4_OFF;
            }
            break;
        case MotorClockwiseFlash:
            
            break;
        case MotorOn:
            M1_ON;
            M2_ON;
            M3_ON;
            M4_ON;
            break;
        case MotorOff:
            M1_OFF;
            M2_OFF;
            M3_OFF;
            M4_OFF;
            break;
        default:
            break;
    }
    
    if(!g_FMUflg.unlock)
    {
        g_LedManager.emLEDStatus = StatusOff;
        g_LedManager.emLEDMotor = MotorFlash;
    }else
    {
        g_LedManager.emLEDStatus = StatusOn;
        g_LedManager.emLEDMotor = MotorOn;
    }
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
