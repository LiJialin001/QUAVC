#include "driverlib.h"
#include "mpu6050.h"
#include "stdint.h"
#include "delay.h"
#include "i2c.h"
#include "imu.h"
#include "board.h"
#include "timer.h"
#include "kernel.h"
#include "SPL06.h"
#include "motor.h"
#include "LED.h"
#include "pid.h"



extern void I2C_Init(void);

int main(void)
{
  //�رտ��Ź�
  WDTCTL = WDTPW + WDTHOLD;
  //��ֹ�ж�
  _DINT();
  //ϵͳʱ�ӳ�ʼ��
  System_Clock_Init();
  //IICͨ�ų�ʼ��
  I2C_Init();
  //�����ʼ��
  Motor_Init();
  //LED���Ƴ�ʼ��
  LEDInit();                      
  //�����Ǽ��ٶȼƳ�ʼ��
  MPU6050Init();
  //�߶ȼƳ�ʼ��
  SPL06_Init();
  //PID��ʼ��
  PID_Init();
  //��ʱ����ʼ��
  Timer_Init();
  //ʹ���ж�
  _EINT();
  while(1) 
  { 
    //��ѯ����
    PollingKernel();
  } 
}