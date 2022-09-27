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
  //关闭看门狗
  WDTCTL = WDTPW + WDTHOLD;
  //禁止中断
  _DINT();
  //系统时钟初始化
  System_Clock_Init();
  //IIC通信初始化
  I2C_Init();
  //电机初始化
  Motor_Init();
  //LED闪灯初始化
  LEDInit();                      
  //陀螺仪加速度计初始化
  MPU6050Init();
  //高度计初始化
  SPL06_Init();
  //PID初始化
  PID_Init();
  //定时器初始化
  Timer_Init();
  //使能中断
  _EINT();
  while(1) 
  { 
    //轮询函数
    PollingKernel();
  } 
}