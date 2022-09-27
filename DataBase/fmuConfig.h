/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：fmuConfig.h
  * 摘    要：
  *
  * 当前版本：V1.0
  * 作    者：北京中科浩电科技有限公司研发部 
  * 完成日期：    
  * 修改说明：
  * 
  *
  * 历史版本：
  *         固件号：100
  *         原始版本
  *
  *         固件号：101
  *         更新：
  *         1.支持修改发送接收地址；
  *         2.将收发信息改为USB接口
  *
  *         固件号：102
  *         更新：
  *         1.支持定高功能；
  *         2.支持大桨叶；
  *         3.更新LED显示；
  *         4.更新YAW轴控制；
  *         5.更新定高下放防误触功能；
  *         
  *******************************************************************************/
#ifndef __FMU_CONFIG_H
#define __FMU_CONFIG_H
//外部文件引用
#include "driverlib.h"


//宏定义区
//#define DEBUG

#define FIRMWARE_INFO          102

/*高度控制常量*/

#define MAX_ISFD_ATTITUDE      40
#define MIN_ALT_CM             50
#define MAX_ALT_CM             200
#define MAX_ALT_RATE           0.05
#define FIX_ALT_RATE           20
#define MAX_REMOTE_THROTTLE    2000
#define THROTTLE_DEAD_ZONE     0.2f

#define MOTOR_MAX_INIT_VALUE   900
#define NRF24L01_FREQ          20

//数据结构声明



//Extern引用



//函数声明

#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
