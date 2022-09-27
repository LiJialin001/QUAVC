/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：height_control.h
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
#ifndef __HEIGHT_CONTROL_H
#define __HEIGHT_CONTROL_H
//外部文件引用
#include "stdint.h"


//宏定义区



//数据结构声明
typedef struct
{
    float Z_Speed;
    float Z_Acc;
    float Z_Postion;

    float Alt;
    uint16_t Thr;
}HeightInfo_t;

typedef enum
{
    on_the_ground,
    take_off_ready,
    take_off,//起飞
    keep_altitude,//定点悬停
    touch_down,//降落
    stop,//停止旋转
}height_task_mode_e;


//Extern引用
extern HeightInfo_t HeightInfo;
extern height_task_mode_e  task_flag;

//函数声明
void ALT_Ctrl(float dT_s);
extern void mode_control(float dT_s);
#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
