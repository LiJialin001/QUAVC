/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：myMath.h
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

#ifndef __MY_MATH_H
#define    __MY_MATH_H

//外部文件引用
#include "driverlib.h"


//宏定义区
#define PI 3.1415926f
#define squa( Sq )        (((float)Sq)*((float)Sq))
#define absu16( Math_X )  ((Math_X)<0? -(Math_X):(Math_X))
#define absFloat( Math_X )((Math_X)<0? -(Math_X):(Math_X))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define ABS(x) ((x) > 0 ? (x) : -(x))
#define LIMIT( x, min, max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )


//数据结构声明


//Extern引用
extern const float M_PI;
extern const float AtR;
extern const float RtA;
extern const float Gyro_G;
extern const float Gyro_Gr;


//函数声明
extern float safe_asin(float v);
extern float arcsin(float x);
extern float arctan(float x);
extern float sine(float x);
extern float cosine(float x);
extern float Q_rsqrt(float number);
extern float VariableParameter(float error);
#endif /* __Algorithm_math_H */

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
