#ifndef __IMU_H
#define __IMU_H

//�ⲿ�ļ�����
#include "stdint.h"
#include "I2C.h"
#include "mpu6050.h"

typedef struct{
    float roll;
    float pitch;
    float yaw;
}Attitude_t;

typedef struct {  //��Ԫ��
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion_t;

typedef struct 
{
  float x;
  float y;
  float z;
}Vector_t; 

extern Attitude_t g_Attitude;
extern void ATT_Update(const MPU6050Manager_t *pMpu,Attitude_t *pAngE, float dt);
extern void GetAngle(Attitude_t *pAngE);
extern void ResetAttitude();

#endif 