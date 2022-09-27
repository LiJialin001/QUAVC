//外部文件引用
#include "stdint.h"
#include "mpu6050.h"
#include "myMath.h"
#include <math.h>
#include "imu.h"
#include "i2c.h"
#include "driverlib.h"
#include "height_control.h"

//私有变量区
//私有变量区
Attitude_t g_Attitude;    //当前角度姿态值
static Quaternion_t NumQ = {1, 0, 0, 0};
float vecxZ,vecyZ,veczZ;
float wz_acc_tmp[2];

//姿态解算函数
void ATT_Update(const MPU6050Manager_t *pMpu,Attitude_t *pAngE, float dt) 
{    
    Vector_t Gravity,Acc,Gyro,AccGravity;
    static Vector_t GyroIntegError = {0};
    static float KpDef = 0.8f ; //四元数收勉值
    static float KiDef = 0.0003f; 
    float q0_t,q1_t,q2_t,q3_t;
    float NormQuat; 
    float HalfTime = dt * 0.5f;

    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);                
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);              
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);  
    // 加速度归一化，
    NormQuat = Q_rsqrt(squa(g_MPUManager.accX)+ squa(g_MPUManager.accY) +squa(g_MPUManager.accZ));

    Acc.x = pMpu->accX * NormQuat; //归一后可化为单位向量下方向分量
    Acc.y = pMpu->accY * NormQuat;  
    Acc.z = pMpu->accZ * NormQuat;  

    //向量叉乘得出的值，叉乘后可以得到旋转矩阵的重力分量在新的加速度分量上的偏差
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
    //角速度融合加速度比例补偿值，与上面三句共同形成了PI补偿，得到矫正后的角速度值
    Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//弧度制，此处补偿的是角速度的漂移
    Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
    Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;    
    // 一阶龙格库塔法, 更新四元数
    //矫正后的角速度值积分，得到两次姿态解算中四元数一个实部Q0，三个虚部Q1~3的值的变化
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = ( NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = ( NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t; //积分后的值累加到上次的四元数中，即新的四元数
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;
    
    // 重新四元数归一化，得到单位向量下
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3)); //得到四元数的模长
    NumQ.q0 *= NormQuat; //模长更新四元数值
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q2 *= NormQuat;
    
    float NormAccz = -pMpu->accX * vecxZ+ pMpu->accY * vecyZ + pMpu->accZ * veczZ;  /*Z轴垂直方向上的加速度，此值涵盖了倾斜时在Z轴角速度的向量和，不是单纯重力感应得出的值*/        
    
    wz_acc_tmp[0] = (NormAccz - 8192) * 0.1196f;// cm/ss //0.1196f;厘米每二次方秒
    wz_acc_tmp[1] += 0.1f *(wz_acc_tmp[0] - wz_acc_tmp[1]);//LPF
    HeightInfo.Z_Acc = wz_acc_tmp[1];    
}

//计算角度
void GetAngle(Attitude_t *pAngE)
{
    vecxZ = 2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3 ;/*矩阵(3,1)项*///地理坐标系下的X轴的重力分量
    vecyZ = 2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;/*矩阵(3,2)项*///地理坐标系下的Y轴的重力分量
    veczZ = NumQ.q0 * NumQ.q0 - NumQ.q1 * NumQ.q1 - NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3;  /*矩阵(3,3)项*///地理坐标系下的Z轴的重力分量 
    
    float g1, g2, g3, g4, g5;
    g1 = 2.0*(NumQ.q1*NumQ.q3-NumQ.q0*NumQ.q2);
    g2 = 2.0* (NumQ.q0*NumQ.q1+NumQ.q2*NumQ.q3);
    g3 = NumQ.q0*NumQ.q0-NumQ.q1*NumQ.q1-NumQ.q2*NumQ.q2+NumQ.q3*NumQ.q3;
    g4 = 2.0* (NumQ.q0*NumQ.q3+NumQ.q1*NumQ.q2);
    g5 = NumQ.q0*NumQ.q0+NumQ.q1*NumQ.q1-NumQ.q2*NumQ.q2- NumQ.q3*NumQ.q3;
    
    pAngE->pitch = -arcsin(g1)*RtA;                   //下正上负
    pAngE->roll = arctan(g2/g3)*RtA;                  //左正右负
    pAngE->yaw = arctan(g4/g5)*RtA;//1弧度=57.3度     //右正左负
}

void ResetAttitude()
{
    NumQ.q0 = 1;
    NumQ.q1 = 0;
    NumQ.q2 = 0;
    NumQ.q3 = 0;
}