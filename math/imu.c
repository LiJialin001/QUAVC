//�ⲿ�ļ�����
#include "stdint.h"
#include "mpu6050.h"
#include "myMath.h"
#include <math.h>
#include "imu.h"
#include "i2c.h"
#include "driverlib.h"
#include "height_control.h"

//˽�б�����
//˽�б�����
Attitude_t g_Attitude;    //��ǰ�Ƕ���ֵ̬
static Quaternion_t NumQ = {1, 0, 0, 0};
float vecxZ,vecyZ,veczZ;
float wz_acc_tmp[2];

//��̬���㺯��
void ATT_Update(const MPU6050Manager_t *pMpu,Attitude_t *pAngE, float dt) 
{    
    Vector_t Gravity,Acc,Gyro,AccGravity;
    static Vector_t GyroIntegError = {0};
    static float KpDef = 0.8f ; //��Ԫ������ֵ
    static float KiDef = 0.0003f; 
    float q0_t,q1_t,q2_t,q3_t;
    float NormQuat; 
    float HalfTime = dt * 0.5f;

    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);                
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);              
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);  
    // ���ٶȹ�һ����
    NormQuat = Q_rsqrt(squa(g_MPUManager.accX)+ squa(g_MPUManager.accY) +squa(g_MPUManager.accZ));

    Acc.x = pMpu->accX * NormQuat; //��һ��ɻ�Ϊ��λ�����·������
    Acc.y = pMpu->accY * NormQuat;  
    Acc.z = pMpu->accZ * NormQuat;  

    //������˵ó���ֵ����˺���Եõ���ת����������������µļ��ٶȷ����ϵ�ƫ��
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
    //���ٶ��ںϼ��ٶȱ�������ֵ�����������乲ͬ�γ���PI�������õ�������Ľ��ٶ�ֵ
    Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//�����ƣ��˴��������ǽ��ٶȵ�Ư��
    Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
    Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;    
    // һ�����������, ������Ԫ��
    //������Ľ��ٶ�ֵ���֣��õ�������̬��������Ԫ��һ��ʵ��Q0�������鲿Q1~3��ֵ�ı仯
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = ( NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = ( NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t; //���ֺ��ֵ�ۼӵ��ϴε���Ԫ���У����µ���Ԫ��
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;
    
    // ������Ԫ����һ�����õ���λ������
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3)); //�õ���Ԫ����ģ��
    NumQ.q0 *= NormQuat; //ģ��������Ԫ��ֵ
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q2 *= NormQuat;
    
    float NormAccz = -pMpu->accX * vecxZ+ pMpu->accY * vecyZ + pMpu->accZ * veczZ;  /*Z�ᴹֱ�����ϵļ��ٶȣ���ֵ��������бʱ��Z����ٶȵ������ͣ����ǵ���������Ӧ�ó���ֵ*/        
    
    wz_acc_tmp[0] = (NormAccz - 8192) * 0.1196f;// cm/ss //0.1196f;����ÿ���η���
    wz_acc_tmp[1] += 0.1f *(wz_acc_tmp[0] - wz_acc_tmp[1]);//LPF
    HeightInfo.Z_Acc = wz_acc_tmp[1];    
}

//����Ƕ�
void GetAngle(Attitude_t *pAngE)
{
    vecxZ = 2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3 ;/*����(3,1)��*///��������ϵ�µ�X�����������
    vecyZ = 2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;/*����(3,2)��*///��������ϵ�µ�Y�����������
    veczZ = NumQ.q0 * NumQ.q0 - NumQ.q1 * NumQ.q1 - NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3;  /*����(3,3)��*///��������ϵ�µ�Z����������� 
    
    float g1, g2, g3, g4, g5;
    g1 = 2.0*(NumQ.q1*NumQ.q3-NumQ.q0*NumQ.q2);
    g2 = 2.0* (NumQ.q0*NumQ.q1+NumQ.q2*NumQ.q3);
    g3 = NumQ.q0*NumQ.q0-NumQ.q1*NumQ.q1-NumQ.q2*NumQ.q2+NumQ.q3*NumQ.q3;
    g4 = 2.0* (NumQ.q0*NumQ.q3+NumQ.q1*NumQ.q2);
    g5 = NumQ.q0*NumQ.q0+NumQ.q1*NumQ.q1-NumQ.q2*NumQ.q2- NumQ.q3*NumQ.q3;
    
    pAngE->pitch = -arcsin(g1)*RtA;                   //�����ϸ�
    pAngE->roll = arctan(g2/g3)*RtA;                  //�����Ҹ�
    pAngE->yaw = arctan(g4/g5)*RtA;//1����=57.3��     //������
}

void ResetAttitude()
{
    NumQ.q0 = 1;
    NumQ.q1 = 0;
    NumQ.q2 = 0;
    NumQ.q3 = 0;
}