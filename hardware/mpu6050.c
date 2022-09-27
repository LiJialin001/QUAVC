//外部文件引用
#include "stdint.h"
#include "stdbool.h"
#include "mpu6050.h"
#include <string.h>
#include "i2c.h"
#include "driverlib.h"
#include "delay.h"

//宏定义区
#define SMPLRT_DIV          0x19    //陀螺仪采样率，典型值：0x07(125Hz)
#define CONFIGL             0x1A    //低通滤波频率，典型值：0x06(5Hz)
#define GYRO_CONFIG         0x1B    //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define ACCEL_CONFIG        0x1C    //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define ACCEL_ADDRESS       0x3B
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_ADDRESS        0x43
#define GYRO_XOUT_L         0x44    
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define PWR_MGMT_1          0x6B    //电源管理，典型值：0x00(正常启用)
#define WHO_AM_I            0x75    //I2C地址寄存器(默认数值0x68，只读)
#define MPU6050_PRODUCT_ID  0x68
#define MPU6052C_PRODUCT_ID 0x72
#define MPU6050_ADDRESS     0xD0    //0x68

#define Acc_Read()          I2C_Read_Bytes(MPU6050_ADDRESS, 0X3B, buffer, 6) //读取加速度
#define Gyro_Read()         I2C_Read_Bytes(MPU6050_ADDRESS, 0x43, &buffer[6], 6)  //  读取角速度
#define MPU6050_RawRead()     I2C_Read_Bytes(MPU6050_ADDRESS, 0X3B, buffer, 14) //读取加速度

//Extern引用
extern uint8_t I2C_Read_Bytes(uint8_t Slaveaddr, uint8_t REG_Address, uint8_t *ptr, uint8_t length);

//私有变量区
MPU6050Manager_t g_MPUManager;   //g_MPUManager原始数据
int16_t *pMpu = (int16_t *)&g_MPUManager;

void GetMPU6050Offset(void);

//初始化函数
void MPU6050Init(void)
{   
  
    uint8_t check = 0;
    g_MPUManager.Check = false;
    
    /*
    //手动校准          
     I2C_Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1,    0x80);   //复位
    __delay_cycles(100);
    I2C_Write_Byte(MPU6050_ADDRESS, SMPLRT_DIV,   0x02);   //陀螺仪采样率，0x00(333Hz)
    I2C_Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1,   0x03);   //设置设备时钟源，陀螺仪Z轴
    I2C_Write_Byte(MPU6050_ADDRESS, CONFIGL,      0x03);   //低通滤波频率，0x03(42Hz)
    I2C_Write_Byte(MPU6050_ADDRESS, GYRO_CONFIG,  0x18);   //+-2000deg/s
    I2C_Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x08);   //+-4G   
    */
    delay_ms(200);
    I2C_Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1,    0x80);   //复位
    delay_ms(200);
    I2C_Write_Byte(MPU6050_ADDRESS, SMPLRT_DIV,   0x02);   //陀螺仪采样率，0x00(333Hz)
    I2C_Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1,   0x03);   //设置设备时钟源，陀螺仪Z轴
    delay_ms(200);
    I2C_Write_Byte(MPU6050_ADDRESS, CONFIGL,      0x03);   //低通滤波频率，0x03(42Hz)
    I2C_Write_Byte(MPU6050_ADDRESS, GYRO_CONFIG,  0x18);   //+-2000deg/s
    I2C_Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x08);   //+-4G
    
    check = I2C_Read_Byte(MPU6050_ADDRESS, 0x75);  //判断g_MPUManager地址

    if(check != MPU6050_PRODUCT_ID) //如果地址不正确
    {
        g_MPUManager.Check = false;
    }
    else
    {
        GetMPU6050Offset();
        g_MPUManager.Check = true;
    }
}

//读取陀螺仪数据
void MPU_6050_read(void)
{
  
    uint8_t buffer[14];   
    static float mpu_filter[3][6];
    int16_t mpu_filter_tmp[6];
    
    Acc_Read();          
    Gyro_Read();
    
    for(int i = 0; i < 6; i++)
    {
        //依次获取buffer中的传感器数据
        mpu_filter_tmp[i] = (((int16_t)buffer[i << 1] << 8) | buffer[(i << 1) + 1]) - g_MPUManager.Offset[i];
        
        //二阶LPF
        mpu_filter[0][i] += 0.25f * (mpu_filter_tmp[i] - mpu_filter[0][i]);
        mpu_filter[1][i] += 0.25f * (mpu_filter[0][i]  - mpu_filter[1][i]);
        
        pMpu[i] = (int16_t)mpu_filter[1][i];
    }  
         
}

void GetMPU6050Offset(void) //校准
{
    int32_t buffer[6] = {0};
    int16_t i = 0;  
    uint8_t k = 30;
    const int8_t MAX_GYRO_QUIET = 50;
    const int8_t MIN_GYRO_QUIET = -50;    
    
    int16_t LastGyro[3] = {0};  /*wait for calm down*/
    int16_t ErrorGyro[3] = {0};        /*set offset initial to zero*/
    
    memset(g_MPUManager.Offset, 0, 12);
    g_MPUManager.Offset[2] = 8192;   //根据手册量程设定加速度标定值 

    while(k--)  //判断飞控是否处于静止状态
    {
        do
        {
            delay_ms(10);
            MPU_6050_read();
            
            for(i = 0; i < 3; i++)
            {
                ErrorGyro[i] = pMpu[i + 3] - LastGyro[i];
                LastGyro[i] = pMpu[i + 3];    
            }
        }while ((ErrorGyro[0] > MAX_GYRO_QUIET) 
             || (ErrorGyro[0] < MIN_GYRO_QUIET)
             || (ErrorGyro[1] > MAX_GYRO_QUIET) 
             || (ErrorGyro[1] < MIN_GYRO_QUIET)
             || (ErrorGyro[2] > MAX_GYRO_QUIET)
             || (ErrorGyro[2] < MIN_GYRO_QUIET));
    }

    for(i = 0; i < 356; i++)  //取第100到第356组的平均值做为校准值
    {        
        MPU_6050_read();
        
        if(100 <= i)
        {
            for(int k = 0; k < 6; k++)
            {
                buffer[k] += pMpu[k];
            }
        }
    }

    for(i = 0; i < 6; i++)  //保存校准值
    {
        g_MPUManager.Offset[i] = buffer[i] >> 8;
    }
    
    g_MPUManager.Offset[4] += 1;
    g_MPUManager.Offset[5] += 1;
}

