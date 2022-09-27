/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�fmuConfig.h
  * ժ    Ҫ��
  *
  * ��ǰ�汾��V1.0
  * ��    �ߣ������пƺƵ�Ƽ����޹�˾�з��� 
  * ������ڣ�    
  * �޸�˵����
  * 
  *
  * ��ʷ�汾��
  *         �̼��ţ�100
  *         ԭʼ�汾
  *
  *         �̼��ţ�101
  *         ���£�
  *         1.֧���޸ķ��ͽ��յ�ַ��
  *         2.���շ���Ϣ��ΪUSB�ӿ�
  *
  *         �̼��ţ�102
  *         ���£�
  *         1.֧�ֶ��߹��ܣ�
  *         2.֧�ִ�Ҷ��
  *         3.����LED��ʾ��
  *         4.����YAW����ƣ�
  *         5.���¶����·ŷ��󴥹��ܣ�
  *         
  *******************************************************************************/
#ifndef __FMU_CONFIG_H
#define __FMU_CONFIG_H
//�ⲿ�ļ�����
#include "driverlib.h"


//�궨����
//#define DEBUG

#define FIRMWARE_INFO          102

/*�߶ȿ��Ƴ���*/

#define MAX_ISFD_ATTITUDE      40
#define MIN_ALT_CM             50
#define MAX_ALT_CM             200
#define MAX_ALT_RATE           0.05
#define FIX_ALT_RATE           20
#define MAX_REMOTE_THROTTLE    2000
#define THROTTLE_DEAD_ZONE     0.2f

#define MOTOR_MAX_INIT_VALUE   900
#define NRF24L01_FREQ          20

//���ݽṹ����



//Extern����



//��������

#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
