//
// Created by Ray on 2023/11/3.
//

#ifndef INC_2024RC_R1_UPPER_H
#define INC_2024RC_R1_UPPER_H

#include "main.h"
#include "stdio.h"
#include <math.h>
#include "rm_motor.h"
#include "pid.h"
#include "usr_can.h"

extern uint8_t RxData[8]; // ���ݽ������飬can������ֻ֡��8֡
extern uint8_t TxData[8];

extern MOTOR_REAL_INFO motorRealInfo[7];
extern PID_T MOTOR_PID_RPM[7]; //�ٶ�pid��Ϣ
extern PID_T MOTOR_PID_POS[7];	//λ��pid��Ϣ

typedef struct Upper_Motor
{
    MOTOR_REAL_INFO *motorinfo;
    float init_pos;     //��ʼλ�ã�����Сֵ
    float final_pos;    //����λ�ã������ֵ
    float pos;
}upper_motor;

#define Motor_BELT_MOTOR_1       0
#define Motor_BELT_MOTOR_2       1
#define Motor_BELT_MOTOR_3       2
#define Motor_CLAW               3
#define Motor_UPLIFT             4

/**
 * @brief ��צ��е��ת����ֵ��
 * û�Թ���ȫ����0��
 * ��ֵ��ҪϹ�������裬һ��һ����һ��һ�����ȥ��
 * ������һ���̶���0��ֻҪ������һ���Ϳ�����
 */
#define claw2ground 0
#define claw2top -270

/**
 * @brief ̧�������ת����ֵ��
 * ������һ������Ҫһ�����͸��ܴ��ֵ������һ��
 */
#define lift2ground 0
#define lift2top -350


extern GPIO_PinState claw_mode;

/*****��ŷ���������11.5�ο� 
 * 11.7������******/
void Finger1_Close(void);
void Finger1_Open(void);

void Finger2_Close(void);
void Finger2_Open(void);

void Claw_Open(void);
void Claw_Close(void);

void Claw_turn(void);

void Cylinder_PUSH(void);
void Cylinder_BACK(void);

//���ʹ�����
void belt_ctrl(float target_spd);

//��צ�������
void claw_motor(float target_spd);

//��צ�̶�
void claw_hold(void);

//��צУ׼
// void claw_calibration(void);

//������ָ�����������
void lift_motor(float target_pos);

void lift_hold(void);

#endif //INC_2024RC_R1_UPPER_H
