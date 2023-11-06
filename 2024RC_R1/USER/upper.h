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

#define M3508_BELT_MOTOR_ID_1       0x201
#define M3508_BELT_MOTOR_ID_2       0x202
#define M3508_BELT_MOTOR_ID_3       0x203
#define M3508_CLAW                  0x204
#define M3508_UPLIFT                0x205

/*****��ŷ��������� ���ſɸ��ݰ��ӵ��� 11.5�ο�******/   //�����п��ܸ㷴����
#define Finger_Open1  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);	 //������1������ָ��
#define Finger_Close1  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);	 	 //������1������ָ�н�

#define Finger_Open2   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);	 //������2������ָ��
#define Finger_Close2  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);		 //������2������ָ�н�

#define Cylinder_PUSH  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);		 //��������
#define Cylinder_BACK  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);	 //���׻�

#define Claw_Open   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET);		 //��צ��
#define Claw_Close  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_SET);		 //��צ�н�

//���ʹ�����
void belt_ctrl(float target_spd);

//��צ�������
void claw_motor(float target_spd);

//��צ�̶�
void claw_hold(void);

//��צУ׼
void claw_calibration(void);

//������ָ�����������
void lift_motor(float target_pos);

void lift_hold(void);

#endif //INC_2024RC_R1_UPPER_H
