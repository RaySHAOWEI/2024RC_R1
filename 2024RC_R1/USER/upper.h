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

extern uint8_t RxData[8]; // 数据接收数组，can的数据帧只有8帧
extern uint8_t TxData[8];

extern MOTOR_REAL_INFO motorRealInfo[7];
extern PID_T MOTOR_PID_RPM[7]; //速度pid信息
extern PID_T MOTOR_PID_POS[7];	//位置pid信息

#define M3508_BELT_MOTOR_ID_1       0x201
#define M3508_BELT_MOTOR_ID_2       0x202
#define M3508_BELT_MOTOR_ID_3       0x203
#define M3508_CLAW                  0x204
#define M3508_UPLIFT                0x205

/*****电磁阀控制命令 引脚可根据板子调整 11.5廖俊******/   //！！有可能搞反！！
#define Finger_Open1  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);	 //秧苗组1气动手指打开
#define Finger_Close1  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);	 	 //秧苗组1气动手指夹紧

#define Finger_Open2   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);	 //秧苗组2气动手指打开
#define Finger_Close2  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);		 //秧苗组2气动手指夹紧

#define Cylinder_PUSH  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);		 //气缸推送
#define Cylinder_BACK  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);	 //气缸回

#define Claw_Open   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET);		 //夹爪打开
#define Claw_Close  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_SET);		 //夹爪夹紧

//传送带控制
void belt_ctrl(float target_spd);

//夹爪电机控制
void claw_motor(float target_spd);

//夹爪固定
void claw_hold(void);

//夹爪校准
void claw_calibration(void);

//气动手指升降电机控制
void lift_motor(float target_pos);

void lift_hold(void);

#endif //INC_2024RC_R1_UPPER_H
