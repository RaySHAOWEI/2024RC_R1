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

typedef struct Upper_Motor
{
    MOTOR_REAL_INFO *motorinfo;
    float init_pos;     //初始位置，即最小值
    float final_pos;    //最终位置，即最大值
    float pos;
}upper_motor;

#define Motor_BELT_MOTOR_1       0
#define Motor_BELT_MOTOR_2       1
#define Motor_BELT_MOTOR_3       2
#define Motor_CLAW               3
#define Motor_UPLIFT             4

/**
 * @brief 夹爪机械臂转动阈值。
 * 没试过先全部给0，
 * 数值不要瞎几把乱设，一定一定先一点一点加上去。
 * 先设置一个固定是0，只要调另外一个就可以了
 */
#define claw2ground 0
#define claw2top -270

/**
 * @brief 抬升电机的转动阈值。
 * 和上面一样，不要一上来就给很大的值，先试一下
 */
#define lift2ground 0
#define lift2top -350

/*****电磁阀控制命令11.5廖俊 
 * 11.7许少威******/
#define Finger1_Open  HAL_GPIO_WritePin(finger1_GPIO_Port, finger1_Pin, GPIO_PIN_RESET);	 //秧苗组1气动手指打开
#define Finger1_Close  HAL_GPIO_WritePin(finger1_GPIO_Port, finger1_Pin, GPIO_PIN_SET);	 	 //秧苗组1气动手指夹紧

#define Finger2_Open   HAL_GPIO_WritePin(finger2_GPIO_Port, finger2_Pin, GPIO_PIN_RESET);	 //秧苗组2气动手指打开
#define Finger2_Close  HAL_GPIO_WritePin(finger2_GPIO_Port, finger2_Pin, GPIO_PIN_SET);		 //秧苗组2气动手指夹紧

#define Claw_Open   HAL_GPIO_WritePin(claw_GPIO_Port, claw_Pin, GPIO_PIN_RESET);		    //夹爪打开
#define Claw_Close  HAL_GPIO_WritePin(claw_GPIO_Port, claw_Pin, GPIO_PIN_SET);		        //夹爪夹紧

#define Cylinder_PUSH  HAL_GPIO_WritePin(cylinder_GPIO_Port, cylinder_Pin, GPIO_PIN_SET);	 //气缸推送
#define Cylinder_BACK  HAL_GPIO_WritePin(cylinder_GPIO_Port, cylinder_Pin, GPIO_PIN_RESET);	 //气缸回

//传送带控制
void belt_ctrl(float target_spd);

//夹爪电机控制
void claw_motor(float target_spd);

//夹爪固定
void claw_hold(void);

//夹爪校准
// void claw_calibration(void);

//气动手指升降电机控制
void lift_motor(float target_pos);

void lift_hold(void);

#endif //INC_2024RC_R1_UPPER_H
