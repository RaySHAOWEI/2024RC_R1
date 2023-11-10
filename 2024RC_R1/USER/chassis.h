//
// Created by Ray on 2023/11/4.
//

#ifndef INC_2024RC_R1_CHASSIS_H
#define INC_2024RC_R1_CHASSIS_H

#include "rm_motor.h"
#include "main.h"
#include "stdio.h"
#include <math.h>
#include "pid.h"
#include "usr_can.h"
#include "air_joy.h"

#define CAN_CHASSIS_ALL_ID        0x200
#define CHASSIS_M3508_M1_ID       0x201
#define CHASSIS_M3508_M2_ID       0x202
#define CHASSIS_M3508_M3_ID       0x203
#define CHASSIS_M3508_M4_ID       0x204



#define COS60               0.500000f
#define COS30               0.866025f
#define COS45               0.707106f
#define V_REAL              0.128f/60                   //轮子的线速度
#define PI                  3.1415926f                  //PI的值
#define WHEEL_R             0.152f/2                    //轮子半径 
#define RM_transition_MS    (PI*WHEEL_R)/570.0f         //转速与速度的转换  
#define MS_transition_RM    570.0f/(PI*WHEEL_R)         //速度与转速的转换  
#define L 70

#define CHASSIS_R 0.70f


extern ROBOT_CHASSIS ROBOT_chassis;   // 机器人底盘结构体
extern MOTOR_REAL_INFO ChassisInfo[4];
extern PID_T Chassis_PID_RPM[4]; //速度pid信息
extern PID_T Chassis_PID_POS[4];	//位置pid信息

void get_chassis_motor_measure(CAN_RxHeaderTypeDef *msg, uint8_t Data[8]);

void Chassis_Send_Currents(void);

void ROBOT_CHASSIS_INIT(void);

void chassis_kinematic(void);

#endif //INC_2024RC_R1_CHASSIS_H
