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

#define CAN_CHASSIS_ALL_ID        0x200
#define CHASSIS_M3508_M1_ID       0x201
#define CHASSIS_M3508_M2_ID       0x202
#define CHASSIS_M3508_M3_ID       0x203
#define CHASSIS_M3508_M4_ID       0x204

#define PI 3.14
#define L 80

extern ROBOT_CHASSIS ROBOT_chassis;   // �����˵��̽ṹ��

void get_chassis_motor_measure(CAN_RxHeaderTypeDef *msg, uint8_t Data[8]);

void Chassis_Send_Currents(void);

void ROBOT_CHASSIS_INIT(void);

void chassis_motor_init(void);

void free_ctrl(void);

void chassis_kinematic(void);

#endif //INC_2024RC_R1_CHASSIS_H