//
// Created by Ray on 2023/11/2.
//

#ifndef INC_2024RC_R1_USR_CAN_H
#define INC_2024RC_R1_USR_CAN_H

#include "main.h"
#include "rm_motor.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern MOTOR_REAL_INFO motorRealInfo[7];
extern MOTOR_REAL_INFO ChassisInfo[4];;

void can_filter_init(void);

#endif //INC_2024RC_R1_USR_CAN_H
