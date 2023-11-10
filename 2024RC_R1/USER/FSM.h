//
// Created by Ray on 2023/11/5.
//

#ifndef INC_2024RC_R1_FSM_H
#define INC_2024RC_R1_FSM_H

#include "chassis.h"
#include "upper.h"

typedef enum {
    init,               //初始化
    calibration,        //校准
    chassis_ctrl,      //底盘控制模式
	chassis_ctrl_change,//换头模式
    upper_ctrl,         //上层控制模式
}Robot_State;

void chassis_motor_init(void);

void Upper_INIT(void);

void free_ctrl(void);

void R1_config(void);

void lift_init_state(void);

void lift_work_state(void);

void claw_init_state(void);

void claw_work_state(void);

float belt_calc(void);

void fsm(void);

#endif //INC_2024RC_R1_FSM_H
