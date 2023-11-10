//
// Created by Ray on 2023/11/5.
//

#ifndef INC_2024RC_R1_FSM_H
#define INC_2024RC_R1_FSM_H

#include "chassis.h"
#include "upper.h"

typedef enum {
    init,               //��ʼ��
    calibration,        //У׼
    chassis_ctrl,      //���̿���ģʽ
	chassis_ctrl_change,//��ͷģʽ
    upper_ctrl,         //�ϲ����ģʽ
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
