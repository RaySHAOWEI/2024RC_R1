//
// Created by Ray on 2023/11/3.
//

#include "upper.h"

extern uint8_t RxData[8]; // 数据接收数组，can的数据帧只有8帧
extern uint8_t TxData[8];

extern MOTOR_REAL_INFO motorRealInfo[7];
extern PID_T MOTOR_PID_RPM[7]; //速度pid信息
extern PID_T MOTOR_PID_POS[7];	//位置pid信息

void Upper_INIT(void)
{
    motorRealInfo[0].Motor_Type = M_3508;
    motorRealInfo[1].Motor_Type = M_3508;
    motorRealInfo[2].Motor_Type = M_3508;
    motorRealInfo[3].Motor_Type = M_3508;
    motorRealInfo[4].Motor_Type = M_3508;
    motorRealInfo[5].Motor_Type = NONE;
    motorRealInfo[6].Motor_Type = NONE;

    pid_param_init(&MOTOR_PID_RPM[0], PID_Incremental, 16384, 1024, 2048, 0.1f, 16384, 15, 4, 0.1);
    pid_param_init(&MOTOR_PID_RPM[1], PID_Incremental, 16384, 1024, 2048, 0.1f, 16384, 15, 4, 0.1);
    pid_param_init(&MOTOR_PID_RPM[2], PID_Incremental, 16384, 1024, 2048, 0.1f, 16384, 15, 4, 0.1);
    pid_param_init(&MOTOR_PID_RPM[3], PID_Incremental, 16384, 1024, 2048, 0.1f, 16384, 15, 0, 0);
    pid_param_init(&MOTOR_PID_RPM[4], PID_Incremental, 16384, 1024, 2048, 0.1f, 16384, 10, 0, 0.6);

    pid_param_init(&MOTOR_PID_POS[0], PID_Position, 8192, 1024, 2048, 0.1f, 8192, 10, 0, 0.3);
    pid_param_init(&MOTOR_PID_POS[1], PID_Position, 8192, 1024, 2048, 0.1f, 8192, 10, 0, 0.3);
    pid_param_init(&MOTOR_PID_POS[2], PID_Position, 8192, 1024, 2048, 0.1f, 8192, 10, 0, 0.3);
    pid_param_init(&MOTOR_PID_POS[3], PID_Position, 8192, 1024, 2048, 0.1f, 8192, 12, 0.3, 0);
    pid_param_init(&MOTOR_PID_POS[4], PID_Position, 8192, 1024, 2048, 0.1f, 8192, 3, 0.3, 0.3);
}

void belt_ctrl(float target_spd)
{
    float belt_spd[3] = {target_spd,target_spd,target_spd};
    Speed_Control(&motorRealInfo[0], belt_spd[0]);
    Speed_Control(&motorRealInfo[1], belt_spd[1]);
    Speed_Control(&motorRealInfo[2], belt_spd[2]);
}

void claw_motor(float target_pos)
{
    Position_Control(&motorRealInfo[3], target_pos);
}

void lift_motor(float target_pos)
{
    Position_Control(&motorRealInfo[4], target_pos);
}
