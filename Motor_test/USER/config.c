//
// Created by Ray on 2023/11/26.
//

#include "config.h"

/**
 * @brief 底盘电机类型初始化
 *
 */
void can1_config(void)
{
    can1motorRealInfo[MOTOR_LEFT].Motor_Type = M_3508;  // 秧苗夹取电机
    can1motorRealInfo[MOTOR_RIGHT].Motor_Type = M_3508; // 秧苗夹取电机

    // 速度环
		pid_param_init(&can1MOTOR_PID_RPM[MOTOR_LEFT], PID_Position, 16384, 1024, 0, 10.0, 0, 18.0f, 0.0f, 0.0f);
    pid_param_init(&can1MOTOR_PID_RPM[MOTOR_RIGHT], PID_Position, 16384, 1024, 0, 10.0, 0, 18.0f, 0.0f, 0.0f);
    // 位置环
    // pid_param_init(&can1MOTOR_PID_POS[MOTOR_LEFT], PID_Position, 3000, 800, 0.7, 0.1f, 16384, 15.0f, 0.0f, 0.5f);
    // pid_param_init(&can1MOTOR_PID_POS[MOTOR_RIGHT], PID_Position, 3000, 800, 0.7, 0.1f, 16384, 15.0f, 0.0f, 0.5f);
}

void can2_config(void)
{
    can2motorRealInfo[MOTOR_LEFT].Motor_Type = M_3508;  // 秧苗夹取电机
    can2motorRealInfo[MOTOR_RIGHT].Motor_Type = M_3508; // 秧苗夹取电机

    // 速度环
    pid_param_init(&can2MOTOR_PID_RPM[MOTOR_LEFT], PID_Position, 16384, 1024, 0, 10.0, 0, 18.0f, 0.0f, 0.0f);
    pid_param_init(&can2MOTOR_PID_RPM[MOTOR_RIGHT], PID_Position, 16384, 1024, 0, 10.0, 0, 18.0f, 0.0f, 0.0f);
    // 位置环
    // pid_param_init(&can1MOTOR_PID_POS[MOTOR_LEFT], PID_Position, 3000, 800, 0.7, 0.1f, 16384, 15.0f, 0.0f, 0.5f);
    // pid_param_init(&can1MOTOR_PID_POS[MOTOR_RIGHT], PID_Position, 3000, 800, 0.7, 0.1f, 16384, 15.0f, 0.0f, 0.5f);
}
