//
// Created by Ray on 2023/11/3.
//

#include "upper.h"

void belt_ctrl(float target_spd)
{
    float belt_spd[3] = {target_spd,target_spd,target_spd};
    Speed_Control(&motorRealInfo[0], belt_spd[0]);
    Speed_Control(&motorRealInfo[1], belt_spd[1]);
    Speed_Control(&motorRealInfo[2], belt_spd[2]);
    Motor_Control();
}

/**
 * @brief 夹爪电机控制测试函数
 * 目前只有位置环
 * 由于3508角度计算不够精准，导致累积误差很大，接下来打算采用机械限位辅助定位，思路如下：
 *      不妨设target_pos = 0 是夹爪放到最下面时的绝对角度，
 *      如果我们给定一个小转矩，令电机往夹爪向下的方向转动，
 *      当电机电流突变为大电流的时候，说明已经碰到机械限位了，此时电机绝对位置置零。
 * 
 * @param target_pos 目标位置
 */
void claw_motor(float target_pos)
{
    Position_Control(&motorRealInfo[3], target_pos);
    Motor_Control();
}

void claw_hold(void)
{
    Speed_Control(&motorRealInfo[3], 0);
    Motor_Control();
}

/**
 * @brief 夹爪校准函数
 * 待测试，测试之后直接同理得到升降电机的校准
 */
void claw_calibration(void)
{
    float offset = 10 / 360 * 8192;//在向下转10度（好像是向上是负数，向下是正数）
    Pos_Torque_Control(&motorRealInfo[3], 1024, offset);
    if(motorRealInfo[3].Position_Tarque.Flag == 1){
        motorRealInfo[3].REAL_ANGLE = 0;            //绝对角度置零
        motorRealInfo[3].Position_Tarque.Pos = 0;   //停止转动
    }
}

void lift_motor(float target_pos)
{
    Position_Control(&motorRealInfo[4], target_pos);
    Motor_Control();
}

void lift_hold(void)
{
    Speed_Control(&motorRealInfo[4],0);
    Motor_Control();
}
