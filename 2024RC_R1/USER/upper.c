//
// Created by Ray on 2023/11/3.
//
/**
 * @file upper.c
 * @author Ray
 * @brief 上层机构的封装，这个大家大概看一下就行了，之后迭代还得重新搞。主要是学一下怎么运用rm_motor.c里面的函数。这些函数基本是不会变的了
 * @version 0.1
 * @date 2023-11-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "upper.h"


void lim(float *input, float max, float min) {
    if (max > min){
        if (*input > max) *input = max;
        if (*input < min) *input = min;
    }
    else if (max < min){
        if (*input > min) *input = min;
        if (*input < max) *input = max;
    }
}

GPIO_PinState claw_mode;

void Finger1_Close(void){
    HAL_GPIO_WritePin(finger1_GPIO_Port, finger1_Pin, GPIO_PIN_SET);	 //秧苗组1气动手指夹紧
}
    
void Finger1_Open(void){
    HAL_GPIO_WritePin(finger1_GPIO_Port, finger1_Pin, GPIO_PIN_RESET);	 	 //秧苗组1气动手指打开
}

void Finger2_Close(void){
    HAL_GPIO_WritePin(finger2_GPIO_Port, finger2_Pin, GPIO_PIN_SET);	 //秧苗组2气动手指夹紧
}
void Finger2_Open(void){
    HAL_GPIO_WritePin(finger2_GPIO_Port, finger2_Pin, GPIO_PIN_RESET);		 //秧苗组2气动手指打开
}

void Claw_Open(void){
    HAL_GPIO_WritePin(claw_GPIO_Port, claw_Pin, GPIO_PIN_SET);
}//夹爪夹紧

void Claw_Close(void){
    HAL_GPIO_WritePin(claw_GPIO_Port, claw_Pin, GPIO_PIN_RESET);
}//夹爪打开

void Claw_turn(void)
{
    if (claw_mode == GPIO_PIN_SET){
        HAL_GPIO_WritePin(claw_GPIO_Port, claw_Pin, GPIO_PIN_RESET);
    }else {
        HAL_GPIO_WritePin(claw_GPIO_Port, claw_Pin, GPIO_PIN_SET);
    }
    
}

void Cylinder_PUSH(void){
    HAL_GPIO_WritePin(cylinder_GPIO_Port, cylinder_Pin, GPIO_PIN_SET);	 //气缸推送
}
void Cylinder_BACK(void){
    HAL_GPIO_WritePin(cylinder_GPIO_Port, cylinder_Pin, GPIO_PIN_RESET);	 //气缸回
}

void belt_ctrl(float target_spd)
{
    float belt_spd[3] = {target_spd * 0.025f,target_spd * 4.0f,target_spd * 0.025f};//上一次：{target_spd * 0.025f,target_spd * 4.0f,target_spd * 0.025f}
    Speed_Control(&motorRealInfo[Motor_BELT_MOTOR_1], belt_spd[Motor_BELT_MOTOR_1]);
    Speed_Control(&motorRealInfo[Motor_BELT_MOTOR_2], belt_spd[Motor_BELT_MOTOR_2]);
    Speed_Control(&motorRealInfo[Motor_BELT_MOTOR_3], belt_spd[Motor_BELT_MOTOR_3]);
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
    lim(&target_pos,claw2ground,claw2top);
    if (target_pos <= claw2ground && target_pos >= claw2top)//软限位
    {
        Position_Control(&motorRealInfo[Motor_CLAW], target_pos);
        Motor_Control();
    }
}

void claw_hold(void)
{
    Position_Control(&motorRealInfo[Motor_CLAW], motorRealInfo[Motor_CLAW].REAL_ANGLE);
    Motor_Control();
}

/**
 * @brief 夹爪校准函数
 * 待测试，测试之后直接同理得到升降电机的校准
 * 
 * 11.9 用不了，狗屁不通，注释掉了
 */
// void claw_calibration(void)
// {
//     float offset = 100;
    
//     if (claw_m.pos != 0)        //pos值先归零(测试一下此处pos值是否会归零)
//     {
//         claw_motor(0);
//     }
//     else if (claw_m.pos == 0)   //归零后再往下运动一定角度
//     {
//         Position_Control(&motorRealInfo[Motor_CLAW], offset);
//         if(motorRealInfo[Motor_CLAW].stalled == 1){                 //电机堵转
//             motorRealInfo[Motor_CLAW].REAL_ANGLE = 0;               //绝对角度置零
//             Position_Control(&motorRealInfo[Motor_CLAW], 0);   //停止转动
//             Motor_Control();
//             return;
//         }
//         Motor_Control();
//     }
// }

/**
 * @brief 升降电机控制。
 * 
 * @param target_pos 
 */
void lift_motor(float target_pos)
{
    lim(&target_pos,lift2ground,lift2top);
	if (target_pos <= lift2ground && target_pos >= lift2top)//软限位
    {
        Position_Control(&motorRealInfo[Motor_UPLIFT], target_pos);
        Motor_Control();
	}
}

void lift_hold(void)
{
    Position_Control(&motorRealInfo[Motor_UPLIFT], motorRealInfo[Motor_UPLIFT].REAL_ANGLE);
    Motor_Control();
}
