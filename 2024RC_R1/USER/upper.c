//
// Created by Ray on 2023/11/3.
//
/**
 * @file upper.c
 * @author Ray
 * @brief �ϲ�����ķ�װ�������Ҵ�ſ�һ�¾����ˣ�֮������������¸㡣��Ҫ��ѧһ����ô����rm_motor.c����ĺ�������Щ���������ǲ�������
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
    HAL_GPIO_WritePin(finger1_GPIO_Port, finger1_Pin, GPIO_PIN_SET);	 //������1������ָ�н�
}
    
void Finger1_Open(void){
    HAL_GPIO_WritePin(finger1_GPIO_Port, finger1_Pin, GPIO_PIN_RESET);	 	 //������1������ָ��
}

void Finger2_Close(void){
    HAL_GPIO_WritePin(finger2_GPIO_Port, finger2_Pin, GPIO_PIN_SET);	 //������2������ָ�н�
}
void Finger2_Open(void){
    HAL_GPIO_WritePin(finger2_GPIO_Port, finger2_Pin, GPIO_PIN_RESET);		 //������2������ָ��
}

void Claw_Open(void){
    HAL_GPIO_WritePin(claw_GPIO_Port, claw_Pin, GPIO_PIN_SET);
}//��צ�н�

void Claw_Close(void){
    HAL_GPIO_WritePin(claw_GPIO_Port, claw_Pin, GPIO_PIN_RESET);
}//��צ��

void Claw_turn(void)
{
    if (claw_mode == GPIO_PIN_SET){
        HAL_GPIO_WritePin(claw_GPIO_Port, claw_Pin, GPIO_PIN_RESET);
    }else {
        HAL_GPIO_WritePin(claw_GPIO_Port, claw_Pin, GPIO_PIN_SET);
    }
    
}

void Cylinder_PUSH(void){
    HAL_GPIO_WritePin(cylinder_GPIO_Port, cylinder_Pin, GPIO_PIN_SET);	 //��������
}
void Cylinder_BACK(void){
    HAL_GPIO_WritePin(cylinder_GPIO_Port, cylinder_Pin, GPIO_PIN_RESET);	 //���׻�
}

void belt_ctrl(float target_spd)
{
    float belt_spd[3] = {target_spd * 0.025f,target_spd * 4.0f,target_spd * 0.025f};//��һ�Σ�{target_spd * 0.025f,target_spd * 4.0f,target_spd * 0.025f}
    Speed_Control(&motorRealInfo[Motor_BELT_MOTOR_1], belt_spd[Motor_BELT_MOTOR_1]);
    Speed_Control(&motorRealInfo[Motor_BELT_MOTOR_2], belt_spd[Motor_BELT_MOTOR_2]);
    Speed_Control(&motorRealInfo[Motor_BELT_MOTOR_3], belt_spd[Motor_BELT_MOTOR_3]);
    Motor_Control();
}

/**
 * @brief ��צ������Ʋ��Ժ���
 * Ŀǰֻ��λ�û�
 * ����3508�Ƕȼ��㲻����׼�������ۻ����ܴ󣬽�����������û�е��λ������λ��˼·���£�
 *      ������target_pos = 0 �Ǽ�צ�ŵ�������ʱ�ľ��ԽǶȣ�
 *      ������Ǹ���һ��Сת�أ���������צ���µķ���ת����
 *      ���������ͻ��Ϊ�������ʱ��˵���Ѿ�������е��λ�ˣ���ʱ�������λ�����㡣
 * 
 * @param target_pos Ŀ��λ��
 */
void claw_motor(float target_pos)
{
    lim(&target_pos,claw2ground,claw2top);
    if (target_pos <= claw2ground && target_pos >= claw2top)//����λ
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
 * @brief ��צУ׼����
 * �����ԣ�����֮��ֱ��ͬ��õ����������У׼
 * 
 * 11.9 �ò��ˣ���ƨ��ͨ��ע�͵���
 */
// void claw_calibration(void)
// {
//     float offset = 100;
    
//     if (claw_m.pos != 0)        //posֵ�ȹ���(����һ�´˴�posֵ�Ƿ�����)
//     {
//         claw_motor(0);
//     }
//     else if (claw_m.pos == 0)   //������������˶�һ���Ƕ�
//     {
//         Position_Control(&motorRealInfo[Motor_CLAW], offset);
//         if(motorRealInfo[Motor_CLAW].stalled == 1){                 //�����ת
//             motorRealInfo[Motor_CLAW].REAL_ANGLE = 0;               //���ԽǶ�����
//             Position_Control(&motorRealInfo[Motor_CLAW], 0);   //ֹͣת��
//             Motor_Control();
//             return;
//         }
//         Motor_Control();
//     }
// }

/**
 * @brief ����������ơ�
 * 
 * @param target_pos 
 */
void lift_motor(float target_pos)
{
    lim(&target_pos,lift2ground,lift2top);
	if (target_pos <= lift2ground && target_pos >= lift2top)//����λ
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
