//
// Created by Ray on 2023/11/3.
//

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

void belt_ctrl(float target_spd)
{
    float belt_spd[3] = {target_spd,target_spd,target_spd};
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
