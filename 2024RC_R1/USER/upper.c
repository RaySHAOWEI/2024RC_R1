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
    Position_Control(&motorRealInfo[3], target_pos);
    Motor_Control();
}

void claw_hold(void)
{
    Speed_Control(&motorRealInfo[3], 0);
    Motor_Control();
}

/**
 * @brief ��צУ׼����
 * �����ԣ�����֮��ֱ��ͬ��õ����������У׼
 */
void claw_calibration(void)
{
    float offset = 10 / 360 * 8192;//������ת10�ȣ������������Ǹ�����������������
    Pos_Torque_Control(&motorRealInfo[3], 1024, offset);
    if(motorRealInfo[3].Position_Tarque.Flag == 1){
        motorRealInfo[3].REAL_ANGLE = 0;            //���ԽǶ�����
        motorRealInfo[3].Position_Tarque.Pos = 0;   //ֹͣת��
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
