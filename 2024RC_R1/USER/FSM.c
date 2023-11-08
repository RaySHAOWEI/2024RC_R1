//
// Created by Ray on 2023/11/5.
//

#include "FSM.h"

Robot_State state = init;
int belt_level = 0;//���ʹ���λ��0��ʾֹͣ��1��ʾ���ٵ���2��ʾ���ٵ�

/**
 * @brief �ѳ�ʼ������ȫ���ŵ����������Ե�ʱ��Ķ���
 * p i d �ֱ���pid_param_init����������������Ҫ����������
 * pid_param_init�ĵ�������������������ƣ��ٶȻ���ò�Ҫ����8192��
 * ���������ֻ��Ͳ���ߡ����Ե�ʱ�����������һ�㣬��ֹ���ʧ��
 */
void chassis_motor_init(void)
{
    ChassisInfo[0].Motor_Type = M_3508;
    ChassisInfo[1].Motor_Type = M_3508;
    ChassisInfo[2].Motor_Type = M_3508;
    ChassisInfo[3].Motor_Type = M_3508;

    pid_param_init(&Chassis_PID_RPM[0], PID_Position, 8192, 1024, 0, 0.1, 16384, 12.0f, 0.0f, 0.2f);
    pid_param_init(&Chassis_PID_RPM[1], PID_Position, 8192, 1024, 0, 0.1, 16384, 12.0f, 0.0f, 0.2f);
    pid_param_init(&Chassis_PID_RPM[2], PID_Position, 8192, 1024, 0, 0.1, 16384, 12.0f, 0.0f, 0.2f);
    pid_param_init(&Chassis_PID_RPM[3], PID_Position, 8192, 1024, 0, 0.1, 16384, 12.0f, 0.0f, 0.2f);
}

void Upper_INIT(void)
{
    motorRealInfo[0].Motor_Type = M_3508;
    motorRealInfo[1].Motor_Type = M_3508;
    motorRealInfo[2].Motor_Type = M_3508;
    motorRealInfo[3].Motor_Type = M_3508;
    motorRealInfo[4].Motor_Type = M_3508;

       //���ʹ����
    pid_param_init(&MOTOR_PID_RPM[0], PID_Incremental, 16384, 1024, 0, 0.1f, 16384, 15, 4, 0.1);
    pid_param_init(&MOTOR_PID_RPM[1], PID_Incremental, 16384, 1024, 0, 0.1f, 16384, 15, 4, 0.1);
    pid_param_init(&MOTOR_PID_RPM[2], PID_Incremental, 16384, 1024, 0, 0.1f, 16384, 15, 4, 0.1);

    pid_param_init(&MOTOR_PID_POS[0], PID_Position, 16384, 1024, 0, 0.1f, 16384, 10, 0, 0.3);
    pid_param_init(&MOTOR_PID_POS[1], PID_Position, 16384, 1024, 0, 0.1f, 16384, 10, 0, 0.3);
    pid_param_init(&MOTOR_PID_POS[2], PID_Position, 16384, 1024, 0, 0.1f, 16384, 10, 0, 0.3);
    //���������Բ���ע��λ�û���


    //̧���ͼ�צ���
    pid_param_init(&MOTOR_PID_RPM[3], PID_Incremental, 8192, 900, 0, 0.1f, 16384, 15, 0.4, 0.2);
    pid_param_init(&MOTOR_PID_RPM[4], PID_Incremental, 8192, 900, 0, 0.1f, 16384, 20, 0.5, 0.2);

    pid_param_init(&MOTOR_PID_POS[3], PID_Position, 1024, 800, 0, 0.1f, 16384, 12, 0, 0.3);
    pid_param_init(&MOTOR_PID_POS[4], PID_Position, 2048, 800, 0, 0.1f, 16384, 15, 0, 0.3);

}

#define Vy_MAX 6000.0f
#define Vx_MAX 6000.0f
#define Vw_MAX 30.0f
#define VX_CONTROL 0.15f
#define Angle_MAX 1.0f

void free_ctrl(void)
{
    if(ABS(YaoGan_LEFT_Y-1500) > 5){
        ROBOT_chassis.Vx = ((YaoGan_LEFT_Y-1500.0f)/500) * Vx_MAX;
    }else if(ABS(YaoGan_LEFT_Y-1500) <= 5){
        ROBOT_chassis.Vx = 0;
    }
    if(ABS(YaoGan_LEFT_X-1500) > 5){
        ROBOT_chassis.Vy = ((YaoGan_LEFT_X-1500.0f)/500) * Vy_MAX;
    }else if(ABS(YaoGan_LEFT_X-1500) <= 5){
        ROBOT_chassis.Vy = 0;
    }
    if(ABS(YaoGan_RIGHT_X-1500) > 5){
        ROBOT_chassis.Vw = ((YaoGan_RIGHT_X-1500.0f)/500) * Vw_MAX;
    }else if(ABS(YaoGan_RIGHT_X-1500) <= 5){
        ROBOT_chassis.Vw = 0;
    }
}

void R1_config(void)
{
    ROBOT_CHASSIS_INIT();
	chassis_motor_init();
	Upper_INIT();
    state = init;
}


void lift_init_state(void)//���粿�ֳ�ʼ״̬
{

    lift_motor(lift2ground);
    Finger1_Open;
    Finger2_Open;
}

void lift_work_state(void)//���粿�ֹ���״̬
{
    Finger1_Close;
    Finger2_Close;
    lift_motor(lift2top);
}

void claw_init_state(void)//��צ���ֳ�ʼ״̬
{
    Claw_Open;
    claw_motor(claw2ground);
}

void claw_work_state(void)//��צ���ֹ���״̬
{
    Claw_Close;
    claw_motor(claw2top);
}

int belt_calc(void){
    if (belt_level >= 2)
    {
        belt_level = 2;
    }
    else if (belt_level <= 0)
    {
        belt_level = 0;
    }
    return belt_level * 4096;
}//����һ�����belt_level�ܲ��������Ӽ���

void fsm(void)
{
    switch (state)
    {
    case init:
        belt_level = 0;
        ROBOT_chassis.Vw = 0;
        ROBOT_chassis.Vx = 0;
        ROBOT_chassis.Vy = 0;
        state = calibration;
        break;
    
    case calibration:
        //��צ��̧�����У׼��ûʲô˼·���ȷ��š�����ȫ�ֶ�
        if(SWA != 0 && SWB != 0 && SWC != 0 && SWD != 0){//ȷ�Ϻ�ģ��ʼ���ɹ�
            if (SWD < 1500)
            {
                state = chassis_ctrl;
            } else if(SWD > 1500)
            {
                state = upper_ctrl;
            }
        }
        break;

    case chassis_ctrl:
        //���̿���ģʽ
        if (SWD < 1500)
        {
            free_ctrl();
            chassis_kinematic();
            //���粿�ֿ���
            if (SWB < 1200){
                lift_init_state();
            } 
//			else if (1300 < SWB && SWB < 1700)
//            {
//                lift_hold();
//            } 
			else if (SWB > 1800)
            {
                lift_work_state();
            }
            //��צ���ֿ���
            if (SWC < 1200){
                claw_init_state();
            } 
			else if (1300 < SWC && SWC < 1700)
            {
                claw_hold();
                // claw_calibration();
            } 
			else if (SWC > 1800)
            {
                claw_work_state();
            }
            //�Ƹ˿���
            if (SWA < 1500)
            {
                Cylinder_BACK;//�ջ�
            } else if(SWA > 1500)
            {
                Cylinder_PUSH;//����
            }
            //���ʹ�����
            if (YaoGan_RIGHT_Y < 1100)
            {
                belt_level++;
                belt_ctrl(belt_calc());
            } else if (YaoGan_RIGHT_Y > 1900)
            {
                belt_level--;
                belt_ctrl(belt_calc());
            }
        } else if (SWD > 1500)//�л�Ϊ�ϲ����ģʽ
        {
            ROBOT_chassis.Vw = 0;
            ROBOT_chassis.Vx = 0;
            ROBOT_chassis.Vy = 0;
            chassis_kinematic();
            //���̹���
            state = upper_ctrl;
        }
        break;

    case upper_ctrl:
        //�ϲ��ֶ�ģʽ
            ROBOT_chassis.Vw = 0;
            ROBOT_chassis.Vx = 0;
            ROBOT_chassis.Vy = 0;
            chassis_kinematic();
        if (SWD > 1500)
        {
            //ҡ�˿��Ƶ��
            if(ABS(YaoGan_LEFT_Y - 1500) > 5){
                float lift_pos = motorRealInfo[4].REAL_ANGLE;
                lift_motor(lift_pos - ((YaoGan_LEFT_Y-1500)/5));
            }
			else if(ABS(YaoGan_LEFT_Y-1500) <= 5){
                lift_hold();
            }
            if(ABS(YaoGan_RIGHT_X - 1500) > 5){
                float claw_pos = motorRealInfo[3].REAL_ANGLE;
                claw_motor(claw_pos + ((YaoGan_RIGHT_X-1500)/10));
            }
			else if(ABS(YaoGan_RIGHT_X - 1500) <= 5){
                claw_hold();
            }
            //����������ָ
            if (1300 < SWB && SWB < 1700)
            {
                Finger1_Open;
                Finger2_Open;
            } else if (SWB > 1800)
            {
                Finger1_Close;
                Finger2_Close;
            }
            //��צ����
            if (1300 < SWC && SWC < 1700)
            {
                Claw_Open;
            } else if (SWC > 1800)
            {
                Claw_Close;
            }
            //�Ƹ˿���
            if (SWA < 1500)
            {
                Cylinder_BACK;//�ջ�
            } else if(SWA > 1500)
            {
                Cylinder_PUSH;//����
            }
            //���ʹ�����
            if (YaoGan_RIGHT_Y < 1100)
            {
                belt_level++;
                belt_ctrl(belt_calc());
            } else if (YaoGan_RIGHT_Y > 1900)
            {
                belt_level--;
                belt_ctrl(belt_calc());
            }
        } else if (SWD < 1500)//�л�Ϊ�ϲ����ģʽ
        {
            state = chassis_ctrl;
        }
        break;

    default:
        break;
    }
}
