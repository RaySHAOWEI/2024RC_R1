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

    pid_param_init(&Chassis_PID_RPM[0], PID_Position, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.0f, 0.2f);
    pid_param_init(&Chassis_PID_RPM[1], PID_Position, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.0f, 0.2f);
    pid_param_init(&Chassis_PID_RPM[2], PID_Position, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.0f, 0.2f);
    pid_param_init(&Chassis_PID_RPM[3], PID_Position, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.0f, 0.2f);
}

void Upper_INIT(void)
{
    motorRealInfo[0].Motor_Type = M_3508;
    motorRealInfo[1].Motor_Type = M_3508;
    motorRealInfo[2].Motor_Type = M_3508;
    motorRealInfo[3].Motor_Type = M_3508;
    motorRealInfo[4].Motor_Type = M_3508;

    //���ʹ����
    pid_param_init(&MOTOR_PID_RPM[0], PID_Incremental, 16384, 16384, 0, 0.1f, 16384, 13, 0.4, 0.2);
    pid_param_init(&MOTOR_PID_RPM[1], PID_Incremental, 16384, 16384, 0, 0.1f, 16384, 14, 0.5, 0.2);
    pid_param_init(&MOTOR_PID_RPM[2], PID_Incremental, 16384, 16384, 0, 0.1f, 16384, 13, 0.4, 0.2);

    //̧���ͼ�צ���
    pid_param_init(&MOTOR_PID_RPM[3], PID_Incremental, 8192, 900, 0, 0.1f, 16384, 15, 0.4, 0.2);
    pid_param_init(&MOTOR_PID_RPM[4], PID_Incremental, 8192, 900, 0, 0.1f, 16384, 20, 0.5, 0.2);

    pid_param_init(&MOTOR_PID_POS[3], PID_Position, 1024, 800, 0, 0.1f, 16384, 12, 0, 0.3);
    pid_param_init(&MOTOR_PID_POS[4], PID_Position, 2048, 800, 0, 0.1f, 16384, 15, 0, 0.3);


    //���������Բ���ע��λ�û���
    pid_param_init(&MOTOR_PID_POS[0], PID_Position, 16384, 1024, 0, 0.1f, 16384, 10, 0, 0.3);
    pid_param_init(&MOTOR_PID_POS[1], PID_Position, 16384, 1024, 0, 0.1f, 16384, 10, 0, 0.3);
    pid_param_init(&MOTOR_PID_POS[2], PID_Position, 16384, 1024, 0, 0.1f, 16384, 10, 0, 0.3);
    //���������Բ���ע��λ�û���
}

#define Vy_MAX 2.0f
#define Vx_MAX 2.0f
#define Vw_MAX 2.0f
#define VX_CONTROL 0.15f
#define Angle_MAX 1.0f

void free_ctrl(void)
{
    if(ABS(YaoGan_LEFT_Y-1500) > 10){
        ROBOT_chassis.Vx = ((YaoGan_LEFT_Y-1500.0f)/500) * Vx_MAX;
    }else if(ABS(YaoGan_LEFT_Y-1500) <= 10){
        ROBOT_chassis.Vx = 0;
    }
    if(ABS(YaoGan_LEFT_X-1500) > 10){
        ROBOT_chassis.Vy = ((YaoGan_LEFT_X-1500.0f)/500) * Vy_MAX;
    }else if(ABS(YaoGan_LEFT_X-1500) <= 10){
        ROBOT_chassis.Vy = 0;
    }
    if(ABS(YaoGan_RIGHT_X-1500) > 10){
        ROBOT_chassis.Vw = ((YaoGan_RIGHT_X-1500.0f)/500) * Vw_MAX;
    }else if(ABS(YaoGan_RIGHT_X-1500) <= 10){
        ROBOT_chassis.Vw = 0;
    }
}

void free_ctrl_change(void)
{
    if(ABS(YaoGan_LEFT_Y-1500) > 10){
        ROBOT_chassis.Vy = ((YaoGan_LEFT_Y-1500.0f)/500) * Vy_MAX;
    }else if(ABS(YaoGan_LEFT_Y-1500) <= 10){
        ROBOT_chassis.Vy = 0;
    }
    if(ABS(YaoGan_LEFT_X-1500) > 10){
        ROBOT_chassis.Vx = -((YaoGan_LEFT_X-1500.0f)/500) * Vy_MAX;
    }else if(ABS(YaoGan_LEFT_X-1500) <= 10){
        ROBOT_chassis.Vx = 0;
    }
    if(ABS(YaoGan_RIGHT_X-1500) > 10){
        ROBOT_chassis.Vw = ((YaoGan_RIGHT_X-1500.0f)/500) * Vw_MAX;
    }else if(ABS(YaoGan_RIGHT_X-1500) <= 10){
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
    Finger1_Open();
    Finger2_Open();
}

void lift_work_state(void)//���粿�ֹ���״̬
{
    Finger1_Close();
    Finger2_Close();
    lift_motor(lift2top);
}

void claw_init_state(void)//��צ���ֳ�ʼ״̬
{
    Claw_Close();
    claw_motor(claw2ground);
}

void claw_work_state(void)//��צ���ֹ���״̬
{
    Claw_Close();
    claw_motor(claw2top);
}

float belt_calc(void){
    if (belt_level >= 2)
    {
        belt_level = 2;
    }
	if (belt_level == 1)
	{
		belt_level = 1;
	}
    if (belt_level <= 0)
    {
        belt_level = 0;
    }
    return belt_level * 4096;
}//����һ�����belt_level�ܲ��������Ӽ��������ܣ�Ҫ����������Ȼһ���Ӿʹ�0����2�ˣ�

void fsm(void)
{
    switch (state)
    {
    case init:
        belt_level = 0;
        ROBOT_chassis.Vw = 0;
        ROBOT_chassis.Vx = 0;
        ROBOT_chassis.Vy = 0;
        Claw_Close();
        state = calibration;
        break;
    
    case calibration:
        //��צ��̧�����У׼��ûʲô˼·���ȷ��š�����ȫ�ֶ�
//        do
//        {
//            Homeing_Mode(&motorRealInfo[Motor_CLAW], 100, 512);
//			Motor_Control();
//        } while (motorRealInfo[Motor_CLAW].HomingMode.done_flag != 1);
//        do
//        {
//            Homeing_Mode(&motorRealInfo[Motor_UPLIFT], 100, 512);
//			Motor_Control();
//        } while (motorRealInfo[Motor_UPLIFT].HomingMode.done_flag != 1);
//			Claw_Close();
//			Homeing_Mode(&motorRealInfo[Motor_CLAW], 1000, 10000);
//			Homeing_Mode(&motorRealInfo[Motor_UPLIFT], 100, 1024);   || motorRealInfo[Motor_UPLIFT].HomingMode.done_flag == 0
// 		if (motorRealInfo[Motor_CLAW].HomingMode.done_flag == 0){
// 			Homeing_Mode(&motorRealInfo[Motor_CLAW], 1000, 8192);
// //			Homeing_Mode(&motorRealInfo[Motor_UPLIFT], 100, 1024);
// 			Motor_Control();
// 		}
// 		else {}
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
			else if (1300 < SWB && SWB < 1700)
            {
                lift_motor(lift2ground);
				state = chassis_ctrl_change;
            } 
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
                Claw_Open();
            } 
			else if (SWC > 1800)
            {
                claw_work_state();
            }
            //�Ƹ˿���
            if (SWA < 1500)
            {
				--belt_level;
                Cylinder_BACK();//�ջ�
            } else if(SWA > 1500)
            {
				++belt_level;
                Cylinder_PUSH();//����
            }
            belt_ctrl(belt_calc());
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
		
	case chassis_ctrl_change:
		//���̻�ͷ����ģʽ
        if (SWD < 1500)
        {
            free_ctrl_change();
            chassis_kinematic();
            //���粿�ֿ���
            if (SWB < 1200){
                lift_init_state();
				state = chassis_ctrl;
            } 
			else if (1300 < SWB && SWB < 1700)
            {
                lift_motor(lift2ground);
            } 
			else if (SWB > 1800)
            {
                lift_work_state();
				state = chassis_ctrl;
            }
            //��צ���ֿ���
            if (SWC < 1200){
                claw_init_state();
            } 
			else if (1300 < SWC && SWC < 1700)
            {
                Claw_Open();
            } 
			else if (SWC > 1800)
            {
                claw_work_state();
            }
            //�Ƹ˿���
            if (SWA < 1500)
            {
				--belt_level;
                Cylinder_BACK();//�ջ�
            } else if(SWA > 1500)
            {
				++belt_level;
                Cylinder_PUSH();//����
            }
            belt_ctrl(belt_calc());
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
            //ҡ�˿���̧�����
            if(ABS(YaoGan_LEFT_Y - 1500) > 10){
                float lift_pos = motorRealInfo[Motor_UPLIFT].REAL_ANGLE;
				if (lift_pos - ((YaoGan_LEFT_Y-1500)/5) > lift2ground){
					Speed_Control(&motorRealInfo[Motor_UPLIFT], -(YaoGan_LEFT_Y-1500));
                    Homeing_Mode(&motorRealInfo[Motor_UPLIFT], -(YaoGan_LEFT_Y-1500), 1024);
                    Motor_Control();
                }else{
					lift_motor(lift_pos - ((YaoGan_LEFT_Y-1500)/5));
				}
            }
			else if(ABS(YaoGan_LEFT_Y-1500) <= 10){
                lift_hold();
            }

            //ҡ�˿��Ƽ�צ���
            if(ABS(YaoGan_RIGHT_X - 1500) > 10){
				float claw_pos = motorRealInfo[Motor_CLAW].REAL_ANGLE;
				if (claw_pos + ((YaoGan_RIGHT_X-1500)/10) > claw2ground){
					Speed_Control(&motorRealInfo[Motor_CLAW], (YaoGan_RIGHT_X-1500));
                    Homeing_Mode(&motorRealInfo[Motor_CLAW], (YaoGan_RIGHT_X-1500), 8192);
                    Motor_Control();
                }
				else {
					claw_motor(claw_pos + ((YaoGan_RIGHT_X-1500)/10));
				}
            }
			else if(ABS(YaoGan_RIGHT_X - 1500) <= 10){
                claw_hold();
            }

            //����������ָ
            if (1300 < SWB && SWB < 1700)
            {
                Finger1_Open();
                Finger2_Open();
            } else if (SWB > 1800)
            {
                Finger1_Close();
                Finger2_Close();
            }
            //��צ����
            if (1300 < SWC && SWC < 1700)
            {
                Claw_Open();
            } else if (SWC > 1800)
            {
                Claw_Close();
            }
            //�Ƹ˿���
            if (SWA < 1500)
            {
				--belt_level;
                Cylinder_BACK();//�ջ�
            } else if(SWA > 1500)
            {
				++belt_level;
                Cylinder_PUSH();//����
            }
            belt_ctrl(belt_calc());
        } else if (SWD < 1500)//�л�Ϊ���̿���ģʽ
        {
            state = chassis_ctrl;
        }
        break;

    default:
        break;
    }
}
