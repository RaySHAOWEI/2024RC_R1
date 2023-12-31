//
// Created by Ray on 2023/11/5.
//

#include "FSM.h"

/**
 * @brief 状态机结构体，通过航模的拨杆状态组合，设定机器人的运动逻辑。
 * 
 * 注意！！！！！：这里面不能含有任何延时函数，HAL_Delay os_delay vTask什么的全部都不能有，不然机器人会发疯，
 * 具体的原因分析：在这里加了延时，pid的计算就会被推迟，等于说pid计算的是上一秒的电机真实值，一直累积误差机会很大，pid直接飙到最大值。
 * 
 * 之后这里会只有模式转换，具体任务会放在freertos.c里面，这样任务之间的延时就可以直接调用延时函数了
 */

Robot_State state = init;
int belt_level = 0;//传送带档位，0表示停止，1表示低速挡，2表示高速挡

/**
 * @brief 把初始化函数全部放到这里，方便调试的时候改动。
 * p i d 分别是pid_param_init的最后三项参数，主要调试这三项
 * pid_param_init的第三个参数是最大功率限制，速度环最好不要超过8192。
 * 最大功率限制只许低不许高。测试的时候可以再拉低一点，防止电机失控
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

    //传送带电机
    pid_param_init(&MOTOR_PID_RPM[0], PID_Incremental, 16384, 16384, 0, 0.1f, 16384, 13, 0.4, 0.2);
    pid_param_init(&MOTOR_PID_RPM[1], PID_Incremental, 16384, 16384, 0, 0.1f, 16384, 14, 0.5, 0.2);
    pid_param_init(&MOTOR_PID_RPM[2], PID_Incremental, 16384, 16384, 0, 0.1f, 16384, 13, 0.4, 0.2);

    //抬升和夹爪电机
    pid_param_init(&MOTOR_PID_RPM[3], PID_Incremental, 8192, 900, 0, 0.1f, 16384, 15, 0.4, 0.2);
    pid_param_init(&MOTOR_PID_RPM[4], PID_Incremental, 8192, 900, 0, 0.1f, 16384, 20, 0.5, 0.2);

    pid_param_init(&MOTOR_PID_POS[3], PID_Position, 1024, 800, 0, 0.1f, 16384, 12, 0, 0.3);
    pid_param_init(&MOTOR_PID_POS[4], PID_Position, 2048, 800, 0, 0.1f, 16384, 15, 0, 0.3);


    //这三个可以不用注册位置环。
    pid_param_init(&MOTOR_PID_POS[0], PID_Position, 16384, 1024, 0, 0.1f, 16384, 10, 0, 0.3);
    pid_param_init(&MOTOR_PID_POS[1], PID_Position, 16384, 1024, 0, 0.1f, 16384, 10, 0, 0.3);
    pid_param_init(&MOTOR_PID_POS[2], PID_Position, 16384, 1024, 0, 0.1f, 16384, 10, 0, 0.3);
    //这三个可以不用注册位置环。
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


void lift_init_state(void)//秧苗部分初始状态
{

    lift_motor(lift2ground);
    Finger1_Open();
    Finger2_Open();
}

void lift_work_state(void)//秧苗部分工作状态
{
    Finger1_Close();
    Finger2_Close();
    lift_motor(lift2top);
}

void claw_init_state(void)//夹爪部分初始状态
{
    Claw_Close();
    claw_motor(claw2ground);
}

void claw_work_state(void)//夹爪部分工作状态
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
}//测试一下这个belt_level能不能正常加减。（不能，要加消抖，不然一下子就从0跳到2了）

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
        //夹爪、抬升电机校准，没什么思路，先放着。大不了全手动
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
        if(SWA != 0 && SWB != 0 && SWC != 0 && SWD != 0){//确认航模初始化成功
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
        //底盘控制模式
        if (SWD < 1500)
        {
            free_ctrl();
            chassis_kinematic();
            //秧苗部分控制
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
            //夹爪部分控制
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
            //推杆控制
            if (SWA < 1500)
            {
				--belt_level;
                Cylinder_BACK();//收回
            } else if(SWA > 1500)
            {
				++belt_level;
                Cylinder_PUSH();//发射
            }
            belt_ctrl(belt_calc());
        } else if (SWD > 1500)//切换为上层控制模式
        {
            ROBOT_chassis.Vw = 0;
            ROBOT_chassis.Vx = 0;
            ROBOT_chassis.Vy = 0;
            chassis_kinematic();
            //底盘归零
            state = upper_ctrl;
        }
        break;
		
	case chassis_ctrl_change:
		//底盘换头控制模式
        if (SWD < 1500)
        {
            free_ctrl_change();
            chassis_kinematic();
            //秧苗部分控制
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
            //夹爪部分控制
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
            //推杆控制
            if (SWA < 1500)
            {
				--belt_level;
                Cylinder_BACK();//收回
            } else if(SWA > 1500)
            {
				++belt_level;
                Cylinder_PUSH();//发射
            }
            belt_ctrl(belt_calc());
        } else if (SWD > 1500)//切换为上层控制模式
        {
            ROBOT_chassis.Vw = 0;
            ROBOT_chassis.Vx = 0;
            ROBOT_chassis.Vy = 0;
            chassis_kinematic();
            //底盘归零
            state = upper_ctrl;
        }
        break;

    case upper_ctrl:
        //上层手动模式
            ROBOT_chassis.Vw = 0;
            ROBOT_chassis.Vx = 0;
            ROBOT_chassis.Vy = 0;
            chassis_kinematic();
        if (SWD > 1500)
        {
            //摇杆控制抬升电机
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

            //摇杆控制夹爪电机
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

            //秧苗气动手指
            if (1300 < SWB && SWB < 1700)
            {
                Finger1_Open();
                Finger2_Open();
            } else if (SWB > 1800)
            {
                Finger1_Close();
                Finger2_Close();
            }
            //夹爪气缸
            if (1300 < SWC && SWC < 1700)
            {
                Claw_Open();
            } else if (SWC > 1800)
            {
                Claw_Close();
            }
            //推杆控制
            if (SWA < 1500)
            {
				--belt_level;
                Cylinder_BACK();//收回
            } else if(SWA > 1500)
            {
				++belt_level;
                Cylinder_PUSH();//发射
            }
            belt_ctrl(belt_calc());
        } else if (SWD < 1500)//切换为底盘控制模式
        {
            state = chassis_ctrl;
        }
        break;

    default:
        break;
    }
}
