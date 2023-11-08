//
// Created by Ray on 2023/11/5.
//

#include "FSM.h"

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

       //传送带电机
    pid_param_init(&MOTOR_PID_RPM[0], PID_Incremental, 16384, 1024, 0, 0.1f, 16384, 15, 4, 0.1);
    pid_param_init(&MOTOR_PID_RPM[1], PID_Incremental, 16384, 1024, 0, 0.1f, 16384, 15, 4, 0.1);
    pid_param_init(&MOTOR_PID_RPM[2], PID_Incremental, 16384, 1024, 0, 0.1f, 16384, 15, 4, 0.1);

    pid_param_init(&MOTOR_PID_POS[0], PID_Position, 16384, 1024, 0, 0.1f, 16384, 10, 0, 0.3);
    pid_param_init(&MOTOR_PID_POS[1], PID_Position, 16384, 1024, 0, 0.1f, 16384, 10, 0, 0.3);
    pid_param_init(&MOTOR_PID_POS[2], PID_Position, 16384, 1024, 0, 0.1f, 16384, 10, 0, 0.3);
    //这三个可以不用注册位置环。


    //抬升和夹爪电机
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


void lift_init_state(void)//秧苗部分初始状态
{

    lift_motor(lift2ground);
    Finger1_Open;
    Finger2_Open;
}

void lift_work_state(void)//秧苗部分工作状态
{
    Finger1_Close;
    Finger2_Close;
    lift_motor(lift2top);
}

void claw_init_state(void)//夹爪部分初始状态
{
    Claw_Open;
    claw_motor(claw2ground);
}

void claw_work_state(void)//夹爪部分工作状态
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
}//测试一下这个belt_level能不能正常加减。

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
        //夹爪、抬升电机校准，没什么思路，先放着。大不了全手动
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
//			else if (1300 < SWB && SWB < 1700)
//            {
//                lift_hold();
//            } 
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
                claw_hold();
                // claw_calibration();
            } 
			else if (SWC > 1800)
            {
                claw_work_state();
            }
            //推杆控制
            if (SWA < 1500)
            {
                Cylinder_BACK;//收回
            } else if(SWA > 1500)
            {
                Cylinder_PUSH;//发射
            }
            //传送带控制
            if (YaoGan_RIGHT_Y < 1100)
            {
                belt_level++;
                belt_ctrl(belt_calc());
            } else if (YaoGan_RIGHT_Y > 1900)
            {
                belt_level--;
                belt_ctrl(belt_calc());
            }
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
            //摇杆控制电机
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
            //秧苗气动手指
            if (1300 < SWB && SWB < 1700)
            {
                Finger1_Open;
                Finger2_Open;
            } else if (SWB > 1800)
            {
                Finger1_Close;
                Finger2_Close;
            }
            //夹爪气缸
            if (1300 < SWC && SWC < 1700)
            {
                Claw_Open;
            } else if (SWC > 1800)
            {
                Claw_Close;
            }
            //推杆控制
            if (SWA < 1500)
            {
                Cylinder_BACK;//收回
            } else if(SWA > 1500)
            {
                Cylinder_PUSH;//发射
            }
            //传送带控制
            if (YaoGan_RIGHT_Y < 1100)
            {
                belt_level++;
                belt_ctrl(belt_calc());
            } else if (YaoGan_RIGHT_Y > 1900)
            {
                belt_level--;
                belt_ctrl(belt_calc());
            }
        } else if (SWD < 1500)//切换为上层控制模式
        {
            state = chassis_ctrl;
        }
        break;

    default:
        break;
    }
}
