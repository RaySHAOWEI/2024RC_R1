//
// Created by Ray on 2023/11/4.
//

#include "chassis.h"
#include "air_joy.h"

MOTOR_REAL_INFO ChassisInfo[4];
PID_T Chassis_PID_RPM[4]; //速度pid信息
PID_T Chassis_PID_POS[4];	//位置pid信息

void get_chassis_motor_measure(CAN_RxHeaderTypeDef *msg, uint8_t Data[8]) {
    switch (msg->StdId)  // 检测标准ID
    {
        case CHASSIS_M3508_M1_ID: {
            ChassisInfo[0].ANGLE = (uint16_t) ((Data[0] << 8) | Data[1]);  // 转子机械角度
            ChassisInfo[0].RPM = (uint16_t) ((Data[2] << 8) | Data[3]);  // 实际转子转速
            ChassisInfo[0].CURRENT = (uint16_t) ((Data[4] << 8) | Data[5]);  // 实际转矩电流
        };
            break;

        case CHASSIS_M3508_M2_ID: {
            ChassisInfo[1].ANGLE = (uint16_t) ((Data[0] << 8) | Data[1]);  // 转子机械角度
            ChassisInfo[1].RPM = (uint16_t) ((Data[2] << 8) | Data[3]);  // 实际转子转速
            ChassisInfo[1].CURRENT = (uint16_t) ((Data[4] << 8) | Data[5]);  // 实际转矩电流
        };
            break;

        case CHASSIS_M3508_M3_ID: {
            ChassisInfo[2].ANGLE = (uint16_t) ((Data[0] << 8) | Data[1]);  // 转子机械角度
            ChassisInfo[2].RPM = (uint16_t) ((Data[2] << 8) | Data[3]);  // 实际转子转速
            ChassisInfo[2].CURRENT = (uint16_t) ((Data[4] << 8) | Data[5]);  // 实际转矩电流
        };
            break;

        case CHASSIS_M3508_M4_ID: {
            ChassisInfo[3].ANGLE = (uint16_t) ((Data[0] << 8) | Data[1]);  // 转子机械角度
            ChassisInfo[3].RPM = (uint16_t) ((Data[2] << 8) | Data[3]);  // 实际转子转速
            ChassisInfo[3].CURRENT = (uint16_t) ((Data[4] << 8) | Data[5]);  // 实际转矩电流
        };
            break;

        default:
            break;
    }
}

void Chassis_Send_Currents(void)
{
    CAN_TxHeaderTypeDef	TxHeader;
    uint8_t TxData[8];
    uint32_t Send_Mail_Box;

    //配置控制端
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x08;

    //配置仲裁段和数据段
    TxHeader.StdId = CAN_CHASSIS_ALL_ID;//0x200

    TxData[0] = (uint8_t)(ChassisInfo[0].TARGET_CURRENT >> 8);//0x201
    TxData[1] = (uint8_t) ChassisInfo[0].TARGET_CURRENT;

    TxData[2] = (uint8_t)(ChassisInfo[1].TARGET_CURRENT >> 8);//0x202
    TxData[3] = (uint8_t) ChassisInfo[1].TARGET_CURRENT;

    TxData[4] = (uint8_t)(ChassisInfo[2].TARGET_CURRENT >> 8);//0x203
    TxData[5] = (uint8_t) ChassisInfo[2].TARGET_CURRENT;

    TxData[6] = (uint8_t)(ChassisInfo[3].TARGET_CURRENT >> 8);//0x204
    TxData[7] = (uint8_t) ChassisInfo[3].TARGET_CURRENT;

//    TxData[0] = (uint8_t)(10000 >> 8);//0x201
//    TxData[1] = (uint8_t) 10000;

//    TxData[2] = (uint8_t)(10000 >> 8);//0x202
//    TxData[3] = (uint8_t) 10000;

//    TxData[4] = (uint8_t)(10000 >> 8);//0x203
//    TxData[5] = (uint8_t) 10000;

//    TxData[6] = (uint8_t)(10000 >> 8);//0x204
//    TxData[7] = (uint8_t) 10000;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &Send_Mail_Box);
}

void ROBOT_CHASSIS_INIT(void)
{
    ROBOT_chassis.world_x = 0;
    ROBOT_chassis.world_y = 0;
    ROBOT_chassis.world_YAM = 0;
    ROBOT_chassis.Vw = 0;
    ROBOT_chassis.Vx = 0;
    ROBOT_chassis.Vy = 0;
    for (int i = 0; i < 4; i++)
    {
        ROBOT_chassis.Target_RPM[i] = 0;
    }
    ROBOT_chassis.TPlaning.flag = 0;
    ROBOT_chassis.TPlaning.Distance = 0;
    ROBOT_chassis.TPlaning.Pend = 0;
    ROBOT_chassis.TPlaning.Pstart = 0;
    ROBOT_chassis.TPlaning.Rac = 0;
    ROBOT_chassis.TPlaning.Rde = 0;
    ROBOT_chassis.TPlaning.Vend = 0;
    ROBOT_chassis.TPlaning.Vmax = 0;
    ROBOT_chassis.TPlaning.Vstart = 0;
}

void chassis_motor_init(void)
{
    ChassisInfo[0].Motor_Type = M_3508;
    ChassisInfo[1].Motor_Type = M_3508;
    ChassisInfo[2].Motor_Type = M_3508;
    ChassisInfo[3].Motor_Type = M_3508;

    pid_param_init(&Chassis_PID_RPM[0], PID_Incremental, 16384, 20000, 20000, -0.5f, 16384, 7.0f, 0.0f, 0.1f);
    pid_param_init(&Chassis_PID_RPM[1], PID_Incremental, 16384, 20000, 20000, -0.5f, 16384, 7.0f, 0.0f, 0.1f);
    pid_param_init(&Chassis_PID_RPM[2], PID_Incremental, 16384, 20000, 20000, -0.5f, 16384, 7.0f, 0.0f, 0.1f);
    pid_param_init(&Chassis_PID_RPM[3], PID_Incremental, 16384, 20000, 20000, -0.5f, 16384, 7.0f, 0.0f, 0.1f);

}

#define Vy_MAX 6000.0F
#define Vx_MAX 6000.0F
#define Vw_MAX 30.0f
#define VX_CONTROL 0.15f
#define Angle_MAX 1.0f

void free_ctrl(void)
{
    if(YaoGan_RIGHT_X!=0 && YaoGan_LEFT_Y!=0 && YaoGan_LEFT_X!=0)
    {
        ROBOT_chassis.Vx =((YaoGan_LEFT_Y-1500.0f)/500)*Vx_MAX;
        ROBOT_chassis.Vy =((YaoGan_LEFT_X-1500.0f)/500)*Vy_MAX;
        ROBOT_chassis.Vw =((YaoGan_RIGHT_X-1500.0f)/500)*Vw_MAX;
    }
}

/*电机正方向以及编号沿顺时针
Vx为线速度，Vy为角速度，W为自转角速度
*/
// 底盘运动学结算
void chassis_kinematic(void)
{
    float a = cos(PI/4);
    ROBOT_chassis.Target_RPM[0] = (-ROBOT_chassis.Vy) * a + ROBOT_chassis.Vx * a + (ROBOT_chassis.Vw) * L;
    ROBOT_chassis.Target_RPM[1] = (-ROBOT_chassis.Vy) * a - ROBOT_chassis.Vx * a + (ROBOT_chassis.Vw) * L;
    ROBOT_chassis.Target_RPM[2] = (ROBOT_chassis.Vy) * a - ROBOT_chassis.Vx * a + (ROBOT_chassis.Vw) * L;
    ROBOT_chassis.Target_RPM[3] = (ROBOT_chassis.Vy) * a + ROBOT_chassis.Vx * a + (ROBOT_chassis.Vw) * L;

    for(int i=0; i < 4; i++) {
        ChassisInfo[i].Motor_Mode = SPEED_CONTROL_MODE;
        ChassisInfo[i].TARGET_RPM =  ROBOT_chassis.Target_RPM[i];
        pid_calc(&Chassis_PID_RPM[i], ChassisInfo[i].TARGET_RPM, ChassisInfo[i].RPM);//速度环
        ChassisInfo[i].TARGET_CURRENT = Chassis_PID_RPM[i].output;
    }
    Chassis_Send_Currents();    //使能电机，发送电流数据
}
