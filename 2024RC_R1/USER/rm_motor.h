//
// Created by Ray on 2023/10/26.
//

#ifndef INC_2024RC_R1_RM_MOTOR_H
#define INC_2024RC_R1_RM_MOTOR_H
#include "main.h"
#define ABS(x)      ((x)>0? (x):(-(x)))


//驱动器工作模式
#define  SPEED_CONTROL_MODE		    2
#define  VELOCITY_PLANNING_MODE     3
#define  CURRENT_MODE               4
#define  POSITION_CONTROL_MODE		5
#define  SPEED_TARQUE_CONTROL_MODE  6
#define  POSITION_TORQUE_MODE		7
#define  HOMEING_MODE		        9
#define  MOTO_OFF		            0

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CHASSIS_M3508_M1_ID = 0x201,
    CHASSIS_M3508_M2_ID = 0x202,
    CHASSIS_M3508_M3_ID = 0x203,
    CHASSIS_M3508_M4_ID = 0x204,
    CAN_CHASSIS_OTHER_ID = 0x1FF,
    CHASSIS_M3508_M5_ID = 0x205,
    CHASSIS_M3508_M6_ID = 0x206,
    CHASSIS_M3508_M7_ID = 0x207
}Can_Msg_Id_e;

/**
  * @brief T型速度规划结构体
  * @note
*/
typedef struct VELOCITY_PLANNING //速度规划
{
    float Distance;
    float Pstart;        //开始位置
    float Pend;          //结束位置
    float Vstart;        //开始的速度           // 单位：RPM 绝对值
    float Vmax;          //最大的速度
    float Vend;          //末尾的速度
    float Rac;           //加速路程的比例
    float Rde;           //减速路程的比例
    int flag;            //完成标志位，电机停下来的时候置1
}VELOCITY_PLANNING;

/**
 * @brief 回零模式结构体。说实在，这个模式和速度转矩没什么差别。直接用速度转矩即可
*/
typedef struct
{
    float current;
    float Vel;				//目标速度
    float output;
    int16_t  TARGET_TORQUE;//目标转矩，用电流表示
    int flag;
    int32_t cnt;
}HOMING_MODE_TYPE;

/**
  * @brief  速度转矩模式结构体
*/
typedef struct
{
    float Current;
    float Target_Vel;//目标速度
    float Output;
    int16_t  TARGET_TORQUE; //目标转矩，用电流表示
    int Flag;
    int32_t Cnt;
}VELOCITY_TARQUE_TYPDEF;

/**
  * @brief  位置转矩结构体
  */
typedef struct
{
    float Current;
    float Pos;//目标位置
    float Output;
    int16_t  TARGET_TORQUE;//目标转矩，用电流表示
    int Flag;
    int32_t Cnt;
}POS_TARQUE_TYPDEF;

/**
  * @brief  电机种类  M3508和M2006
*/
typedef enum
{
    M_3508 = 1,
    M_2006 = 2,
    NONE = 3  //表示没有接入电机
}MotorType_TypeDef;

typedef struct
{
    uint32_t Motor_Mode;//电机模式
    //POSITION_CONTROL_MODE位置模式
    //POSITION_TARQUE_CONTROL_MODE位置_力度模式
    //SPEED_TARQUE_CONTROL_MODE位置_力度模式
    //SPEED_CONTROL_MODE速度模式
    //MOTO_OFF电机关闭-->电流不发送
    //VELOCITY_PLANNING_MODE梯形规划模式

    MotorType_TypeDef Motor_Type;

    uint16_t  	ANGLE;            	// 采样转子角度
    int16_t  	RPM;				// 实际转子转速
    int16_t  	CURRENT;			// 实际转矩电流
    int16_t  	TARGET_CURRENT;		// 目标转矩电流

    int16_t  TARGET_POS;		//目标角度(位置)
    float    TARGET_RPM;		//目标转速
    int      Velflag;			//数度为零时，置1

    VELOCITY_PLANNING 		Velocity_Planning;	//速度规划
    HOMING_MODE_TYPE 		HomingMode;			//电机回零模式
    VELOCITY_TARQUE_TYPDEF  Velocity_Tarque;	//速度转矩结构体
    POS_TARQUE_TYPDEF		Position_Tarque; 	//位置转矩结构体

    // 角度积分时用到下面变量
    float		REAL_ANGLE;         //处理过的真实角度（必须用float）
    uint8_t	 	FIRST_ANGLE_INTEGRAL_FLAG;  //?
    uint16_t 	LAST_ANGLE;   //?
    int16_t 	Filter_RPM;
}MOTOR_REAL_INFO;


typedef struct ROBOT_CHASSIS
{
    float Vx;//底盘速度
    float Vy;
    float Vw;
    float world_x;//接收action传回来的世界坐标
    float world_y;
    float world_YAM;
    float Target_RPM[4];
    VELOCITY_PLANNING TPlaning;
}ROBOT_CHASSIS;//底盘规划结构体


void get_motor_measure(CAN_RxHeaderTypeDef *msg, uint8_t Data[8]);
void RM_MOTOR_Angle_Integral(MOTOR_REAL_INFO* RM_MOTOR);
void M3508_Send_Currents(void);
void Motor_Control(void);
void Velocity_Planning_MODE(MOTOR_REAL_INFO *M3508_MOTOR);
void Velocity_Planning_setpos(MOTOR_REAL_INFO *M3508_MOTOR,float Pstart,float Pend,float Vstart,float Vmax,float Vend,float Rac,float Rde);
float Max_Value_Limit(float Value, float Limit);
float Position_Control(MOTOR_REAL_INFO *MOTOR_REAL_INFO,float Target_Pos);
void Speed_Control(MOTOR_REAL_INFO* RM_MOTOR, float Target_RPM);
void Vel_Torque_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, uint16_t Target_Torque, float Target_Vel);
void Pos_Torque_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, uint16_t Target_Torque, float Target_Pos);
void Homeing_Mode(MOTOR_REAL_INFO* RM_MOTOR);
// M3508返回的电机真实信息

#endif //INC_2024RC_R1_RM_MOTOR_H
