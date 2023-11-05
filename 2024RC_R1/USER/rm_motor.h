//
// Created by Ray on 2023/10/26.
//

#ifndef INC_2024RC_R1_RM_MOTOR_H
#define INC_2024RC_R1_RM_MOTOR_H
#include "main.h"
#define ABS(x)      ((x)>0? (x):(-(x)))


//����������ģʽ
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
  * @brief T���ٶȹ滮�ṹ��
  * @note
*/
typedef struct VELOCITY_PLANNING //�ٶȹ滮
{
    float Distance;
    float Pstart;        //��ʼλ��
    float Pend;          //����λ��
    float Vstart;        //��ʼ���ٶ�           // ��λ��RPM ����ֵ
    float Vmax;          //�����ٶ�
    float Vend;          //ĩβ���ٶ�
    float Rac;           //����·�̵ı���
    float Rde;           //����·�̵ı���
    int flag;            //��ɱ�־λ�����ͣ������ʱ����1
}VELOCITY_PLANNING;

/**
 * @brief ����ģʽ�ṹ�塣˵ʵ�ڣ����ģʽ���ٶ�ת��ûʲô���ֱ�����ٶ�ת�ؼ���
*/
typedef struct
{
    float current;
    float Vel;				//Ŀ���ٶ�
    float output;
    int16_t  TARGET_TORQUE;//Ŀ��ת�أ��õ�����ʾ
    int flag;
    int32_t cnt;
}HOMING_MODE_TYPE;

/**
  * @brief  �ٶ�ת��ģʽ�ṹ��
*/
typedef struct
{
    float Current;
    float Target_Vel;//Ŀ���ٶ�
    float Output;
    int16_t  TARGET_TORQUE; //Ŀ��ת�أ��õ�����ʾ
    int Flag;
    int32_t Cnt;
}VELOCITY_TARQUE_TYPDEF;

/**
  * @brief  λ��ת�ؽṹ��
  */
typedef struct
{
    float Current;
    float Pos;//Ŀ��λ��
    float Output;
    int16_t  TARGET_TORQUE;//Ŀ��ת�أ��õ�����ʾ
    int Flag;
    int32_t Cnt;
}POS_TARQUE_TYPDEF;

/**
  * @brief  �������  M3508��M2006
*/
typedef enum
{
    M_3508 = 1,
    M_2006 = 2,
    NONE = 3  //��ʾû�н�����
}MotorType_TypeDef;

typedef struct
{
    uint32_t Motor_Mode;//���ģʽ
    //POSITION_CONTROL_MODEλ��ģʽ
    //POSITION_TARQUE_CONTROL_MODEλ��_����ģʽ
    //SPEED_TARQUE_CONTROL_MODEλ��_����ģʽ
    //SPEED_CONTROL_MODE�ٶ�ģʽ
    //MOTO_OFF����ر�-->����������
    //VELOCITY_PLANNING_MODE���ι滮ģʽ

    MotorType_TypeDef Motor_Type;

    uint16_t  	ANGLE;            	// ����ת�ӽǶ�
    int16_t  	RPM;				// ʵ��ת��ת��
    int16_t  	CURRENT;			// ʵ��ת�ص���
    int16_t  	TARGET_CURRENT;		// Ŀ��ת�ص���

    int16_t  TARGET_POS;		//Ŀ��Ƕ�(λ��)
    float    TARGET_RPM;		//Ŀ��ת��
    int      Velflag;			//����Ϊ��ʱ����1

    VELOCITY_PLANNING 		Velocity_Planning;	//�ٶȹ滮
    HOMING_MODE_TYPE 		HomingMode;			//�������ģʽ
    VELOCITY_TARQUE_TYPDEF  Velocity_Tarque;	//�ٶ�ת�ؽṹ��
    POS_TARQUE_TYPDEF		Position_Tarque; 	//λ��ת�ؽṹ��

    // �ǶȻ���ʱ�õ��������
    float		REAL_ANGLE;         //���������ʵ�Ƕȣ�������float��
    uint8_t	 	FIRST_ANGLE_INTEGRAL_FLAG;  //?
    uint16_t 	LAST_ANGLE;   //?
    int16_t 	Filter_RPM;
}MOTOR_REAL_INFO;


typedef struct ROBOT_CHASSIS
{
    float Vx;//�����ٶ�
    float Vy;
    float Vw;
    float world_x;//����action����������������
    float world_y;
    float world_YAM;
    float Target_RPM[4];
    VELOCITY_PLANNING TPlaning;
}ROBOT_CHASSIS;//���̹滮�ṹ��


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
// M3508���صĵ����ʵ��Ϣ

#endif //INC_2024RC_R1_RM_MOTOR_H
