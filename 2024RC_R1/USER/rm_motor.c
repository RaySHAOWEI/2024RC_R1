//
// Created by Ray on 2023/10/26.
//
#include <math.h>
#include "rm_motor.h"
#include "can.h"
#include "pid.h"

MOTOR_REAL_INFO motorRealInfo[7];
PID_T MOTOR_PID_RPM[7]; //�ٶ�pid��Ϣ
PID_T MOTOR_PID_POS[7];	//λ��pid��Ϣ
ROBOT_CHASSIS ROBOT_chassis;   // �����˵��̽ṹ��

/**
 * @brief  ������ݶ�ȡ
 * @param  can���ݽ��սṹ��
 * @param  can���ݽ�������
 * @return none
*/
void get_motor_measure(CAN_RxHeaderTypeDef *msg, uint8_t Data[8])
{
    switch(msg -> StdId)  // ����׼ID
    {
        case CHASSIS_M3508_M1_ID:
        {
            motorRealInfo[0].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // ת�ӻ�е�Ƕ�
            motorRealInfo[0].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // ʵ��ת��ת��
            motorRealInfo[0].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // ʵ��ת�ص���
        }; break;

        case CHASSIS_M3508_M2_ID:
        {
            motorRealInfo[1].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // ת�ӻ�е�Ƕ�
            motorRealInfo[1].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // ʵ��ת��ת��
            motorRealInfo[1].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // ʵ��ת�ص���
        }; break;

        case CHASSIS_M3508_M3_ID:
        {
            motorRealInfo[2].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // ת�ӻ�е�Ƕ�
            motorRealInfo[2].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // ʵ��ת��ת��
            motorRealInfo[2].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // ʵ��ת�ص���
        }; break;

        case CHASSIS_M3508_M4_ID:
        {
            motorRealInfo[3].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // ת�ӻ�е�Ƕ�
            motorRealInfo[3].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // ʵ��ת��ת��
            motorRealInfo[3].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // ʵ��ת�ص���
        }; break;

        case CHASSIS_M3508_M5_ID:
        {
            motorRealInfo[4].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // ת�ӻ�е�Ƕ�
            motorRealInfo[4].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // ʵ��ת��ת��
            motorRealInfo[4].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // ʵ��ת�ص���
        }; break;

        case CHASSIS_M3508_M6_ID:
        {
            motorRealInfo[5].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // ת�ӻ�е�Ƕ�
            motorRealInfo[5].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // ʵ��ת��ת��
            motorRealInfo[5].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // ʵ��ת�ص���
        }; break;

        case CHASSIS_M3508_M7_ID:
        {
            motorRealInfo[6].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // ת�ӻ�е�Ƕ�
            motorRealInfo[6].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // ʵ��ת��ת��
            motorRealInfo[6].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // ʵ��ת�ص���
        }; break;

        default: break;
    }
}


/**
 * @brief M3508�ǶȻ���
 * @param ����ṹ��
 * @return NULL
*/
void RM_MOTOR_Angle_Integral(MOTOR_REAL_INFO* RM_MOTOR)
{
    static float Delta_Pos = 0;
    float Deceleration_P = 0;

    //��¼��һ�ν���ʱ������
    if(!RM_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG)
    {
        RM_MOTOR->LAST_ANGLE = RM_MOTOR->ANGLE;
        RM_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG = 1;
        return;
    }

    switch (RM_MOTOR->Motor_Type)
    {
        case M_3508:
            Deceleration_P = 19.0f;
            break;

        case M_2006:
            Deceleration_P = 36.0f;
            break;

        default:
            break;
    }

    //����Ƕȱ仯
    if (RM_MOTOR->RPM >= 0)
    {
        /* code */
        if(RM_MOTOR->ANGLE < RM_MOTOR->LAST_ANGLE)
        {
            if (ABS(8191 + RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE < 1250))
            {
                /* code */
                Delta_Pos = ((float)(8191 + RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE)/8191.0f) * 360.0f;
                Delta_Pos = Delta_Pos / Deceleration_P;	//���ٱ�
            }
        }
        else
        {
            Delta_Pos = ((float)(RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
            Delta_Pos = Delta_Pos / Deceleration_P;	//���ٱ�
        }

        //ƽͨ�˲�
        if(Delta_Pos > 0)
        {
            RM_MOTOR->REAL_ANGLE += Delta_Pos;  // ����
        }
    }
    else
    {
        if(RM_MOTOR->ANGLE > RM_MOTOR->LAST_ANGLE)
        {
            if(ABS(8191 - RM_MOTOR->ANGLE + RM_MOTOR->LAST_ANGLE) < 1250)  // ��������CAN����ʱ�������ת���ǶȽ����˲�
            {
                Delta_Pos = ((float)(8191 - RM_MOTOR->ANGLE + RM_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
                Delta_Pos = Delta_Pos /Deceleration_P;	//���ٱ�
            }
        }
        else
        {
            Delta_Pos = ((float)(RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
            Delta_Pos = Delta_Pos / Deceleration_P;	//���ٱ�
        }

        // �˲�
        if(Delta_Pos < 0)
            RM_MOTOR->REAL_ANGLE += Delta_Pos;  // ����
    }

    // �洢�Ƕ�ֵ
    RM_MOTOR->LAST_ANGLE = RM_MOTOR->ANGLE;
}

/**
 * @brief ���͵������
 * ��can2��������������
 * @param NULL
 * @return NULL
*/
void M3508_Send_Currents(void)
{
    CAN_TxHeaderTypeDef	TxHeader1;
    CAN_TxHeaderTypeDef	TxHeader2;
    uint8_t TxData[8];
    uint8_t TxData2[8];
    uint32_t Send_Mail_Box;
	uint32_t Send_Mail_Box2;

    //���ÿ��ƶ�
    TxHeader1.IDE = CAN_ID_STD;
    TxHeader1.RTR = CAN_RTR_DATA;
    TxHeader1.DLC = 0x08;

    //�����ٲöκ����ݶ�
    TxHeader1.StdId = CAN_CHASSIS_ALL_ID;//0x200

    // //���ÿ��ƶ�
    TxHeader2.IDE = CAN_ID_STD;
    TxHeader2.RTR = CAN_RTR_DATA;
    TxHeader2.DLC = 0x08;

    //�����ٲöκ����ݶ�
    TxHeader2.StdId = CAN_CHASSIS_OTHER_ID;//0x1FF

   TxData[0] = (uint8_t)(motorRealInfo[0].TARGET_CURRENT >> 8);//0x201
   TxData[1] = (uint8_t) motorRealInfo[0].TARGET_CURRENT;

   TxData[2] = (uint8_t)(motorRealInfo[1].TARGET_CURRENT >> 8);//0x202
   TxData[3] = (uint8_t) motorRealInfo[1].TARGET_CURRENT;

   TxData[4] = (uint8_t)(motorRealInfo[2].TARGET_CURRENT >> 8);//0x203
   TxData[5] = (uint8_t) motorRealInfo[2].TARGET_CURRENT;

   TxData[6] = (uint8_t)(motorRealInfo[3].TARGET_CURRENT >> 8);//0x204
   TxData[7] = (uint8_t) motorRealInfo[3].TARGET_CURRENT;

   TxData2[0] = (uint8_t)(motorRealInfo[0].TARGET_CURRENT >> 8);//0x205
   TxData2[1] = (uint8_t) motorRealInfo[0].TARGET_CURRENT;

//    TxData[0] = (uint8_t)(10000 >> 8);//0x201
//    TxData[1] = (uint8_t) 10000;

//    TxData[2] = (uint8_t)(10000 >> 8);//0x202
//    TxData[3] = (uint8_t) 10000;

//    TxData[4] = (uint8_t)(10000 >> 8);//0x203
//    TxData[5] = (uint8_t) 10000;

//    TxData[6] = (uint8_t)(10000 >> 8);//0x204
//    TxData[7] = (uint8_t) 10000;

//    TxData2[0] = (uint8_t)(10000 >> 8);//0x205
//    TxData2[1] = (uint8_t) 10000;


    HAL_CAN_AddTxMessage(&hcan2, &TxHeader1, TxData, &Send_Mail_Box);
    HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &Send_Mail_Box2);
}

/**
 * @brief �������ģʽ
 * @param NULL
 * @return NULL
*/
void Motor_Control(void)
{
    for(int i=0; i < 7; i++)
    {
        //�ж��Ƿ���ѡ�������ͣ�����ֱ���˳�
        if (motorRealInfo[i].Motor_Type == NONE)
            break;

        //���ģʽѡ��
        switch (motorRealInfo[i].Motor_Mode)
        {
            case SPEED_CONTROL_MODE: //�ٶ�ģʽ
            {
                pid_calc(&MOTOR_PID_RPM[i],motorRealInfo[i].TARGET_RPM,motorRealInfo[i].RPM);//�ٶȻ�
                break;
            }

            case VELOCITY_PLANNING_MODE: //����ģʽ
            {
                Velocity_Planning_MODE(&motorRealInfo[i]);
                pid_calc(&MOTOR_PID_RPM[i],motorRealInfo[i].TARGET_RPM,motorRealInfo[i].RPM);
                break;
            }

            case CURRENT_MODE: //����ģʽ(ֱ�Ӹ�����ֵ)
            {
                break;
            }

            case POSITION_CONTROL_MODE://λ��ģʽ
            {
                pid_calc(&MOTOR_PID_POS[i],motorRealInfo[i].TARGET_POS, motorRealInfo[i].REAL_ANGLE);//λ�û�
                pid_calc(&MOTOR_PID_RPM[i], MOTOR_PID_POS[i].output, motorRealInfo[i].RPM);//�ٶȻ�
                break;
            }

            case SPEED_TARQUE_CONTROL_MODE://�ٶ�ת��ģʽ
            {
                pid_calc(&MOTOR_PID_RPM[i],motorRealInfo[i].Velocity_Tarque.Target_Vel,motorRealInfo[i].RPM);	//�ٶȻ�
                MOTOR_PID_RPM[i].output = Max_Value_Limit(MOTOR_PID_RPM[i].output,motorRealInfo[i].Velocity_Tarque.TARGET_TORQUE);	//����ת��ģʽʱ����ֵ

                //�ж��Ƿ��ȡ
                //flag = 1��ȡ�ɹ�
                // if(fabsf(motorRealInfo[i].RPM) <=10)
                // {
                //     motorRealInfo[i].Velocity_Tarque.Cnt++;
                // }
                // else
                // {
                //     motorRealInfo[i].Velocity_Tarque.Cnt = 0;
                // }

                // if(motorRealInfo[i].Velocity_Tarque.Cnt >= 10)
                // {
                //     motorRealInfo[i].Velocity_Tarque.Cnt=0;
                //     motorRealInfo[i].Velocity_Tarque.Flag = 1;
                //     motorRealInfo[i].Motor_Mode= SPEED_CONTROL_MODE;
                //     motorRealInfo[i].TARGET_RPM = 0;
                // }
                break;
            }

            case MOTO_OFF://����ر�
            {
                motorRealInfo[i].TARGET_CURRENT = 0.0f;//������ֵ
                break;
            }

            case HOMEING_MODE://˵ʵ�ڣ����ٶ�ת��ûʲô���ֱ�����ٶ�ת�ؼ���
            {
                Homeing_Mode(&motorRealInfo[i]);	//����У׼ģʽ
                pid_calc(&MOTOR_PID_RPM[i], motorRealInfo[i].TARGET_RPM, motorRealInfo[i].RPM); 	//�ٶȻ�
                MOTOR_PID_RPM[i].output = Max_Value_Limit(MOTOR_PID_RPM[i].output,motorRealInfo[i].HomingMode.TARGET_TORQUE);	//����У׼ģʽ����
            }

            case POSITION_TORQUE_MODE://λ��ת��ģʽ
            {
                pid_calc(&MOTOR_PID_POS[i],motorRealInfo[i].Position_Tarque.Pos, motorRealInfo[i].REAL_ANGLE);//λ�û�
                pid_calc(&MOTOR_PID_RPM[i], MOTOR_PID_POS[i].output, motorRealInfo[i].RPM);//�ٶȻ�
                MOTOR_PID_RPM[i].output = Max_Value_Limit(MOTOR_PID_RPM[i].output,motorRealInfo[i].Position_Tarque.TARGET_TORQUE);//����ת��ģʽʱ����ֵ
                
                if(motorRealInfo[i].CURRENT >= 2 * motorRealInfo[i].Position_Tarque.TARGET_TORQUE)//�ж��Ƿ��ת�������ԣ�
                {
                    motorRealInfo[i].Position_Tarque.Flag = 1;
                }
                else{
                    motorRealInfo[i].Position_Tarque.Flag = 0;
                }
                
               //�ж��Ƿ񵽴�Ŀ��λ��
            //    if(fabsf(motorRealInfo[i].RPM) <=10)
            //    {
            //        motorRealInfo[i].Position_Tarque.Cnt++;
            //    }
            //    else
            //    {
            //        motorRealInfo[i].Position_Tarque.Cnt = 0;
            //    }

            //    if(motorRealInfo[i].Position_Tarque.Cnt>=50)//50ms
            //    {
            //        motorRealInfo[i].Position_Tarque.Cnt = 0;
            //        motorRealInfo[i].Position_Tarque.Flag = 1;
            //    }
                break;
            }

            default: break;
        }
    }

    //���ת������
    for(int i = 0; i < 7; i++)
    {
        if(motorRealInfo[i].Motor_Mode == CURRENT_MODE)//��ֹѡ���ģʽȴ�޷��ж�
        {

        }//����ģʽ�µ��������
        else
        {
            if (motorRealInfo[i].Motor_Type == M_3508)
            {
                /* code */
                motorRealInfo[i].TARGET_CURRENT = MOTOR_PID_RPM[i].output;	//M3508��λ����
            }
            else if (motorRealInfo[i].Motor_Type == M_2006)
            {
                /* code */
                motorRealInfo[i].TARGET_CURRENT = MOTOR_PID_RPM[i].output;  	//M2006��λ����
            }
            else
            {
                motorRealInfo[i].TARGET_CURRENT = 0;
            }
        }
    }

    //ʹ�ܵ�������͵�������
    M3508_Send_Currents();
}


/**
 * @brief ��ֵ���ƺ���
 * @param ����ֵ
 * @param ����ֵ(��ֵ)
 * @return ���ֵ
*/
float Max_Value_Limit(float Value, float Limit)
{
    if(Value > Limit) Value = Limit;
    if(Value < -Limit) Value = -Limit;

    return Value;
}



/**
  * @brief  Homing mode ����ģʽ
  * @param  ����ṹ��
  * @return NULL
 */
void Homeing_Mode(MOTOR_REAL_INFO* RM_MOTOR)
{
    int Sign_Vel;
    RM_MOTOR->HomingMode.flag = 0;

    if (RM_MOTOR->HomingMode.Vel >= 0)
    {
        Sign_Vel = -1.0f;
    }

    //ת�ٸ�ֵ
    RM_MOTOR->TARGET_RPM = RM_MOTOR->HomingMode.Vel;

    if(fabsf(RM_MOTOR->RPM) <= 30)
    {
        RM_MOTOR->HomingMode.cnt++;
    }
    else
    {
        RM_MOTOR->HomingMode.cnt = 0;
    }

    if(RM_MOTOR->HomingMode.cnt >= 50) //500ms
    {
        //������
        RM_MOTOR->HomingMode.cnt = 0;
        RM_MOTOR->REAL_ANGLE=0.0f;
        RM_MOTOR->HomingMode.flag=1;
        RM_MOTOR->Motor_Mode = SPEED_CONTROL_MODE;
        RM_MOTOR->TARGET_RPM = 0;
    }
}



/**
 * @brief �ݶ��ٶȹ滮
 * @param M����ṹ��
 * @return NULL
*/
void Velocity_Planning_MODE(MOTOR_REAL_INFO *M3508_MOTOR)
{
    //static int cnt;//��ʱ��
    float Ssu;   //��·��
    float Sac;   //����·��
    float Sde;   //����·��
    float Sco;   //����·��
    float Aac;   //���ټ��ٶ�
    float Ade;   //���ټ��ٶ�
    float S;     //��ǰ·��

    // �����������������ִ���ٶȹ滮
    if((M3508_MOTOR->Velocity_Planning.Rac > 1) || (M3508_MOTOR->Velocity_Planning.Rac < 0) ||		//����·�̵ı���
       (M3508_MOTOR->Velocity_Planning.Rde > 1) || (M3508_MOTOR->Velocity_Planning.Rde < 0) ||	//����·�̵ı���
       (M3508_MOTOR->Velocity_Planning.Vmax < M3508_MOTOR->Velocity_Planning.Vstart) )			//�����ٶ�<��ʼ���ٶ�
    {
        M3508_MOTOR->TARGET_RPM = 0;  // ���צ���˶�
        return;
    }
    // ����ģʽ
    if(M3508_MOTOR->Velocity_Planning.Pstart == M3508_MOTOR->Velocity_Planning.Pend)	//��ʼλ��=����λ��
    {
        M3508_MOTOR->TARGET_RPM = M3508_MOTOR->Velocity_Planning.Vstart * M3508_MOTOR->Velocity_Planning.Vmax;	//��ʼ���ٶ�*�����ٶ�
        return;
    }

    // ����һЩ����
    Ssu = ABS(M3508_MOTOR->Velocity_Planning.Pend - M3508_MOTOR->Velocity_Planning.Pstart); 	//��·��
    Sac = Ssu * M3508_MOTOR->Velocity_Planning.Rac;		//����·�� =	��·�� * ����·�̵ı���
    Sde = Ssu * M3508_MOTOR->Velocity_Planning.Rde;		//����·�� =	��·�� * ����·�̵ı���
    Sco = Ssu - Sac - Sde;								//����·�� = ��·�� - ����·�� - ����·��
    Aac = (M3508_MOTOR->Velocity_Planning.Vmax * M3508_MOTOR->Velocity_Planning.Vmax - M3508_MOTOR->Velocity_Planning.Vstart * M3508_MOTOR->Velocity_Planning.Vstart) / (2.0f * Sac);	//���ټ��ٶ� (�����ٶ�*�����ٶ� - ��ʼ���ٶ� *��ʼ���ٶ� ) / (2.0f * ����·��)
    Ade = (M3508_MOTOR->Velocity_Planning.Vend * M3508_MOTOR->Velocity_Planning.Vend -   M3508_MOTOR->Velocity_Planning.Vmax *   M3508_MOTOR->Velocity_Planning.Vmax) / (2.0f * Sde);

    // �����쳣���
    if(((M3508_MOTOR->Velocity_Planning.Pend > M3508_MOTOR->Velocity_Planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->Velocity_Planning.Pstart)) ||		//[(����λ�� > ��ʼλ��) && (���������ʵ�Ƕ�pos <��ʼλ��)]	||
       ((M3508_MOTOR->Velocity_Planning.Pend < M3508_MOTOR->Velocity_Planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->Velocity_Planning.Pstart)))		//	[(����λ�� < ��ʼλ��) && (���������ʵ�Ƕ�pos >��ʼλ��)]
    {
        M3508_MOTOR->TARGET_RPM = M3508_MOTOR->Velocity_Planning.Vstart;	//TARGET_RPM = ��ʼ���ٶ�
    }
    else if(((M3508_MOTOR->Velocity_Planning.Pend > M3508_MOTOR->Velocity_Planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->Velocity_Planning.Pend)) ||
            ((M3508_MOTOR->Velocity_Planning.Pend < M3508_MOTOR->Velocity_Planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->Velocity_Planning.Pend)))
    {
        M3508_MOTOR->TARGET_RPM = M3508_MOTOR->Velocity_Planning.Vstart;	//TARGET_RPM = ĩβ���ٶ�
    }
    else
    {
        S = ABS(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->Velocity_Planning.Pstart);      //��ʼλ��

        // �滮RPM
        if     (S < Sac)       M3508_MOTOR->TARGET_RPM = sqrt(2.0 * Aac * S + M3508_MOTOR->Velocity_Planning.Vstart * M3508_MOTOR->Velocity_Planning.Vstart);         // ���ٽ׶�
        else if(S < (Sac+Sco)) M3508_MOTOR->TARGET_RPM = M3508_MOTOR->Velocity_Planning.Vmax;                                                        // ���ٽ׶�
        else                   M3508_MOTOR->TARGET_RPM = sqrt(M3508_MOTOR->Velocity_Planning.Vend * M3508_MOTOR->Velocity_Planning.Vend - 2.0f * Ade * ABS(Ssu - S));  // ���ٽ׶�
    }

    // ������ʵ�������
    if(M3508_MOTOR->Velocity_Planning.Pend < M3508_MOTOR->Velocity_Planning.Pstart) M3508_MOTOR->TARGET_RPM = -M3508_MOTOR->TARGET_RPM;
    //�ж��Ƿ����
    if((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->Velocity_Planning.Pend)) < 3)
    {
        M3508_MOTOR->Velocity_Planning.flag = 1;//���ñ�־λ
        M3508_MOTOR->TARGET_RPM=0;
    }


    if((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->Velocity_Planning.Pend)) > 3)
    {
        M3508_MOTOR->Velocity_Planning.flag = 0;
    }
}



/**
  * @brief  λ�ÿ���(��λ�û�����)
  * @param  target_posĿ��λ��
  * @return
*/
float Position_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO,float target_pos)
{
    MOTO_REAL_INFO->Motor_Mode = POSITION_CONTROL_MODE;
    MOTO_REAL_INFO->TARGET_POS = target_pos;
    if(ABS(MOTO_REAL_INFO->TARGET_POS-target_pos)<1)
        return 1;
    else
        return 0;
}

/**
  * @brief
	* @param ����ṹ��
	* @param  target_torqueĿ��ת�أ��õ�����ʾ
	* @param target_posĿ��λ��
	* @retval none
  */
void Pos_Torque_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, uint16_t Target_Torque, float Target_Pos)
{
    MOTO_REAL_INFO->Motor_Mode = POSITION_TORQUE_MODE;
    MOTO_REAL_INFO->Position_Tarque.Pos = Target_Pos;
    MOTO_REAL_INFO->Position_Tarque.TARGET_TORQUE = Target_Torque;
}

/**
 * @brief �ٶ�ģʽ
 * @param Ŀ��ת��
 * @return NULL
*/
void Speed_Control(MOTOR_REAL_INFO *RM_MOTOR, float Target_RPM)
{
    RM_MOTOR->Motor_Mode = SPEED_CONTROL_MODE;
    RM_MOTOR->TARGET_RPM = Target_RPM;
}

/**
  * @brief  �ٶ�ת�ؿ��ƺ���,������Ҫ�ı�����ת����ôֱ�Ӹı�Target_Vel��ֵ����
  * @param  target_torqueĿ��ת��,�õ�����ʾ���������ͣ�
  * @param target_velĿ��λ�ã�������������ת��
  * @retval none
*/
void Vel_Torque_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, uint16_t Target_Torque, float Target_Vel)
{
    MOTO_REAL_INFO->Motor_Mode = SPEED_TARQUE_CONTROL_MODE;
    MOTO_REAL_INFO->Velocity_Tarque.Target_Vel = Target_Vel;
    MOTO_REAL_INFO->Velocity_Tarque.TARGET_TORQUE = Target_Torque;
}

/**
  * @brief  �����ٶȹ滮�Ĳ����������ٶȹ滮����
  * @param
  * @param float Pstart;        //��ʼλ��
  * @param float Pend;          //����λ��
  * @param float Vstart;        //��ʼ���ٶ�  ��λ��RPM ����ֵ
  * @param float Vmax;          //�����ٶ�
  * @param float Vend;          //ĩβ���ٶ�
  * @param float Rac;           //����·�̵ı���
  * @param float Rde;           //����·�̵ı���
  * @retval NULL
  */
void Velocity_Planning_setpos(MOTOR_REAL_INFO *M3508_MOTOR,float Pstart,float Pend,float Vstart,float Vmax,float Vend,float Rac,float Rde)
{
    M3508_MOTOR->Motor_Mode = VELOCITY_PLANNING_MODE;//����ģʽ
    M3508_MOTOR->Velocity_Planning.Pstart = Pstart;
    M3508_MOTOR->Velocity_Planning.Pend = Pend;
    M3508_MOTOR->Velocity_Planning.Vstart = Vstart;
    M3508_MOTOR->Velocity_Planning.Vmax = Vmax;
    M3508_MOTOR->Velocity_Planning.Vend = Vend;
    M3508_MOTOR->Velocity_Planning.Rac = Rac;
    M3508_MOTOR->Velocity_Planning.Rde = Rde;
    M3508_MOTOR->Velocity_Planning.flag = 0;
}
