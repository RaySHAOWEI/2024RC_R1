//
// Created by Ray on 2023/11/2.
//
/**
 * @file usr_can.c
 * @author Ray
 * @brief �����û�can�˲����Ŀ����������󽮹ٷ��Ķ������ƹ����ģ��ع�3508��Ӧ�ö��˽����
 * @version 0.1
 * @date 2023-11-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "usr_can.h"
#include "upper.h"
#include "chassis.h"

void can_filter_init(void)//�����˲���������������
{
    CAN_FilterTypeDef can1_filter_st;
    CAN_FilterTypeDef can2_filter_st;

    can1_filter_st.FilterActivation = ENABLE;
    can1_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can1_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can1_filter_st.FilterIdHigh = 0x0000;
    can1_filter_st.FilterIdLow = 0x0000;
    can1_filter_st.FilterMaskIdHigh = 0x0000;
    can1_filter_st.FilterMaskIdLow = 0x0000;
    can1_filter_st.FilterBank = 0;
    can1_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    can2_filter_st.FilterActivation = ENABLE;
    can2_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can2_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can2_filter_st.FilterIdHigh = 0x0000;
    can2_filter_st.FilterIdLow = 0x0000;
    can2_filter_st.FilterMaskIdHigh = 0x0000;
    can2_filter_st.FilterMaskIdLow = 0x0000;
    can2_filter_st.FilterBank = 0;
    can2_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    HAL_CAN_ConfigFilter(&hcan1, &can1_filter_st);//��ʼ���˲���
    HAL_CAN_Start(&hcan1);//����can
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//�˲����ж�
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);//�򿪷����ж�

    HAL_CAN_ConfigFilter(&hcan2, &can2_filter_st);//��ʼ���˲���
    HAL_CAN_Start(&hcan2);//����can
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//�˲����ж�
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);//�򿪷����ж�
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef	RxHeader;      //����
    uint8_t	RxData[8];  //���ݽ������飬can������ֻ֡��8֡
    uint8_t	RxData2[8];

    if(hcan == &hcan1)//���̵��can
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
        get_chassis_motor_measure(&RxHeader,RxData);  // M3508������ݴ���
		for(int m=0;m<4;m++)
        {
            RM_MOTOR_Angle_Integral(&ChassisInfo[m]);//�ǶȻ���
        }

    }
    if(hcan == &hcan2)//�ϲ�������
    {
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, RxData2);
        get_motor_measure(&RxHeader,RxData2);  // M3508������ݴ���
        for(int m=0;m<7;m++)
        {
            RM_MOTOR_Angle_Integral(&motorRealInfo[m]);//�ǶȻ���
        }
    }
}
