//
// Created by Ray on 2023/11/2.
//
/**
 * @file usr_can.c
 * @author Ray
 * @brief 这是用户can滤波器的开启函数，大疆官方的东西复制过来的，控过3508的应该都了解过。
 * @version 0.1
 * @date 2023-11-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "usr_can.h"
#include "upper.h"
#include "chassis.h"

void can_filter_init(void)//设置滤波器接收所有数据
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

    HAL_CAN_ConfigFilter(&hcan1, &can1_filter_st);//初始化滤波器
    HAL_CAN_Start(&hcan1);//开启can
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//滤波器中断
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);//打开发送中断

    HAL_CAN_ConfigFilter(&hcan2, &can2_filter_st);//初始化滤波器
    HAL_CAN_Start(&hcan2);//开启can
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//滤波器中断
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);//打开发送中断
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef	RxHeader;      //接收
    uint8_t	RxData[8];  //数据接收数组，can的数据帧只有8帧
    uint8_t	RxData2[8];

    if(hcan == &hcan1)//底盘电机can
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
        get_chassis_motor_measure(&RxHeader,RxData);  // M3508电机数据处理
		for(int m=0;m<4;m++)
        {
            RM_MOTOR_Angle_Integral(&ChassisInfo[m]);//角度积分
        }

    }
    if(hcan == &hcan2)//上层电机控制
    {
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, RxData2);
        get_motor_measure(&RxHeader,RxData2);  // M3508电机数据处理
        for(int m=0;m<7;m++)
        {
            RM_MOTOR_Angle_Integral(&motorRealInfo[m]);//角度积分
        }
    }
}
