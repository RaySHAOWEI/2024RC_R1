/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "upper.h"
#include "chassis.h"
#include "pid.h"
#include "rm_motor.h"
#include "usr_can.h"
#include "air_joy.h"
#include "FSM.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotorCtrl */
osThreadId_t MotorCtrlHandle;
const osThreadAttr_t MotorCtrl_attributes = {
  .name = "MotorCtrl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for safe_printf */
osMutexId_t safe_printfHandle;
const osMutexAttr_t safe_printf_attributes = {
  .name = "safe_printf"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// void debug_safe_printf(const char *format, ...)
// {
// #if DEBUG_PRINTF_ENABLE
  
//   HAL_UART_MspInit(&huart5);

//   osStatus xReturn;
//   va_list args;
//   va_start(args,format);

//   xReturn = osMutexAcquire(safe_printfHandle,portMAX_DELAY);
//   if(xReturn == osOK)
//   {
//     vprintf(format, args);
//   }
//   xReturn = osMutexRelease(safe_printfHandle);

// #else
//   HAL_UART_MspDeInit(&huart5);
//   (void)0;
// #endif
// }
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void MotorCtrlFunction(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of safe_printf */
  safe_printfHandle = osMutexNew(&safe_printf_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MotorCtrl */
  MotorCtrlHandle = osThreadNew(MotorCtrlFunction, NULL, &MotorCtrl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_MotorCtrlFunction */
/**
* @brief Function implementing the MotorCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorCtrlFunction */
void MotorCtrlFunction(void *argument)
{
  /* USER CODE BEGIN MotorCtrlFunction */
	can_filter_init();
  R1_config();
	// ROBOT_CHASSIS_INIT();
	// chassis_motor_init();
	// Upper_INIT();

  /* Infinite loop */
  for(;;)
  {
//	  lift_motor(lift2top);
//	  belt_ctrl(100);
     fsm();
//    Homeing_Mode(&motorRealInfo[Motor_CLAW], 100, 512);
//        do
//        {
            // Homeing_Mode(&motorRealInfo[Motor_CLAW], 150, 8192);
            // Motor_Control();
//      if (motorRealInfo[Motor_CLAW].HomingMode.done_flag == 0){
//        Homeing_Mode(&motorRealInfo[Motor_CLAW], 1000, 8192);
//		Motor_Control();
//      }
//        } while (motorRealInfo[Motor_CLAW].HomingMode.done_flag != 1);
        // Vel_Torque_Control(&motorRealInfo[Motor_CLAW], , float Target_Vel)
//	  claw_motor(-270);
//	  if (SWD > 1500)
//    {
//      if(ABS(YaoGan_RIGHT_X - 1500) > 5){
//          float claw_pos = motorRealInfo[3].REAL_ANGLE;
//          claw_motor(claw_pos + ((YaoGan_RIGHT_X-1500)/10));
//      }
//      else if(ABS(YaoGan_RIGHT_X - 1500) <= 5){
//          claw_hold();
//      }
//    }
//	  claw_hold();
	  // free_ctrl();
    // chassis_kinematic();
//	belt_ctrl(8192);
//    claw_motor((SWB - 1500) / 3);
//    Motor_Control();
//	M3508_Send_Currents();
    osDelay(1);
  }
  /* USER CODE END MotorCtrlFunction */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

