#ifndef __AIR_JOY_H
#define __AIR_JOY_H

#include "main.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#define SWD					PPM_Databuf[7]
#define SWC					PPM_Databuf[6]
#define SWB       			PPM_Databuf[5]
#define SWA      			PPM_Databuf[4]
#define YaoGan_LEFT_X		PPM_Databuf[0]
#define YaoGan_LEFT_Y		PPM_Databuf[1]
#define YaoGan_RIGHT_Y		PPM_Databuf[2]
#define YaoGan_RIGHT_X		PPM_Databuf[3]

#define Hour         3
#define Minute       2
#define Second       1
#define MicroSecond  0

extern uint16_t PPM_Databuf[10];

#endif
