//
// Created by Ray on 2024/1/21.
//

#include "usr_uart.h"


uint8_t BUFF[30];

void sent_data(int16_t A,int16_t B,int16_t C,int16_t D)
{
	int i;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t _cnt=0;
	BUFF[_cnt++]=0xAA;//??
	BUFF[_cnt++]=0xFF;//????
	BUFF[_cnt++]=0XF1;//???
	BUFF[_cnt++]=0x08;//????
	BUFF[_cnt++]=BYTE0(A);//????,?????????
	BUFF[_cnt++]=BYTE1(A);//?????????????????????
	BUFF[_cnt++]=BYTE0(B);
	BUFF[_cnt++]=BYTE1(B);	
	BUFF[_cnt++]=BYTE0(C);
    BUFF[_cnt++]=BYTE1(C);
    BUFF[_cnt++]=BYTE0(D);
    BUFF[_cnt++]=BYTE1(D);
	//SC?AC????????????????
	for(i=0;i<BUFF[3]+4;i++) 
	{
		sumcheck+=BUFF[i];
		addcheck+=sumcheck;
	}
	BUFF[_cnt++]=sumcheck;	
	BUFF[_cnt++]=addcheck;	
	
	for(i=0;i<_cnt;i++) HAL_UART_Transmit(&huart1,BUFF,_cnt,100);//??????
}
