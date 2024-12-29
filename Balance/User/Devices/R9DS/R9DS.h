#ifndef __R9DS_H
#define __R9DS_H

#include "stm32h7xx_hal.h"

#define USART_REC_LEN 25

typedef struct
{
	uint8_t SBUS_data[USART_REC_LEN];  
}UART_RxBuffer;

typedef struct
{
	uint32_t onlineCheckCnt;
	uint8_t  isOnline;
	
	 struct {       
	 uint16_t ch0;       
	 uint16_t ch1;       
	 uint16_t ch2;      
	 uint16_t ch3;      
	 uint8_t  s1; 
	 uint8_t  s2;	
	 uint8_t  s3;
	 uint8_t  s4;
	 uint16_t sw1;
	 uint16_t sw2;
	 uint16_t sw3;		 
	}rc;
	
}RC_Ctrl;


void DR16_DataUnpack(uint8_t *buf,RC_Ctrl* rc_ctrl);
void DR16_DataUnpack1(uint8_t *buf,RC_Ctrl* rc_ctrl);
void R9DS_callback(RC_Ctrl* rc_ctrl);
void R9DS_Init(RC_Ctrl* rc_ctrl);


extern UART_RxBuffer uart5_RxBuffer;
extern RC_Ctrl rc_Ctrl;
#endif

