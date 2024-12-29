/**
  ******************************************************************************
  * @file	 R9DS.c
  * @author  Wang Tianyu
  * @version V1.0.0
  * @date    2024/5/25
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "r9ds.h"
#include "usart.h"
#include "dma.h"
#include "main.h"

UART_RxBuffer uart5_RxBuffer __attribute__((at(0x24018020)));;
RC_Ctrl rc_Ctrl;

uint8_t tmp_flag ;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART5)
	{	
		R9DS_callback(&rc_Ctrl);
  }
	
}

void R9DS_Init(RC_Ctrl* rc_ctrl)
{
	HAL_UART_Receive_DMA(&huart5, uart5_RxBuffer.SBUS_data,USART_REC_LEN);
	rc_ctrl->rc.ch0=1000;
	rc_ctrl->rc.ch1=1000;
	rc_ctrl->rc.ch2=1000;
	rc_ctrl->rc.ch3=1000;
	rc_ctrl->rc.s1=1;
	rc_ctrl->rc.s2=1;	
	rc_ctrl->rc.s3=2;
	rc_ctrl->rc.sw1 = 200;
	rc_ctrl->rc.sw2 = 200;
	rc_ctrl->rc.sw3 = 200;
}

void R9DS_callback(RC_Ctrl* rc_ctrl)
{
	tmp_flag  = __HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE);
	
		if(tmp_flag  != RESET)
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart5);//清除标志位
			HAL_UART_DMAStop(&huart5); //停止DMA接收
			if(uart5_RxBuffer.SBUS_data[1]==0x0F)
			{

				DR16_DataUnpack(uart5_RxBuffer.SBUS_data,&rc_Ctrl);
				
			}
//			else if(uart5_RxBuffer.SBUS_data[0]==0x0F)
//			{
//				DR16_DataUnpack1(uart5_RxBuffer.SBUS_data,&rc_Ctrl);
//			}
			HAL_UART_Receive_DMA(&huart5, uart5_RxBuffer.SBUS_data, USART_REC_LEN);  //发生接收中断后，中断使能位会清除，故此处再调用此函数使能接收中断  
		}
}
 
 
uint16_t SBUS_thoroughfare[16] = {0};
void DR16_DataUnpack(uint8_t *buf,RC_Ctrl* rc_ctrl)	//将8*22=176字节重新放入到SBUS_thoroughfare通道数据中
{
		if(buf[24]==0x00) //遥控器离线保护
			rc_ctrl->isOnline = 1;
		else
			rc_ctrl->isOnline = 0;
	
		SBUS_thoroughfare[ 0] = ((int16_t)buf[ 2] >> 0 | ((int16_t)buf[ 3] << 8 )) & 0x07FF;
		SBUS_thoroughfare[ 1] = ((int16_t)buf[ 3] >> 3 | ((int16_t)buf[ 4] << 5 )) & 0x07FF;
		SBUS_thoroughfare[ 2] = ((int16_t)buf[ 4] >> 6 | ((int16_t)buf[ 5] << 2 )  | (int16_t)buf[ 5] << 10 ) & 0x07FF;
		SBUS_thoroughfare[ 3] = ((int16_t)buf[ 5] >> 1 | ((int16_t)buf[ 7] << 7 )) & 0x07FF;
		SBUS_thoroughfare[ 4] = ((int16_t)buf[ 7] >> 4 | ((int16_t)buf[ 8] << 4 )) & 0x07FF;
		SBUS_thoroughfare[ 5] = ((int16_t)buf[ 8] >> 7 | ((int16_t)buf[ 9] << 1 )  | (int16_t)buf[9] <<  9 ) & 0x07FF;
		SBUS_thoroughfare[ 6] = ((int16_t)buf[10] >> 2 | ((int16_t)buf[11] << 6 )) & 0x07FF;
		SBUS_thoroughfare[ 7] = ((int16_t)buf[11] >> 5 | ((int16_t)buf[12] << 3 )) & 0x07FF;
	
		SBUS_thoroughfare[ 8] = ((int16_t)buf[13] << 0 | ((int16_t)buf[14] << 8 )) & 0x07FF;
		SBUS_thoroughfare[ 9] = ((int16_t)buf[14] >> 3 | ((int16_t)buf[15] << 5 )) & 0x07FF;
		SBUS_thoroughfare[10] = ((int16_t)buf[15] >> 6 | ((int16_t)buf[16] << 2 )  | (int16_t)buf[18] << 10 ) & 0x07FF;
		SBUS_thoroughfare[11] = ((int16_t)buf[17] >> 1 | ((int16_t)buf[18] << 7 )) & 0x07FF;
		SBUS_thoroughfare[12] = ((int16_t)buf[18] >> 4 | ((int16_t)buf[19] << 4 )) & 0x07FF;
		SBUS_thoroughfare[13] = ((int16_t)buf[19] >> 7 | ((int16_t)buf[20] << 1 )  | (int16_t)buf[20] <<  9 ) & 0x07FF;
		SBUS_thoroughfare[14] = ((int16_t)buf[21] >> 2 | ((int16_t)buf[22] << 6 )) & 0x07FF;
		SBUS_thoroughfare[15] = ((int16_t)buf[22] >> 5 | ((int16_t)buf[23] << 3 )) & 0x07FF;
	
		rc_ctrl->rc.ch0 = SBUS_thoroughfare[ 3]-20;
		rc_ctrl->rc.ch1 = SBUS_thoroughfare[ 2];
		rc_ctrl->rc.ch2 = SBUS_thoroughfare[ 0];
		rc_ctrl->rc.ch3 = SBUS_thoroughfare[ 1];
		rc_ctrl->rc.sw1 = SBUS_thoroughfare[ 5];
		rc_ctrl->rc.sw2 = SBUS_thoroughfare[ 7];
		rc_ctrl->rc.sw3 = SBUS_thoroughfare[ 6];
	
		if((rc_ctrl->rc.ch0>996)&&(rc_ctrl->rc.ch0<1004))          //遥控器零飘
			rc_ctrl->rc.ch0=1000;
		if((rc_ctrl->rc.ch1>996)&&(rc_ctrl->rc.ch1<1004))
			rc_ctrl->rc.ch1=1000;
		if((rc_ctrl->rc.ch2>994)&&(rc_ctrl->rc.ch2<1002))
			rc_ctrl->rc.ch2=1000;
		if((rc_ctrl->rc.ch3>994)&&(rc_ctrl->rc.ch3<1002))
			rc_ctrl->rc.ch3=1000;
	
		if(SBUS_thoroughfare[ 9]<500)
			rc_ctrl->rc.s1 = 1;
		else
			rc_ctrl->rc.s1 = 2;
		if(SBUS_thoroughfare[ 8]<500)
			rc_ctrl->rc.s2 = 1;
		else
			rc_ctrl->rc.s2 = 2;
		if(SBUS_thoroughfare[ 4]<500)
			rc_ctrl->rc.s3 = 1;
		else if(SBUS_thoroughfare[ 4]<1200 && SBUS_thoroughfare[ 4]>600)
			rc_ctrl->rc.s3 = 2;
		else
			rc_ctrl->rc.s3 = 3;
	
}

 void DR16_DataUnpack1(uint8_t *buf,RC_Ctrl* rc_ctrl)	//将8*22=176字节重新放入到SBUS_thoroughfare通道数据中
{
		if(buf[23]==0x00) //遥控器离线保护
			rc_ctrl->isOnline = 1;
		else
			rc_ctrl->isOnline = 0;
	
		SBUS_thoroughfare[ 0] = ((int16_t)buf[ 1] >> 0 | ((int16_t)buf[ 2] << 8 )) & 0x07FF;
		SBUS_thoroughfare[ 1] = ((int16_t)buf[ 2] >> 3 | ((int16_t)buf[ 3] << 5 )) & 0x07FF;
		SBUS_thoroughfare[ 2] = ((int16_t)buf[ 3] >> 6 | ((int16_t)buf[ 4] << 2 )  | (int16_t)buf[ 4] << 10 ) & 0x07FF;
		SBUS_thoroughfare[ 3] = ((int16_t)buf[ 4] >> 1 | ((int16_t)buf[ 6] << 7 )) & 0x07FF;
		SBUS_thoroughfare[ 4] = ((int16_t)buf[ 6] >> 4 | ((int16_t)buf[ 7] << 4 )) & 0x07FF;
		SBUS_thoroughfare[ 5] = ((int16_t)buf[ 7] >> 7 | ((int16_t)buf[ 8] << 1 )  | (int16_t)buf[8] <<  9 ) & 0x07FF;
		SBUS_thoroughfare[ 6] = ((int16_t)buf[9] >> 2 | ((int16_t)buf[10] << 6 )) & 0x07FF;
		SBUS_thoroughfare[ 7] = ((int16_t)buf[10] >> 5 | ((int16_t)buf[11] << 3 )) & 0x07FF;
	
		SBUS_thoroughfare[ 8] = ((int16_t)buf[12] << 0 | ((int16_t)buf[13] << 8 )) & 0x07FF;
		SBUS_thoroughfare[ 9] = ((int16_t)buf[13] >> 3 | ((int16_t)buf[14] << 5 )) & 0x07FF;
		SBUS_thoroughfare[10] = ((int16_t)buf[14] >> 6 | ((int16_t)buf[15] << 2 )  | (int16_t)buf[17] << 10 ) & 0x07FF;
		SBUS_thoroughfare[11] = ((int16_t)buf[16] >> 1 | ((int16_t)buf[17] << 7 )) & 0x07FF;
		SBUS_thoroughfare[12] = ((int16_t)buf[17] >> 4 | ((int16_t)buf[18] << 4 )) & 0x07FF;
		SBUS_thoroughfare[13] = ((int16_t)buf[18] >> 7 | ((int16_t)buf[19] << 1 )  | (int16_t)buf[19] <<  9 ) & 0x07FF;
		SBUS_thoroughfare[14] = ((int16_t)buf[20] >> 2 | ((int16_t)buf[21] << 6 )) & 0x07FF;
		SBUS_thoroughfare[15] = ((int16_t)buf[21] >> 5 | ((int16_t)buf[22] << 3 )) & 0x07FF;
	
		rc_ctrl->rc.ch0 = SBUS_thoroughfare[ 3]-20;
		rc_ctrl->rc.ch1 = SBUS_thoroughfare[ 2];
		rc_ctrl->rc.ch2 = SBUS_thoroughfare[ 0];
		rc_ctrl->rc.ch3 = SBUS_thoroughfare[ 1];
		rc_ctrl->rc.sw1 = SBUS_thoroughfare[ 5];
		rc_ctrl->rc.sw2 = SBUS_thoroughfare[ 7];
		rc_ctrl->rc.sw3 = SBUS_thoroughfare[ 6];
	
		if((rc_ctrl->rc.ch0>996)&&(rc_ctrl->rc.ch0<1004))          //遥控器零飘
			rc_ctrl->rc.ch0=1000;
		if((rc_ctrl->rc.ch1>996)&&(rc_ctrl->rc.ch1<1004))
			rc_ctrl->rc.ch1=1000;
		if((rc_ctrl->rc.ch2>994)&&(rc_ctrl->rc.ch2<1002))
			rc_ctrl->rc.ch2=1000;
		if((rc_ctrl->rc.ch3>994)&&(rc_ctrl->rc.ch3<1002))
			rc_ctrl->rc.ch3=1000;
	
		if(SBUS_thoroughfare[ 9]<500)
			rc_ctrl->rc.s1 = 1;
		else
			rc_ctrl->rc.s1 = 2;
		if(SBUS_thoroughfare[ 8]<500)
			rc_ctrl->rc.s2 = 1;
		else
			rc_ctrl->rc.s2 = 2;
		if(SBUS_thoroughfare[ 4]<500)
			rc_ctrl->rc.s3 = 1;
		else if(SBUS_thoroughfare[ 4]<1200 && SBUS_thoroughfare[ 4]>600)
			rc_ctrl->rc.s3 = 2;
		else
			rc_ctrl->rc.s3 = 3;
	
}


