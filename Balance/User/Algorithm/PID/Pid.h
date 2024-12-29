#ifndef __PID_H_
#define __PID_H_

#include "stm32h7xx_hal.h"

/**
  * @brief  ����PID 
  */
typedef struct 
{	
	float Kp, Ki, Kd;
	float Error;
	float KpPart, KiPart, KdPart;
	float Out;
	float KiPartDetachment;
	float Max_Out;
	float LastError;
	float LastlastError;
}BasePID_Object;


/**
  * @brief  ˫��PID
  */
typedef struct 
{
	BasePID_Object* ShellPID;
	BasePID_Object* CorePID;
}DualPID_Object;
 

/**
  * @brief  		�޷�
  * @param[in]	input		������
  * @param[in]	output	�����
  */
float AmplitudeLimit(float input,float amplitude);


void BasePID_Init(BasePID_Object* base_pid, float kp, float ki, float kd, float detach);
float BasePID_PositionControl(BasePID_Object* base_pid, float target_speed, float feedback_speed);
float BasePID_IncrementalControl(BasePID_Object* base_pid, float target_speed, float feedback_speed);

#endif
