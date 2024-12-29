#include "pid.h"
#include "filter.h"

/**
  * @brief  �޷�
  */
float AmplitudeLimit(float input,float amplitude)
{
	if(input<-amplitude)
		return -amplitude;
	else if(input>amplitude)
		return amplitude;
	else return input;
}


/**
  * @brief ����PID��ʼ��
  */
void BasePID_Init(BasePID_Object* base_pid, float kp, float ki, float kd, float detach )
{
	base_pid->KiPartDetachment = detach;
	
	base_pid->Kp = kp;
	base_pid->Ki = ki;
	base_pid->Kd = kd;
	
	base_pid->KpPart = 0;
	base_pid->KiPart = 0;
	base_pid->KdPart = 0;
	
}


/**
  * @brief λ��ʽpid
  */
float BasePID_PositionControl(BasePID_Object* base_pid, float target_speed, float feedback_speed)
{	
	base_pid->Error = target_speed - feedback_speed;
	
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	
	if(base_pid->Error > base_pid->KiPartDetachment)
	{
		base_pid->KiPart = 0;
	}
	else if(base_pid->Error < -(base_pid->KiPartDetachment))
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (base_pid->Error - base_pid->LastError)*base_pid->KdPart;
	
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart ;
	
	base_pid->LastError = base_pid->Error;	
	base_pid->LastlastError = base_pid->LastError;
	

	return base_pid->Out;	
}


/**
  * @brief ����ʽpid
  */
float BasePID_IncrementalControl(BasePID_Object* base_pid, float target_speed, float feedback_speed)
{	
	base_pid->Error = target_speed - feedback_speed;
	
	base_pid->KpPart = (base_pid->Error - base_pid->LastError) * base_pid->Kp;
	base_pid->KiPart = base_pid->Error * base_pid->Ki;
	base_pid->KdPart = (base_pid->Error - 2.0f*base_pid->LastError + base_pid->LastlastError)*base_pid->KdPart;
	
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart ;
	
	base_pid->LastError = base_pid->Error;	
	base_pid->LastlastError = base_pid->LastError;
	
	return base_pid->Out;	
}
