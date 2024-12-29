#ifndef __CHASSIS_R_H
#define __CHASSIS_R_H

#include "Chassis_R.h"
#include "stm32h7xx_hal.h"
#include "dm4310_drv.h"
#include "main.h"
#include "pid.h"
#include "VMC_calc.h"
#include "INS.h"

#define Mg 14.0f

typedef struct
{
  Joint_Motor_t joint_motor[4];
  Wheel_Motor_t wheel_motor[2];
	
	float v_set;//�����ٶȣ���λ��m/s
	float target_v;
	float x_set;//����λ�ã���λ��m
	float target_x;
	float turn_set;//����yaw�ỡ��
	float leg_set;//�����ȳ�����λ��m
	float leg_set_R;
	float leg_set_L;
	float last_leg_set;
	float last_leg_set_R;
	float last_leg_set_L;

	float v_filter;//�˲���ĳ����ٶȣ���λ��m/s
	float x_filter;//�˲���ĳ���λ�ã���λ��m
	
	float myPithR;
	float myPithGyroR;
	float myPithL;
	float myPithGyroL;
	float roll;
	float roll_set;
	float roll_target;
	float now_roll_set;
	float total_yaw;
	float theta_err;//���ȼн����
		
	float turn_T;//yaw�Ჹ��
	float leg_tp;//�����油��
	
	uint8_t start_flag;//������־
	
	uint8_t recover_flag;//һ������µĵ��������־
	
	uint32_t count_key;
	uint8_t jump_flag;
	float jump_leg;
	uint32_t jump_time_r;
	uint32_t jump_time_l;
	uint8_t jump_status_r;
	uint8_t jump_status_l;	

	
} chassis_t;



extern void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc);
extern void ChassisR_task(void);
extern void mySaturate(float *in,float min,float max);
extern void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins);
extern void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,BasePID_Object Tp_pid,BasePID_Object Turn_pid,BasePID_Object LegR_pid,BasePID_Object RollR_Pid);

extern chassis_t chassis_move;
extern vmc_leg_t right;
extern uint8_t right_flag;
extern float Poly_Coefficient[12][4];

#endif

