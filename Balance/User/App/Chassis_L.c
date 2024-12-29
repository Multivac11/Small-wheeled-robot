/**
  *********************************************************************
  * @file      Chassis_L.c/h
  * @brief     该任务控制左半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can2总线上
	*						 从底盘上往下看，左上角的DM4310发送id为8、接收id为4，
	*						 左下角的DM4310发送id为6、接收id为3，
	*						 左边DM轮毂电机发送id为1、接收id为0。
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "Chassis_L.h"
#include "fdcan.h"
#include "VMC_calc.h"
#include "INS.h"


vmc_leg_t left;
BasePID_Object LegL_pid;//左腿的腿长pd
uint32_t CHASSL_TIME=1;	

float LQR_K_L[12]={ 
   -4.4394,   -0.513  , -1.3493,   -1.3052,    2.0990  ,  0.2698,
    3.1791,   0.3891  ,  1.5877  ,  1.4070  , 13.3789 ,   0.6359};

void ChassisL_init(chassis_t *chassis,vmc_leg_t *vmc)
{
	BasePID_Init(&LegL_pid,350.0 , 0 ,3000.0f, 0);
	joint_motor_init(&chassis->joint_motor[2],6,MIT_MODE);//发送id为6
	joint_motor_init(&chassis->joint_motor[3],8,MIT_MODE);//发送id为8
	
	wheel_motor_init(&chassis->wheel_motor[1],1,MIT_MODE);//发送id为1
	
	VMC_init(vmc);//给杆长赋值

	for(int j=0;j<50;j++)
	{
	  enable_motor_mode(&hfdcan2,chassis->joint_motor[3].para.id,chassis->joint_motor[3].mode);
	  HAL_Delay(3);
	}
	for(int j=0;j<50;j++)
	{
	  enable_motor_mode(&hfdcan2,chassis->joint_motor[2].para.id,chassis->joint_motor[2].mode);
	  HAL_Delay(3);
	}
	for(int j=0;j<50;j++)
	{
    enable_motor_mode(&hfdcan2,chassis->wheel_motor[1].para.id,chassis->wheel_motor[1].mode);//左边轮毂电机
	  HAL_Delay(3);
	}	
}	
		
void ChassisL_task(void)
{
	chassisL_feedback_update(&chassis_move,&left,&INS);//更新数据
	
	chassisL_control_loop(&chassis_move,&left,&INS,LQR_K_L,LegL_pid);//控制计算
}

void chassisL_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
{
  vmc->phi1=pi/2.0f+chassis->joint_motor[2].para.pos;
	vmc->phi4=pi/2.0f+chassis->joint_motor[3].para.pos;
		
	chassis->myPithL=0.0f-ins->Pitch;
	chassis->myPithGyroL=0.0f-ins->Gyro[0];
}

uint8_t left_flag=0;
void chassisL_control_loop(chassis_t *chassis,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K,BasePID_Object LegL_pid)
{
	VMC_calc_1_left(vmcl,ins,((float)CHASSL_TIME)*1.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是1*0.001秒
	
		for(int i=0;i<12;i++)
	{
		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcl->L0 );	
	}
	
	chassis->wheel_motor[1].wheel_T=(LQR_K[0]*(vmcl->theta-0.0f)
																	+LQR_K[1]*(vmcl->d_theta-0.0f)
																	+LQR_K[2]*(chassis->x_set-chassis->x_filter)
																	+LQR_K[3]*(chassis->v_set-chassis->v_filter)
																	+LQR_K[4]*(chassis->myPithL-0.0f)
																	+LQR_K[5]*(chassis->myPithGyroL-0.0f));
	
	//右边髋关节输出力矩				
	vmcl->Tp=(LQR_K[6]*(vmcl->theta-0.0f)
					+LQR_K[7]*(vmcl->d_theta-0.0f)
					+LQR_K[8]*(chassis->x_set-chassis->x_filter)
					+LQR_K[9]*(chassis->v_set-chassis->v_filter)
					+LQR_K[10]*(chassis->myPithL-0.0f)
					+LQR_K[11]*(chassis->myPithGyroL-0.0f));
	
	chassis->wheel_motor[1].wheel_T= chassis->wheel_motor[1].wheel_T-chassis->turn_T;	//轮毂电机输出力矩
	mySaturate(&chassis->wheel_motor[1].wheel_T,-2.0f,2.0f);	
	
	vmcl->Tp=vmcl->Tp+chassis->leg_tp;//髋关节输出力矩

//	vmcl->F0=Mg+BasePID_PositionControl(&LegL_pid, chassis->leg_set_L , vmcl->L0);
	vmcl->F0=Mg/arm_cos_f32(vmcl->theta)+BasePID_PositionControl(&LegL_pid, chassis->leg_set_L , vmcl->L0)+chassis->now_roll_set ;//前馈+pd
	
	left_flag=ground_detectionL(vmcl,ins);//左腿离地检测
	
	 if(chassis->recover_flag==0)	
	 {//倒地自起不需要检测是否离地
		if(left_flag==1&&right_flag==1&&vmcl->leg_flag==0)
		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
			chassis->wheel_motor[1].wheel_T=0.0f;
			vmcl->Tp=LQR_K[6]*(vmcl->theta-0.0f)+ LQR_K[7]*(vmcl->d_theta-0.0f);
			
			chassis->x_filter=0.0f;//对位移清零
			chassis->x_set=chassis->x_filter;
			chassis->turn_set=chassis->total_yaw;
			vmcl->Tp=vmcl->Tp+chassis->leg_tp;		
		}
		else
		{//没有离地
			vmcl->leg_flag=0;//置为0			
		}
	 }
	 else if(chassis->recover_flag==1)
	 {
		 vmcl->Tp=0.0f;
	 }
	
	mySaturate(&vmcl->F0,-100.0f,100.0f);//限幅 
	
	VMC_calc_2(vmcl);//计算期望的关节输出力矩
	
  //额定扭矩
  mySaturate(&vmcl->torque_set[1],-6.0f,6.0f);	
	mySaturate(&vmcl->torque_set[0],-6.0f,6.0f);
}

