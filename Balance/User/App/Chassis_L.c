/**
  *********************************************************************
  * @file      Chassis_L.c/h
  * @brief     �����������벿�ֵĵ�����ֱ�������DM4310��һ��DM6215�����������������can2������
	*						 �ӵ��������¿������Ͻǵ�DM4310����idΪ8������idΪ4��
	*						 ���½ǵ�DM4310����idΪ6������idΪ3��
	*						 ���DM��챵������idΪ1������idΪ0��
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
BasePID_Object LegL_pid;//���ȵ��ȳ�pd
uint32_t CHASSL_TIME=1;	

float LQR_K_L[12]={ 
   -4.4394,   -0.513  , -1.3493,   -1.3052,    2.0990  ,  0.2698,
    3.1791,   0.3891  ,  1.5877  ,  1.4070  , 13.3789 ,   0.6359};

void ChassisL_init(chassis_t *chassis,vmc_leg_t *vmc)
{
	BasePID_Init(&LegL_pid,350.0 , 0 ,3000.0f, 0);
	joint_motor_init(&chassis->joint_motor[2],6,MIT_MODE);//����idΪ6
	joint_motor_init(&chassis->joint_motor[3],8,MIT_MODE);//����idΪ8
	
	wheel_motor_init(&chassis->wheel_motor[1],1,MIT_MODE);//����idΪ1
	
	VMC_init(vmc);//���˳���ֵ

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
    enable_motor_mode(&hfdcan2,chassis->wheel_motor[1].para.id,chassis->wheel_motor[1].mode);//�����챵��
	  HAL_Delay(3);
	}	
}	
		
void ChassisL_task(void)
{
	chassisL_feedback_update(&chassis_move,&left,&INS);//��������
	
	chassisL_control_loop(&chassis_move,&left,&INS,LQR_K_L,LegL_pid);//���Ƽ���
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
	VMC_calc_1_left(vmcl,ins,((float)CHASSL_TIME)*1.0f/1000.0f);//����theta��d_theta��lqr�ã�ͬʱҲ�������ȳ�L0,���������������1*0.001��
	
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
	
	//�ұ��Źؽ��������				
	vmcl->Tp=(LQR_K[6]*(vmcl->theta-0.0f)
					+LQR_K[7]*(vmcl->d_theta-0.0f)
					+LQR_K[8]*(chassis->x_set-chassis->x_filter)
					+LQR_K[9]*(chassis->v_set-chassis->v_filter)
					+LQR_K[10]*(chassis->myPithL-0.0f)
					+LQR_K[11]*(chassis->myPithGyroL-0.0f));
	
	chassis->wheel_motor[1].wheel_T= chassis->wheel_motor[1].wheel_T-chassis->turn_T;	//��챵���������
	mySaturate(&chassis->wheel_motor[1].wheel_T,-2.0f,2.0f);	
	
	vmcl->Tp=vmcl->Tp+chassis->leg_tp;//�Źؽ��������

//	vmcl->F0=Mg+BasePID_PositionControl(&LegL_pid, chassis->leg_set_L , vmcl->L0);
	vmcl->F0=Mg/arm_cos_f32(vmcl->theta)+BasePID_PositionControl(&LegL_pid, chassis->leg_set_L , vmcl->L0)+chassis->now_roll_set ;//ǰ��+pd
	
	left_flag=ground_detectionL(vmcl,ins);//������ؼ��
	
	 if(chassis->recover_flag==0)	
	 {//����������Ҫ����Ƿ����
		if(left_flag==1&&right_flag==1&&vmcl->leg_flag==0)
		{//������ͬʱ��ز���ң����û���ڿ����ȵ�����ʱ������Ϊ���
			chassis->wheel_motor[1].wheel_T=0.0f;
			vmcl->Tp=LQR_K[6]*(vmcl->theta-0.0f)+ LQR_K[7]*(vmcl->d_theta-0.0f);
			
			chassis->x_filter=0.0f;//��λ������
			chassis->x_set=chassis->x_filter;
			chassis->turn_set=chassis->total_yaw;
			vmcl->Tp=vmcl->Tp+chassis->leg_tp;		
		}
		else
		{//û�����
			vmcl->leg_flag=0;//��Ϊ0			
		}
	 }
	 else if(chassis->recover_flag==1)
	 {
		 vmcl->Tp=0.0f;
	 }
	
	mySaturate(&vmcl->F0,-100.0f,100.0f);//�޷� 
	
	VMC_calc_2(vmcl);//���������Ĺؽ��������
	
  //�Ť��
  mySaturate(&vmcl->torque_set[1],-6.0f,6.0f);	
	mySaturate(&vmcl->torque_set[0],-6.0f,6.0f);
}

