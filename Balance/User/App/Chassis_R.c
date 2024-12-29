/**
  *********************************************************************
  * @file      Chassis_R.c/h
  * @brief     该任务控制右半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can1总线上
	*						 从底盘上往下看，右上角的DM4310发送id为6、接收id为3，
	*						 右下角的DM4310发送id为8、接收id为4，
	*						 右边DM轮毂电机发送id为1、接收id为0。
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "Chassis_R.h"
#include "fdcan.h"
#include "Chassis_L.h"

float LQR_K_R[12]={ 
   -4.4394,   -0.513  , -1.3493,   -1.3052,    2.0990  ,  0.2698,
    3.1791,   0.3891  ,  1.5877  ,  1.4070  , 13.3789 ,   0.6359};

//三次多项式拟合系数		
//float Poly_Coefficient[12][4]=    //效果不错，theta d_theta x d_x phi d_phi%10 1 100 700 4000 1，R=[90 0;0 4];需测试
//{	{151.722556118840,-10.536393350953,-40.787449410235,0.412874340368},
//	{49.745212091512,-23.720093358129,-2.682443677649,0.036241500484},
//	{-69.983683463400,45.258732246968,-10.797643604646,-0.030701672760},
//	{-190.310281294745,125.390146185938,-31.018390124501,-0.127190842510},
//	{287.547259949660,-66.423419058181,-17.894450229159,7.673350255826},
//	{22.767842608457,-9.190645676730,0.375949210100,0.488627296549},
//	{1912.454677045662,-997.349875011845,171.627729955509,2.334531274223},
//	{130.109675523396,-82.782291357938,20.017678321250,0.223059378412},
//	{184.425865574558,-32.298709680187,-16.315316023932,5.385933436738},
//	{498.050229676097,-83.426721532618,-46.926004713833,15.540365520551},
//	{2347.675347545722,-1514.796949024018,364.162613152341,-3.651005403882},
//	{24.703383666778,-40.863202000031,15.997685515901,-1.278528187436}};
	
	float Poly_Coefficient[12][4]=   //效果不错，机体质量修改theta d_theta x d_x phi d_phi%10 1 100 600 4000 1，R=[90 0;0 4];需测试
{	{112.845135511629,5.515604457792,-40.918030753879,0.418677700004},
	{44.290985054785,-20.882221333954,-2.778535762860,0.038115599263},
	{-78.990133493834,48.915985251273,-11.172441668133,-0.042482447001},
	{-200.832850036330,126.449034522089,-29.944482494393,-0.152592753436},
	{253.498745408067,-45.460379113788,-21.991592836902,7.750927333252},
	{22.619197049632,-8.725285037129,0.217815640035,0.486935173898},
	{1809.686118752799,-921.703834442367,152.975799233306,2.590199551292},
	{128.010302395005,-77.758254225739,17.815094580415,0.266897307102},
	{158.203345928741,-16.819621080958,-19.209991620949,5.445672337314},
	{387.130536671912,-33.065649075500,-52.059673407760,14.688608543190},
	{2645.381722616551,-1635.924975708334,376.325762023084,-3.121774065205},
	{37.576314134604,-45.652245635121,16.199071209702,-1.172542938133}};
	


vmc_leg_t right;

chassis_t chassis_move;
	
BasePID_Object Tp_pid;//防劈叉补偿pd
BasePID_Object Turn_pid;//转向pd
BasePID_Object LegR_pid;//右腿的腿长pd
BasePID_Object RollR_Pid;//ROLL轴补偿pid
	
uint32_t CHASSR_TIME=1;	

void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc)
{
	joint_motor_init(&chassis->joint_motor[0],6,MIT_MODE);//发送id为6 A
	joint_motor_init(&chassis->joint_motor[1],8,MIT_MODE);//发送id为8 B
	
	wheel_motor_init(&chassis->wheel_motor[0],1,MIT_MODE);//发送id为1
	
	VMC_init(vmc);//给杆长赋值
	
	BasePID_Init(&Tp_pid,6.0f , 0 ,0.5f, 0);
	BasePID_Init(&Turn_pid,2.0f , 0 ,0.5f, 0);
	BasePID_Init(&LegR_pid,350.0f , 0 ,3000.0f, 0);
//	BasePID_Init(&RollR_Pid,0.0f , 0 ,0, 0);
	BasePID_Init(&RollR_Pid,50.0f , 10 ,0, 0);
	
	for(int j=0;j<50;j++)
	{
	  enable_motor_mode(&hfdcan1,chassis->joint_motor[1].para.id,chassis->joint_motor[1].mode);
	  HAL_Delay(3);
	}
	
	for(int j=0;j<50;j++)
	{
	  enable_motor_mode(&hfdcan1,chassis->joint_motor[0].para.id,chassis->joint_motor[0].mode);
	  HAL_Delay(3);
	}

	for(int j=0;j<50;j++)
	{
    enable_motor_mode(&hfdcan1,chassis->wheel_motor[0].para.id,chassis->wheel_motor[0].mode);//右边轮毂电机
	  HAL_Delay(3);
	}
}


void ChassisR_task(void)
{
	chassisR_feedback_update(&chassis_move,&right,&INS);//更新数据
	
	chassisR_control_loop(&chassis_move,&right,&INS,LQR_K_R,Tp_pid,Turn_pid,LegR_pid,RollR_Pid);//控制计算
}


void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
{
	vmc->phi1=pi/2.0f+chassis->joint_motor[0].para.pos;
	vmc->phi4=pi/2.0f+chassis->joint_motor[1].para.pos;
		
	chassis->myPithR=ins->Pitch;
	chassis->myPithGyroR=ins->Gyro[0];
	
	chassis->total_yaw=ins->YawTotalAngle;
	chassis->roll=ins->Roll;
	chassis->theta_err=0.0f-(vmc->theta+left.theta);
	
	if(ins->Pitch<(3.1415926f/9.0f)&&ins->Pitch>(-3.1415926f/9.0f))
	{//根据pitch角度判断倒地自起是否完成
		chassis->recover_flag=0;
	}
}

uint8_t right_flag=0;
void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,BasePID_Object Tp_pid,BasePID_Object Turn_pid,BasePID_Object LegR_pid,BasePID_Object RollR_Pid)
{
	VMC_calc_1_right(vmcr,ins,((float)CHASSR_TIME)*1.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是1*0.001秒
	
	for(int i=0;i<12;i++)
	{
		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcr->L0 );	
	}
	chassis->turn_T = Tp_pid.Kp * (chassis->turn_set-chassis->total_yaw) - Tp_pid.Kd * ins->Gyro[2];
	chassis->leg_tp = BasePID_PositionControl(&Tp_pid, 0.0f , chassis->theta_err);
	
	chassis->wheel_motor[0].wheel_T = (LQR_K[0]*(vmcr->theta-0.0f)
																	+LQR_K[1]*(vmcr->d_theta-0.0f)
																	+LQR_K[2]*(chassis->x_filter-chassis->x_set)
																	+LQR_K[3]*(chassis->v_filter-chassis->v_set)
																	+LQR_K[4]*(chassis->myPithR-0.0f)
																	+LQR_K[5]*(chassis->myPithGyroR-0.0f));
	
		//右边髋关节输出力矩				
	vmcr->Tp=(LQR_K[6]*(vmcr->theta-0.0f)
					+LQR_K[7]*(vmcr->d_theta-0.0f)
					+LQR_K[8]*(chassis->x_filter-chassis->x_set)
					+LQR_K[9]*(chassis->v_filter-chassis->v_set)
					+LQR_K[10]*(chassis->myPithR-0.0f)
					+LQR_K[11]*(chassis->myPithGyroR-0.0f));
	
	chassis->wheel_motor[0].wheel_T=chassis->wheel_motor[0].wheel_T-chassis->turn_T;	//轮毂电机输出力矩
	mySaturate(&chassis->wheel_motor[0].wheel_T,-2.0f,2.0f);
	
	vmcr->Tp=vmcr->Tp+chassis->leg_tp;//髋关节输出力矩
	
	chassis->now_roll_set = BasePID_PositionControl(&RollR_Pid,chassis->roll_set,chassis->roll);
	
//	vmcr->F0=13.0f+BasePID_PositionControl(&LegR_pid, chassis->leg_set_R , vmcr->L0);//前馈+pd
	vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + BasePID_PositionControl(&LegR_pid, chassis->leg_set_R , vmcr->L0)-chassis->now_roll_set;
	right_flag=ground_detectionR(vmcr,ins);//右腿离地检测
	
	if(chassis->recover_flag==0)		//没倒地
	 {//倒地自起不需要检测是否离地	 
		if(right_flag==1&&left_flag==1&&vmcr->leg_flag==0)
		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
				chassis->wheel_motor[0].wheel_T=0.0f;
				vmcr->Tp=LQR_K[6]*(vmcr->theta-0.0f)+ LQR_K[7]*(vmcr->d_theta-0.0f);

				chassis->x_filter=0.0f;
				chassis->x_set=chassis->x_filter;
				chassis->turn_set=chassis->total_yaw;
				vmcr->Tp=vmcr->Tp+chassis->leg_tp;		
		}
		else
		{//没有离地
			vmcr->leg_flag=0;//置为0
			
		}
	 }
	 else if(chassis->recover_flag==1)
	 {
		 vmcr->Tp=0.0f;
	 }	
	mySaturate(&vmcr->F0,-100.0f,100.0f);//限幅 
	
	VMC_calc_2(vmcr);//计算期望的关节输出力矩
	//额定扭矩
  mySaturate(&vmcr->torque_set[1],-6.0f,6.0f);	
	mySaturate(&vmcr->torque_set[0],-6.0f,6.0f);
}

void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}

//void jump_loop_r(chassis_t *chassis,vmc_leg_t *vmcr,BasePID_Object LegR_pid)
//{
//	if(chassis->jump_flag == 1)
//	{
//		if(chassis->jump_status_r == 0)
//		{
//			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + BasePID_PositionControl(&LegR_pid, 0.07f , vmcr->L0);//前馈+pd
//			if(vmcr->L0<0.1f)
//			{
//				chassis->jump_time_r++;
//			}
//			if(chassis->jump_time_r>=10&&chassis->jump_time_l>=10)
//			{
//				chassis->jump_time_r = 0;
//				chassis->jump_status_r = 1;
//				chassis->jump_time_l = 0;
//				chassis->jump_status_l = 1;
//			}
//		}
//		else if(chassis->jump_status_r == 1)
//		{
//			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,0.4f) ;//前馈+pd
//			if(vmcr->L0>0.16f)
//			{
//				chassis->jump_time_r++;
//			}
//			if(chassis->jump_time_r>=10&&chassis->jump_time_l>=10)
//			{
//				chassis->jump_time_r = 0;
//				chassis->jump_status_r = 2;
//				chassis->jump_time_l = 0;
//				chassis->jump_status_l = 2;
//			}
//		}
//		else if(chassis->jump_status_r == 2)
//		{
//			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) ;//前馈+pd
//			if(vmcr->L0<(chassis->leg_right_set+0.01f))
//			{
//				chassis->jump_time_r++;
//			}
//			if(chassis->jump_time_r>=10&&chassis->jump_time_l>=10)
//			{
//				chassis->jump_time_r = 0;
//				chassis->jump_status_r = 3;
//				chassis->jump_time_l = 0;
//				chassis->jump_status_l = 3;
//			}
//		}
//		else
//		{
//			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) ;//前馈+pd
//		}

//		if(chassis->jump_status_r == 3&&chassis->jump_status_l == 3)
//		{
//			chassis->jump_flag = 0;
//			chassis->jump_time_r = 0;
//			chassis->jump_status_r = 0;
//			chassis->jump_time_l = 0;
//			chassis->jump_status_l = 0;
//		}
//	}
//	else
//	{
//		vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) - chassis->now_roll_set;//前馈+pd
//	}

//}

