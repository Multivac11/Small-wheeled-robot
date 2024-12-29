#include "R9DS_task.h"
#include "r9ds.h"
#include "Chassis_R.h"
#include "Chassis_L.h"
#include "user_lib.h"

uint16_t R9DS_TIME = 1;//R9DS任务周期是1ms

void R9DS_Task(RC_Ctrl* rc_ctrl,chassis_t *chassis)
{
	if(rc_ctrl->isOnline == 1 && chassis->start_flag == 0) 
	{
		//接收到遥控器信号
		chassis->start_flag=1;
		if(chassis->recover_flag==0
			&&((chassis->myPithR<((-3.1415926f)/4.0f)&&chassis->myPithR>((-3.1415926f)/2.0f))
		  ||(chassis->myPithR>(3.1415926f/4.0f)&&chassis->myPithR<(3.1415926f/2.0f))))
		{
		  chassis->recover_flag=1;//需要自起
		}
	}
	else if(rc_ctrl->isOnline == 0 && chassis->start_flag == 1) 
	{
		//接收不到遥控器信号
		chassis->start_flag=0;
		chassis->recover_flag=0;
	}
	
  
	if(chassis->start_flag==1)
	{//启动
		
		chassis->target_v = ((float)(rc_ctrl->rc.ch3-1000))*(-0.0017f);//往前大于0
		slope_following(&chassis->target_v,&chassis->v_set,0.0010f);	//	坡度跟随
		
		chassis->x_set=chassis->x_set+(chassis->v_set)*1.3f*R9DS_TIME/1000;
		
		
//		chassis->v_set = 0;
//		chassis->target_x = chassis->target_x + ((float)(rc_ctrl->rc.ch3-1000))*(-0.0000015f);
//		slope_following(&chassis->target_x,&chassis->x_set,0.001f);
		
		chassis->turn_set=chassis->turn_set+(rc_ctrl->rc.ch0-1000)*(-0.000004f);//往右大于0
	  			
		//腿长变化
		chassis->leg_set = chassis->leg_set+((float)(rc_ctrl->rc.ch2-1000))*(0.0000003f); 
		mySaturate(&chassis->leg_set,0.065f,0.18f);//腿长限幅在0.065m到0.18m之间
//		chassis->roll_target = ((float)(rc_ctrl->rc.ch2-127))*(0.0025f);
		chassis->roll_target = 0;
		slope_following(&chassis->roll_target,&chassis->roll_set,0.0075f);
		
		jump_key(&rc_Ctrl,&chassis_move);
		
		chassis->leg_set_R = chassis->leg_set;
		chassis->leg_set_L = chassis->leg_set;
		
//		mySaturate(&chassis->leg_set_R,0.065f,0.18f);//腿长限幅在0.065m到0.18m之间
//		mySaturate(&chassis->leg_set_L,0.065f,0.18f);
				
		if(fabsf(chassis->last_leg_set-chassis->leg_set)>0.0001f)
		{//遥控器控制腿长在变化
			right.leg_flag=1;	//为1标志着腿长在主动伸缩(不包括自适应伸缩)，根据这个标志可以不进行离地检测，因为当腿长在主动伸缩时，离地检测会误判端为离地了
      left.leg_flag=1;	 			
		}
		chassis->last_leg_set=chassis->leg_set;
		chassis->last_leg_set_R = chassis->leg_set_R;
		chassis->last_leg_set_L = chassis->leg_set_L;
	} 
	else if(chassis->start_flag==0)
	{//关闭
	  chassis->v_set=0.0f;//清零
		chassis->x_filter = 0;
		chassis->v_filter = 0;
		chassis->x_set=chassis->x_filter;//保存
	  chassis->turn_set=chassis->total_yaw;//保存
		
	  chassis->leg_set=0.09f;//原始腿长
		chassis->leg_set_R = chassis->leg_set;
		chassis->leg_set_L = chassis->leg_set;
	}
	
}

void jump_key(RC_Ctrl* rc_ctrl,chassis_t *chassis)
{
	
}





