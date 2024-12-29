#include "R9DS_task.h"
#include "r9ds.h"
#include "Chassis_R.h"
#include "Chassis_L.h"
#include "user_lib.h"

uint16_t R9DS_TIME = 1;//R9DS����������1ms

void R9DS_Task(RC_Ctrl* rc_ctrl,chassis_t *chassis)
{
	if(rc_ctrl->isOnline == 1 && chassis->start_flag == 0) 
	{
		//���յ�ң�����ź�
		chassis->start_flag=1;
		if(chassis->recover_flag==0
			&&((chassis->myPithR<((-3.1415926f)/4.0f)&&chassis->myPithR>((-3.1415926f)/2.0f))
		  ||(chassis->myPithR>(3.1415926f/4.0f)&&chassis->myPithR<(3.1415926f/2.0f))))
		{
		  chassis->recover_flag=1;//��Ҫ����
		}
	}
	else if(rc_ctrl->isOnline == 0 && chassis->start_flag == 1) 
	{
		//���ղ���ң�����ź�
		chassis->start_flag=0;
		chassis->recover_flag=0;
	}
	
  
	if(chassis->start_flag==1)
	{//����
		
		chassis->target_v = ((float)(rc_ctrl->rc.ch3-1000))*(-0.0017f);//��ǰ����0
		slope_following(&chassis->target_v,&chassis->v_set,0.0010f);	//	�¶ȸ���
		
		chassis->x_set=chassis->x_set+(chassis->v_set)*1.3f*R9DS_TIME/1000;
		
		
//		chassis->v_set = 0;
//		chassis->target_x = chassis->target_x + ((float)(rc_ctrl->rc.ch3-1000))*(-0.0000015f);
//		slope_following(&chassis->target_x,&chassis->x_set,0.001f);
		
		chassis->turn_set=chassis->turn_set+(rc_ctrl->rc.ch0-1000)*(-0.000004f);//���Ҵ���0
	  			
		//�ȳ��仯
		chassis->leg_set = chassis->leg_set+((float)(rc_ctrl->rc.ch2-1000))*(0.0000003f); 
		mySaturate(&chassis->leg_set,0.065f,0.18f);//�ȳ��޷���0.065m��0.18m֮��
//		chassis->roll_target = ((float)(rc_ctrl->rc.ch2-127))*(0.0025f);
		chassis->roll_target = 0;
		slope_following(&chassis->roll_target,&chassis->roll_set,0.0075f);
		
		jump_key(&rc_Ctrl,&chassis_move);
		
		chassis->leg_set_R = chassis->leg_set;
		chassis->leg_set_L = chassis->leg_set;
		
//		mySaturate(&chassis->leg_set_R,0.065f,0.18f);//�ȳ��޷���0.065m��0.18m֮��
//		mySaturate(&chassis->leg_set_L,0.065f,0.18f);
				
		if(fabsf(chassis->last_leg_set-chassis->leg_set)>0.0001f)
		{//ң���������ȳ��ڱ仯
			right.leg_flag=1;	//Ϊ1��־���ȳ�����������(����������Ӧ����)�����������־���Բ�������ؼ�⣬��Ϊ���ȳ�����������ʱ����ؼ������ж�Ϊ�����
      left.leg_flag=1;	 			
		}
		chassis->last_leg_set=chassis->leg_set;
		chassis->last_leg_set_R = chassis->leg_set_R;
		chassis->last_leg_set_L = chassis->leg_set_L;
	} 
	else if(chassis->start_flag==0)
	{//�ر�
	  chassis->v_set=0.0f;//����
		chassis->x_filter = 0;
		chassis->v_filter = 0;
		chassis->x_set=chassis->x_filter;//����
	  chassis->turn_set=chassis->total_yaw;//����
		
	  chassis->leg_set=0.09f;//ԭʼ�ȳ�
		chassis->leg_set_R = chassis->leg_set;
		chassis->leg_set_L = chassis->leg_set;
	}
	
}

void jump_key(RC_Ctrl* rc_ctrl,chassis_t *chassis)
{
	
}





