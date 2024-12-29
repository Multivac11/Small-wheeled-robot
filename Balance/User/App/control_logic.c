#include "control_logic.h"
#include "tim.h"
#include "hardware_config.h"
#include "Ins.h"
#include "Chassis_R.h"
#include "Chassis_L.h"
#include "r9ds.h"
#include "observe.h"
#include "R9DS_task.h"

/**
  * @brief  中断主任务
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim14)
	{
		task_clk.tim14_clk++;
		
		if(INS.ins_flag==1)
		{//等待加速度收敛
			
			INS_task();
			ChassisL_task();
			ChassisR_task();
			R9DS_Task(&rc_Ctrl,&chassis_move);
			
			if(task_clk.tim14_clk % 3 == 0)
			{
				
				Observe_task();
			}
		}
		
		if(chassis_move.start_flag==1)
		{
			//右腿
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,right.torque_set[1]);
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f,right.torque_set[0]);
			mit_ctrl2(&hfdcan1,chassis_move.wheel_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f, chassis_move.wheel_motor[0].wheel_T);
			//左腿
			mit_ctrl(&hfdcan2,chassis_move.joint_motor[3].para.id, 0.0f, 0.0f,0.0f, 0.0f,left.torque_set[1]);
			mit_ctrl(&hfdcan2,chassis_move.joint_motor[2].para.id, 0.0f, 0.0f,0.0f, 0.0f,left.torque_set[0]);
			mit_ctrl2(&hfdcan2,chassis_move.wheel_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,chassis_move.wheel_motor[1].wheel_T);
		}
		else if(chassis_move.start_flag==0)
		{
			//右腿
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);
			mit_ctrl2(&hfdcan1,chassis_move.wheel_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);
			//左腿
			mit_ctrl(&hfdcan2,chassis_move.joint_motor[3].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);
			mit_ctrl(&hfdcan2,chassis_move.joint_motor[2].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);
			mit_ctrl2(&hfdcan2,chassis_move.wheel_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);
		}
	}
}


