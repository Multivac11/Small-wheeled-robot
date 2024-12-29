#include "hardware_config.h"
#include "control_logic.h"
#include "can_bsp.h"
#include "dm4310_drv.h"
#include "arm_math.h"
#include "r9ds.h"
#include "bsp_dwt.h"
#include "gpio.h"
#include "BMI088Middleware.h"
#include "BMI088driver.h"
#include "spi.h"
#include "tim.h"
#include "pid.h"
#include "Chassis_R.h"
#include "Chassis_L.h"
#include "observe.h"

Clock task_clk;

void HardwareConfig(void)
{		
	
	DWT_Init(480);
	
	/* BMI088��ʼ�� *///֮ǰ�Ѿ��Խ��ٶȺͼ��ٶȵ���ƮУ׼���ˣ�����֮���ϵ�Ͳ���ҪУ׼�����Ӳ���豸����������Ҫ����У׼��
  while (BMI088_init(&hspi2, 0) != BMI088_NO_ERROR)
	{
	  ;
	}
	Power_OUT1_ON;
	Power_OUT2_ON;
	Power_OUT3_ON;
	HAL_Delay(200);
	
	FDCAN1_Config();
	FDCAN2_Config();
	
	INS_Init();
	ChassisR_init(&chassis_move,&right);
	HAL_Delay(100);
	ChassisL_init(&chassis_move,&left);
	HAL_Delay(100);
	xvEstimateKF_Init(&vaEstimateKF);
	
	
	while(INS.ins_flag==0)
	{
		INS_task();
	}
	
	R9DS_Init(&rc_Ctrl);
	HAL_TIM_Base_Start_IT(&htim14);

}
