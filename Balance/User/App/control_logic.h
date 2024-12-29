#ifndef __CONTROL_LOGIC_H_
#define __CONTROL_LOGIC_H_

#include "stm32h7xx_hal.h"
#include "Chassis_R.h"
#include "r9ds.h"

typedef struct {
	uint32_t tim14_clk;
	
}Clock;


void Init_Finish(void);


extern Clock task_clk;

#endif
