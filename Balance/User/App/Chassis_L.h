#ifndef __CHASSIS_L_H
#define __CHASSIS_L_H


#include "Chassis_L.h"
#include "Chassis_R.h"
#include "VMC_calc.h"
#include "pid.h"


extern vmc_leg_t left;
extern uint8_t left_flag;

extern void ChassisL_init(chassis_t *chassis,vmc_leg_t *vmc);
extern void ChassisL_task(void);
extern void chassisL_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins);
extern void chassisL_control_loop(chassis_t *chassis,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K,BasePID_Object LegL_pid);

#endif
