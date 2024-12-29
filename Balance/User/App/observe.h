#ifndef __OBSERVE_H
#define __OBSERVE_H

#include "observe.h"
#include "stdint.h"
#include "INS.h"
#include "Chassis_L.h"
#include "main.h"


extern void Observe_task(void);
extern void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);
extern void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel);

extern KalmanFilter_t vaEstimateKF;	


#endif
