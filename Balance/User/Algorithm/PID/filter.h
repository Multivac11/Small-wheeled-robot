#ifndef __FILTER_H__
#define __FILTER_H__

#include "main.h"

struct LowPassFilter_Info{	
	float filter_coefficient;
	float last_output;
	float output;
	float sampling;	
};

float LPFilter(float sampling ,struct LowPassFilter_Info *LPF);

extern struct LowPassFilter_Info LPF_pitch_speed;

#endif
