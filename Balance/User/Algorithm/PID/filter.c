#include "filter.h"

struct LowPassFilter_Info LPF_pitch_speed={
  .filter_coefficient=1.0f,
	.last_output=0,
};


float LPFilter(float sampling ,struct LowPassFilter_Info *LPF){
	//һ�׵�ͨ�˲�����p(n) = c��q(n) + (1 - c)��p(n - 1) 
	(*LPF).sampling =sampling;
	
	(*LPF).output=(*LPF).filter_coefficient *(*LPF).sampling +(1-(*LPF).filter_coefficient)*(*LPF).last_output;
	
	(*LPF).last_output =(*LPF).output ;
	
	return (*LPF).output ;
};
