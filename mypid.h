#ifndef __MYPID_H__
#define __MYPID_H__

#include "main.h"

typedef struct
{
	float kp,ki,kd;
	float refposition;
	float curposition;
	float err;
	float errlast;
	float output;
	float integral;
	float max;
	float min;

}pidregulator;

void pidinit(pidregulator*  pdd);
float pidrealize(pidregulator*  pdd,float aim);

#endif
