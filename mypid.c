
#include "mypid.h"

void pidinit(pidregulator*  pdd)
{
	pdd->refposition=0;
	pdd->curposition=0;
	pdd->err=0;
	pdd->errlast=0;
	pdd->output=0;
	pdd->integral=0;
	pdd->kp=0.5;
	pdd->ki=0;
	pdd->kd=0;
	pdd->max=90;
	pdd->min=-90;

}

float pidrealize(pidregulator*  pdd,float aim)
{
		pdd->refposition=aim;
		pdd->err=pdd->refposition-pdd->curposition;
		pdd->integral+=pdd->err;
	VAL_LIMIT(pdd->integral,-30,30);
		pdd->output=pdd->kp*pdd->err+pdd->ki*pdd->integral+pdd->kd*(pdd->err-pdd->errlast);
		pdd->errlast=pdd->err;
		pdd->curposition+=pdd->output;
		VAL_LIMIT(pdd->curposition,pdd->min,pdd->max);
		return pdd->curposition;
}

