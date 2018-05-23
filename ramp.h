#ifndef __RAMP_H__
#define __RAMP_H__

#include <stdint.h>
#include <stdbool.h>
#include "hw_ints.h"
#include "hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include <string.h>
#include "driverlib/systick.h"
#include "driverlib/pwm.h"
#include "pwm.h"
#include "buttons.h"

typedef struct
{
  uint32_t  count;
	uint32_t  scale;
	float  out;
	int calcflag;


}ramp;
float rampcalc(ramp  ram,int scale);
void rampresetcounter(ramp ram);
void rampsetscale(ramp ram);

#endif
