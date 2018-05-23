#ifndef __MAIN_H__
#define __MAIN_H__

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

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

int main(void);
void CopeSerial2Data(unsigned char ucData);
void instructionsget(unsigned char insData);
void selfadjust(void);
int getworkstate(void);
void controltask (void);
void headprocess(unsigned char ucData);
void passorders(void);
int hextodec(unsigned char insBuf);


#endif 
