#include "main.h"
#include "duoji.h"
#include "pwm.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

void pwminit(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	
	
    //
    // For this example PWM0 is used with PortB Pin6.  The actual port and
    // pins used may be different on your part, consult the data sheet for
    // more information.
    // GPIO port B needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Configure the GPIO pin muxing to select PWM00 functions for these pins.
    // This step selects which alternate function is available for these pins.
    // This is necessary if your part supports GPIO pin function muxing.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PF0_M0PWM0);
		GPIOPinConfigure(GPIO_PF1_M0PWM1);
		GPIOPinConfigure(GPIO_PF2_M0PWM2);
		GPIOPinConfigure(GPIO_PF3_M0PWM3);

    //
    // Configure the PWM function for this pin.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
		GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
		GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
		GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
		
		//clk is 1/4m=250khz
PWMClockSet(PWM0_BASE,PWM_SYSCLK_DIV_16);


    //
    // Configure the PWM0 to count up/down without synchronization.
    //
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);
		PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);
										

    //
    // Set the PWM period to 250Hz.  To calculate the appropriate parameter
    // use the following equation: N = (1 / f) * SysClk.  Where N is the           
    // function parameter, f is the desired frequency, and SysClk is the            1/50*1000000
    // system clock frequency.
    // In this case you get: (1 / 250Hz) * 16MHz = 64000 cycles.  Note that
    // the maximum period you can set is 2^16.
    // TODO: modify this calculation to use the clock frequency that you are    16m/64/50=50k
    // using.(1/50)*250k=50000;   0.5ms/20ms=1/40=0.025,   2.5/20=0.125
    //
		
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 20000);
	   PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 20000);

    //
    // Set PWM0 to a duty cycle of 25%.  You set the duty cycle as a function
    // of the period.  Since the period was set above, you can use the
    // PWMGenPeriodGet() function.  For this example the PWM will be high for
    // 25% of the time or 16000 clock ticks (64000 / 4).
    //    default at 1500us
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1500);
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1500);
		  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 1000);
			 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 1000);
                    
										 
  
    //
    // Enable the PWM0 Bit0 (PD0) output signal.
    //
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
		PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
		PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
		PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
		
    //
    // Enable the PWM generator block.
    //
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
		PWMGenEnable(PWM0_BASE, PWM_GEN_1);

}
