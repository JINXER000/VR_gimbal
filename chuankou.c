
#include "chuankou.h"
#include "main.h"

//*****************************************************************************
//接线
//USB-TTL工具                 board                JY61
//VCC          -----           VCC        ----        VCC
//TX           -----           U0RX     
//RX           -----           U0TX
//GND          -----           GND        ----        GND
//                             U3RX(PA4)        ----        TX
//														 U3TX (PA5)      ----        RX
//															U6RX(PP0)  --------      tx
//															U6TX(PP1)									RX
//****************************************************************************
extern uint32_t g_ui32SysClock;
unsigned char  imudata,headdata;
int numofinst;
extern int workstate;

void uartinit(void)
{
	 // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    //
    // Enable the GPIO pins for the LED (PN0).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

    //
    // Enable the peripherals used by this example.
    //
    
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
	
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
	
    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    //
    // Set GPIO A0 and A1 as UART0 pins.    A4，A5--U3
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
		
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
		
		GPIOPinConfigure(GPIO_PA4_U3RX);
    GPIOPinConfigure(GPIO_PA5_U3TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4);
		
		GPIOPinConfigure(GPIO_PP0_U6RX);
    GPIOPinConfigure(GPIO_PP1_U6TX);
    ROM_GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    ROM_UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
														 
		ROM_UARTConfigSetExpClk(UART3_BASE, g_ui32SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));	
		ROM_UARTConfigSetExpClk(UART6_BASE, g_ui32SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));															 

    //
    // Enable the UART interrupt.
    //
    ROM_IntEnable(INT_UART0);
		 ROM_IntEnable(INT_UART3);
		  ROM_IntEnable(INT_UART6);
		 
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
		 ROM_UARTIntEnable(UART3_BASE, UART_INT_RX );
		  ROM_UARTIntEnable(UART6_BASE, UART_INT_RX );

    //
    // Prompt for text to be entered.
    //
    UARTSend((uint8_t *)"\033[2JEnter text: ", 16);

    //
    // Loop forever echoing data through the UART.
    //


}

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        
       
       if(workstate==2)

			{
			
			instructionsget(ROM_UARTCharGet(UART0_BASE));
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);

        
        SysCtlDelay(g_ui32SysClock / (1000 * 3));

        
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
			}                            
			
    }
}


void U3Handler(void)
{
	
	  uint32_t ui32Status;
    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART3_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART3_BASE, ui32Status);
	
		while(ROM_UARTCharsAvail(UART3_BASE)){
		imudata=ROM_UARTCharGet(UART3_BASE);
		
		CopeSerial2Data(imudata);//处理数据
		}
		
		
	
}

void U6Handler(void)
{
	
	  uint32_t ui32Status;
    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART6_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART6_BASE, ui32Status);
	
		while(ROM_UARTCharsAvail(UART6_BASE)){
		headdata=ROM_UARTCharGet(UART6_BASE);

		headprocess(headdata);//处理数据
		}
}
//		
//		
//	
//}
//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}
