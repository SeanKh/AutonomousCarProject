#include "int_handler.h"
#include "tm4c1294ncpdt.h"
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"

void timerWait(unsigned short);
// back sensor PD4
void timer3AIntHandler(void) {
	GPIO_PORTD_AHB_ICR_R |= 0x04;         // clear interrupt source
	unsigned long ulVal = TIMER3_TAR_R;

//    UARTCharPut(UART0_BASE, (unsigned short)(0xffff-ulVal)/16);
    unsigned short x = (0xffff-ulVal)/16;
    x *=2;

}

