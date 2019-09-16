#include "int_handler.h"
#include "tm4c1294ncpdt.h"
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include <stdbool.h>

void timerWait(unsigned short);
// front sensor PL4
void timer0AIntHandler(void) {
    //bool printed = true;
    GPIO_PORTL_ICR_R |= 0x04;               // clear interrupt source
    unsigned long ulVal = TIMER0_TAR_R;
//    if(UARTBusy(UART0_BASE)){ // true
//    while(UARTBusy(UART0_BASE));
//    UARTCharPut(UART0_BASE, (unsigned short) (0xffff - ulVal) / 16);

    unsigned short x = (0xffff-ulVal)/16;
        x *=2;
}

