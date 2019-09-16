#include "int_handler.h"
#include "tm4c1294ncpdt.h"
#include "functions.h"
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include <stdbool.h>

//PP0 -> RX , PP1 -> TX
// GPS module works with baud rate 9600.
void GPSPosition(void)
{
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART6_BASE, true);       //get interrupt status
    UARTIntClear(UART6_BASE, ui32Status);               //clear the asserted interrupts
    gucRxChar = UARTCharGet(UART6_BASE);

    // prints at serial monitor
    UARTCharPut(UART0_BASE, gucRxChar);

    // prints at mobile
    UARTCharPut(UART4_BASE, gucRxChar);

}
