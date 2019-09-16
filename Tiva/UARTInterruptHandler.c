#include "int_handler.h"
#include "tm4c1294ncpdt.h"
#include "functions.h"
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include <stdbool.h>

// PM0 servo, PM1 motor
void commandRecieved(void)
{
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART4_BASE, true);       //get interrupt status
    UARTIntClear(UART4_BASE, ui32Status);               //clear the asserted interrupts
    gucRxChar = UARTCharGet(UART4_BASE);

    gucNewData = gucRxChar == 'g' ? go = true :
                     gucRxChar == 's' ? stop = true :
                         gucRxChar == 'r' ? right = true :
                             gucRxChar == 'l' ? left = true :
                                  gucRxChar == 'b' ? back = true :   30;

}
