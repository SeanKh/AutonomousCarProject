#include "tm4c1294ncpdt.h"
#include "int_handler.h"
#include "functions.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "utils/uartstdio.h"


// pin PL1
void frontSensorTrigger() {
    GPIO_PORTL_DATA_R |= 0x02;
    timerWait(10);
    GPIO_PORTL_DATA_R &= ~0x02;
}

// pin PL3
void backSensorTrigger() {
    GPIO_PORTL_DATA_R |= 0x08;
    timerWait(10);
    GPIO_PORTL_DATA_R &= ~0x08;
}
