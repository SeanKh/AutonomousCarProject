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


// work on PK0 as U4Rx
void UARTConfigBluetooth(void) {
	SYSCTL_RCGCGPIO_R |= (1 << 0);              // port a
	while (!(SYSCTL_PRGPIO_R & (1 << 0)))
	      ;
	GPIO_PORTA_AHB_DEN_R |= 0x0C;                   // enable pin PA2,PA3
	GPIO_PORTA_AHB_AFSEL_R |= 0x0C;                 // enable alternate function
	GPIO_PORTA_AHB_PCTL_R |= 0x00001100;                  // enable U4Rx

    SYSCTL_RCGCGPIO_R |= (1 << 9);              // port k
    while (!(SYSCTL_PRGPIO_R & (1 << 9)))
        ;
    GPIO_PORTK_DEN_R |= 0x03;                   // enable pin PK0, Pk1
    GPIO_PORTK_AFSEL_R |= 0x03;                 // enable alternate function
    GPIO_PORTK_PCTL_R |= 0x00000011;                  // enable U4Rx


	SYSCTL_RCGCUART_R |= (1 << 4);              // UART 4
	while (!(SYSCTL_PRUART_R & (1 << 4)))
		;
	UART4_CTL_R &= ~0x01;                       // disable the UART4
    //	UART4_IBRD_R = 17;                      // baud rate of 57600 bit/s
    //	UART4_FBRD_R = 23;
	//  BRD = 104.16666667
	UART4_IBRD_R = 104;                         // baud rate of 9600 bit/s
	UART4_FBRD_R = 10;
	UART4_LCRH_R = 0x60;                        // 8N1    bits frame and no parity bit
	UART4_CTL_R |= 0x201;                       // enable the UART4 as Rx
	UART4_ICR_R = 0xE7FF;                       // clear all interrupts
	UART4_IM_R |= 1 << 4; 	                    // activate UART4 Rx interrupts
	NVIC_EN1_R |= 1<< (57-32);                  // enable UART2 interrupt in the nested vector interrupt controller
}

//PP0 -> RX , PP1 -> TX
void UARTConfigGPS(void) {
    SYSCTL_RCGCGPIO_R |= (1 << 13);              // port P
    while (!(SYSCTL_PRGPIO_R & (1 << 13)))
        ;
    GPIO_PORTP_DEN_R |= 0x03;                    // enable pin PP0, PP1
    GPIO_PORTP_AFSEL_R |= 0x03;                  // enable alternate function
    GPIO_PORTP_PCTL_R |= 0x01;                   // enable U6Rx

    SYSCTL_RCGCUART_R |= (1 << 6);               // UART 6
    while (!(SYSCTL_PRUART_R & (1 << 6)))
        ;
    UART6_CTL_R &= ~0x01;                        // disable the UART6
    //  UART4_IBRD_R = 17;                       // baud rate of 57600 bit/s
    //  UART4_FBRD_R = 23;
    //  BRD = 104.16666667
    UART6_IBRD_R = 104;                          // baud rate of 9600 bit/s
    UART6_FBRD_R = 10;
    UART6_LCRH_R = 0x60;                         // 8N1    bits frame and no parity bit
    UART6_CTL_R |= 0x201;                        // enable the UART6 as Rx
    UART6_ICR_R = 0xE7FF;                        // clear all interrupts
    UART6_IM_R |= 1 << 4;                        // activate UART6 Rx interrupts
    NVIC_EN1_R |= 1 << (59-32);                  // enable UART6 interrupt in the nested vector interrupt controller
}

// uart0 configuration -> pc serial
void UARTConfigSerial(void) {

    // Enable the GPIO Peripheral used by the UART.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART for console I/O.
    // GPS module works with baud rate 9600.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 9600, 16000000);
}

// switch the the 25MHZ clock for more precision
void clock_change(void){

        // switch over to main quartz oscillator at 25MHz
        // clear MOSC power down, high oscillator range setting, and no crystal present setting
        SYSCTL_MOSCCTL_R &= ~(SYSCTL_MOSCCTL_OSCRNG | SYSCTL_MOSCCTL_PWRDN |SYSCTL_MOSCCTL_NOXTAL);
        SYSCTL_MOSCCTL_R |= SYSCTL_MOSCCTL_OSCRNG;                             // increase the drive strength for MOSC
        SYSCTL_RSCLKCFG_R = SYSCTL_RSCLKCFG_OSCSRC_MOSC;                    // set the main oscillator as main clock source
}
