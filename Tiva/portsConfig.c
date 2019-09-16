#include "tm4c1294ncpdt.h"
#include <stdio.h>

// PL4 -> echo (In), PL1 -> trigger (Out) front sensor
// PD4 -> echo (In), PL3 -> trigger (Out) back sensor
void configPortL(void)
{
    SYSCTL_RCGCGPIO_R |=   (1 << 10); // activate port L clock
    while ((SYSCTL_PRGPIO_R & 1 << 10) == 0)
        ;    //wait for port L

    GPIO_PORTL_DEN_R |=  0x1A;          // enable pin 1,3,4,5
    GPIO_PORTL_DIR_R |=  0x0A;          // PL1 out, PL3 out
//    GPIO_PORTL_DIR_R &=  ~0x10;          // PL4 in

    GPIO_PORTL_AFSEL_R |= 0x10;         // PL4 for timer
    GPIO_PORTL_PCTL_R  |=0x00030000;    // PL4 +connected to timer0A

    // ideas is to generate interrupt at falling edges and
    // then read the value of timers accordingly
    GPIO_PORTL_IS_R  &= ~0x10;          // edge-sensitive
    GPIO_PORTL_IBE_R &= ~0x10;          // interrupt generation controlled by IEV
    GPIO_PORTL_IEV_R &= ~0x10;          // detect falling edges
    GPIO_PORTL_ICR_R |=  0x10;          // clear interrupt PORTL(4)
    GPIO_PORTL_IM_R  |=  0x10;          // unmask PORTL(4)

    NVIC_EN1_R |= 1<<(53 - 32);        // enable port L interrupt in NVIC
}

// usage of pins PM0 for servo control and PM1 for motor control
void configPortM(void)
{
    SYSCTL_RCGCGPIO_R |= 0x808;                      // Port M clock ini
    while ((SYSCTL_PRGPIO_R & 0x00000800) == 0)
        ;     // Port M ready ?
    // Digital enable
    GPIO_PORTM_DEN_R = 0x03; //M(1:0) port enable
    // Set direction
    GPIO_PORTM_DIR_R = 0x03;  //(1:0) symbols output of the chip


}

void configPortD(void)
{
    SYSCTL_RCGCGPIO_R |= (1 << 3);                      // Port D clock ini
    while ((SYSCTL_PRGPIO_R & 1 << 3) == 0)
        ;     // Port M ready ?
    // Digital enable
    GPIO_PORTD_AHB_DEN_R |= 0x10;           // enable pin PD4
    GPIO_PORTD_AHB_AFSEL_R |= 0x10;        // PD4 for timer

    GPIO_PORTD_AHB_PCTL_R |= 0x00030000;   // PD4 connected to timer3A

    // idea is to generate interrupt at falling edges and
    // then read the value of timers accordingly
    GPIO_PORTD_AHB_IS_R  &= ~0x10;          // edge-sensitive
    GPIO_PORTD_AHB_IBE_R &= ~0x10;         // interrupt generation controlled by IEV
    GPIO_PORTD_AHB_IEV_R &= ~0x10;         // detect falling edges
    GPIO_PORTD_AHB_ICR_R |=  0x10;          // clear interrupt PORTD(4)
    GPIO_PORTD_AHB_IM_R  |=  0x10;           // unmask PORTD(4)

    NVIC_EN0_R |= 1 << 3 ;                 // enable Port D interrupt in NVIC

}

