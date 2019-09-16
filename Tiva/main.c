/**
 * main.c
 */
#include <stdio.h>
#include "tm4c1294ncpdt.h"
#include "functions.h"
#include "int_handler.h"
#include "inc/hw_memmap.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include <stdbool.h>

#define PWM_PERIOD  16666
#define PWM_MIN  1100
#define PWM_MAX  1900
// speed
int pwmServo = 1600;
volatile unsigned int gucNewData = 0;
volatile unsigned char gucRxChar = 0;
volatile unsigned char left = false;
volatile unsigned char right = false;
volatile unsigned char go = false;
volatile unsigned char stop = false;
volatile unsigned char back = false;

int pwmServoDelta = 10; // increase to go faster
int pwmMotorDelta = 90; // increase to go faster
int i;


// prototypes
void systemConfig();
void SysCtlDelay();
void steerLeft();
void steerRight();
void goBack();

void delay(int s) {
    SysCtlDelay(s * 40000); /*in milliseconds*/
}

int main(void) {

    // configure the whole system
    systemConfig();

    while (1) {
        if (stop) {
            go = false;
            stop = false;
        }
        if (left)
            steerLeft();
        if (right)
            steerRight();
// not working at the moment
//        if(back)
//            goBack();
        while (go && !stop) {
            pwmServo += pwmMotorDelta;
            if ((pwmServo >= PWM_MAX) || (pwmServo <= PWM_MAX)) {
                pwmMotorDelta = -pwmMotorDelta;
            }
            GPIO_PORTM_DATA_R |= 0x02;
            timerWait(pwmServo);
            GPIO_PORTM_DATA_R &= ~0x02;
            timerWait(PWM_PERIOD - pwmServo);
        }

    }
 }
void steerLeft() {
     left = false;
     for(i = 0 ; i < 10 ; i++) {
         GPIO_PORTM_DATA_R &= ~0x01 ;
         timerWait(pwmServoDelta);
         GPIO_PORTM_DATA_R |= 0x01;
         timerWait(PWM_PERIOD - pwmServoDelta);
     }
}

void steerRight() {
    right = false;
    for(i = 0 ; i < 2 ; i++) {
        GPIO_PORTM_DATA_R |= 0x01;
        timerWait(pwmServoDelta);
        GPIO_PORTM_DATA_R &= ~0x01;
        timerWait(PWM_PERIOD - pwmServoDelta);
    }
}

void goBack() {
    back = false;
    for(i = 0 ; i < 50 ; i ++) {
        GPIO_PORTM_DATA_R &= ~0x02;
        timerWait(pwmServo);
        GPIO_PORTM_DATA_R |= 0x02;
        timerWait(PWM_PERIOD - pwmServo);
    }
}
