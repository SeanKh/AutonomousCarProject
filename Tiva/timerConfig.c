#include "tm4c1294ncpdt.h"
#include "driverlib/timer.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "inc/hw_memmap.h"


/*
 * Timer 3A is used for controlling back ultrasonic sensor
 */
void timer3AConfig()
{
    // configure Timer 1
    SYSCTL_RCGCTIMER_R |= (1 << 3);             // timer 3
    while (!(SYSCTL_PRTIMER_R & (1 << 3)))
        ;                                       // wait for timer 3 activation
    TIMER3_CTL_R &= ~0x001;                     // disable Timer A
    TIMER3_CFG_R = 0x04;                        // 2 x 16-bit mode

    TIMER3_TAMR_R |= 0x013;                     // capture, up, match disable
    TIMER3_TAILR_R = 0xFFFF;                    // ILR = 65535 (count interval)
    TIMER3_CTL_R  |= 0x0C;                      // both edge

    TIMER3_ICR_R |= 0x001F;                     // clear all flags
    TIMER3_CTL_R  |= 0x001;                     // enable timer A
}

/*
 * Timer 0A is used for controlling front ultrasonic sensor
 */
void timer0AConfig()
{

//    ROM_TimerConfigure(TIMER0_BASE,
//                       (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME));   // timer0A configured 16 bit time capture
//    ROM_TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES); // both edges
//    ROM_IntEnable(INT_TIMER0A);                                          // interrupt enabled for the timer0A
//    ROM_TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);                   // interrupt setup for the timer0 capture events


    // configure Timer 0
    SYSCTL_RCGCTIMER_R |= (1 << 0);             // timer 0
    while (!(SYSCTL_PRTIMER_R & (1 << 0)))
        ;                                       // wait for timer 0 activation
    TIMER0_CTL_R &= ~0x0001;                    // disable Timer A
    TIMER0_CFG_R = 0x04;                        // 2 x 16-bit mode

    TIMER0_TAMR_R |= 0x013;                     // capture, up, match disable
    TIMER0_TAILR_R = 0xFFFF;                    // ILR = 65535 (count interval)
    TIMER0_CTL_R  |= 0x0C;                      // both edge

    TIMER0_ICR_R |= 0x001F;                     // clear all flags
    TIMER0_CTL_R  |= 0x01;                      // enable timer A
}

/*
 * Timer is used to count to a maximum of 16.6 ms used for controlling
 * the car motors
 */
void timer1AConfig()
{
    // configure Timer 0
    SYSCTL_RCGCTIMER_R |= (1 << 1);             // timer 1
    while (!(SYSCTL_PRTIMER_R & (1 << 1)))
        ;                                       // wait for timer 1 activation
    TIMER1_CTL_R &= ~0x0001;                    // disable Timer A
    TIMER1_CFG_R = 0x04;                        // 2 x 16-bit mode

    // compare mode, down, periodic: TAMR=0x2 -no match  value
    TIMER1_TAMR_R |= 1 << 1;                    //last 8 bits: 0000 0010
                                                //4th bit = counts down, 2nd bit = 0x02 - periodic
}

/*
 * functions waits up to 16ms
 * @param time to wait in micro-seconds
 */
void timerWait(unsigned short usec)
{
    timer1AConfig();
    long pre = 5;                   //ceil((fCPU / pow(2.0, 16)) * 0.01); //prescaler Tmax = 16.6 ms

    TIMER1_TAPR_R = pre - 1;
    TIMER1_TAILR_R = 16 / pre * usec - 1;   //0.5? probably needs also rounding //ceil((fCPU / pre) * (double)usec * pow(10.0, -6)) - 1; //load value
    TIMER1_CTL_R |= 0x0001;                 //start Tmer0A

    while (!(TIMER1_RIS_R & 0x0001))
        ;                                   //flag set when 0x0000 reached(time-out)
    TIMER1_ICR_R |= 0x01;                   //clear TIMER0_RIS_R(0) bit
    TIMER1_CTL_R &= ~0x0001;                //disable the Timer0A

}
