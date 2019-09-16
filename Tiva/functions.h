
#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

//configurations
void systemConfig(void);

// timers configurations
void timer1AConfig(void);
void timer0AConfig(void);
void timer3AConfig(void);
void timerWait(unsigned short usec);

// ports configurations
void configPortM(void);
void configPortD(void);
void configPortL(void);

// UART configurations
void UARTConfigBluetooth(void);
void UARTConfigGPS(void);
void UARTConfigSerial(void);

// ISR
void commandRecieved(void);
void GPSPosition(void);
void timer0AIntHandler(void);
void timer3AIntHandler(void);

void frontSensorTrigger(void);
void backSensorTrigger(void);


void clock_change(void);
#endif
