#include "tm4c1294ncpdt.h"
#include "functions.h"
void systemConfig() {

    timer1AConfig();
    timer0AConfig();
    timer3AConfig();

    UARTConfigBluetooth();
    UARTConfigGPS();
    UARTConfigSerial();

    configPortM();
    configPortL();

//    frontSensorTrigger();
//    backSensorTrigger();
}
