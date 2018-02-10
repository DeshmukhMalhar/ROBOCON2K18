#include <Wire.h>
#include "motorDriver.h"

motorDriver::motorDriver(uint8_t addr){
	Wire.begin();
    _addr=addr;
}

void motorDriver::sendRPM(int rpm) {
    Wire.beginTransmission (_addr);
    I2C_writeAnything (rpm);
    Wire.endTransmission ();
  }
void motorDriver::sendPWM(int pwm) {
    Wire.beginTransmission (_addr);
    I2C_writeAnything (pwm);
    Wire.endTransmission ();
  }