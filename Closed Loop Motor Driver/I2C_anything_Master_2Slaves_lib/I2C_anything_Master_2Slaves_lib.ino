// Written by Nick Gammon
// May 2012
#define SLAVE1_ADDRESS 0x51
#define SLAVE2_ADDRESS 0x52

#include <Wire.h>

#include "motorDriver.h"

motorDriver m1(SLAVE1_ADDRESS);
motorDriver m2(SLAVE2_ADDRESS);
void setup()
{
  Wire.begin ();
}  // end of setup
int foo ;
void loop()
{
  m1.sendPWM( 133);
  delay(100);
  m2.sendPWM(-133);
  delay(100);
  //  Wire.beginTransmission (SLAVE1_ADDRESS);
  //  I2C_writeAnything (193);
  //  Wire.endTransmission ();
  //  delay (100);
  //  Wire.beginTransmission (SLAVE2_ADDRESS);
  //  I2C_writeAnything (-93);
  //  Wire.endTransmission ();
  //  delay (100);
  //  sendPWM(SLAVE1_ADDRESS, 133);
  //  delay(100);
  //  sendPWM(SLAVE2_ADDRESS, -133);
  //  delay(100);
}  // end of loop
