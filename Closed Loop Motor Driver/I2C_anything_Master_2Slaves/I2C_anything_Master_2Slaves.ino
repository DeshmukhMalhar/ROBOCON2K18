// Written by Nick Gammon
// May 2012

#include <Wire.h>
#include <I2C_Anything.h>

const byte SLAVE1_ADDRESS = 0x51;
const byte SLAVE2_ADDRESS = 0x52;

void setup()
{
  Wire.begin ();
}  // end of setup
  int foo ;
void loop()
{


  Wire.beginTransmission (SLAVE1_ADDRESS);
  I2C_writeAnything (193);
  Wire.endTransmission ();
  delay (100);
  Wire.beginTransmission (SLAVE2_ADDRESS);
  I2C_writeAnything (-93);
  Wire.endTransmission ();
  delay (100);

}  // end of loop
