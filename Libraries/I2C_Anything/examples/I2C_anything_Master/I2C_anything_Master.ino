

// Written by Nick Gammon
// May 2012

#include <Wire.h>
#include <I2C_Anything.h>

const byte SLAVE_ADDRESS = 42;

void setup()
{
  Wire.begin ();
}  // end of setup
  int foo ;
void loop()
{


  Wire.beginTransmission (SLAVE_ADDRESS);
  I2C_writeAnything (-393);
  Wire.endTransmission ();
  delay (50);

}  // end of loop
