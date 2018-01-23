

// Written by Nick Gammon
// May 2012

#include <Wire.h>
#include <I2C_Anything.h>

const byte SLAVE_ADDRESS = 0x50;

void setup()
{
  Wire.begin ();
}  // end of setup
int foo ;
void loop()
{

  sendRPM(SLAVE_ADDRESS ,140);

  delay (200);

}
void sendRPM(uint8_t addr, int rpm) {
  Wire.beginTransmission (addr);
  I2C_writeAnything (rpm);
  Wire.endTransmission ();
}

