#include <Wire.h>
#include <I2C_Anything.h>
const byte SLAVE_ADDRESS = 42;
void setup()
{
  Wire.begin ();
}  // end of setup
int foo = -250;
void loop()
{
  I2C_writeAnything (foo);
  Wire.endTransmission ();
  delay (200);
}
