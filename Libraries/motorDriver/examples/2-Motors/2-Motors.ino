#define SLAVE1_ADDRESS 0x51
#define SLAVE2_ADDRESS 0x52

#include <Wire.h>
#include <motorDriver.h>

motorDriver m1(SLAVE1_ADDRESS);
motorDriver m2(SLAVE2_ADDRESS);

void setup()
{
  Wire.begin ();
} 

void loop()
{
  m1.sendPWM( 133);
  delay(100);
  m2.sendPWM(-133);
  delay(100);
} 
