#ifndef motorDiver_h
#define motorDiver_h

#include <Wire.h>
#include <Arduino.h>

class motorDriver {
private:
    uint8_t _addr;
public:
    motorDriver(uint8_t addr);
    void sendRPM(int rpm);
    void sendPWM(int pwm);
};

template <typename T> unsigned int I2C_writeAnything (const T& value)
  {
    const byte * p = (const byte*) &value;
    unsigned int i;
    for (i = 0; i < sizeof value; i++)
          Wire.write(*p++);
    return i;
  }  // end of I2C_writeAnything

template <typename T> unsigned int I2C_readAnything(T& value)
  {
    byte * p = (byte*) &value;
    unsigned int i;
    for (i = 0; i < sizeof value; i++)
          *p++ = Wire.read();
    return i;
  }  // end of I2C_readAnything

#endif
