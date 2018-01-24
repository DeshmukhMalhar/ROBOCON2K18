#ifndef motorDiver_h
#include <Wire.h>
#include "I2C_Anything.h"
#define motorDiver_h
class motorDriver {
private:
    uint8_t _addr;
public:
    motorDriver(uint8_t addr)
    void sendRPM(int rpm);
    void sendPWM(int pwm);
}
#endif
