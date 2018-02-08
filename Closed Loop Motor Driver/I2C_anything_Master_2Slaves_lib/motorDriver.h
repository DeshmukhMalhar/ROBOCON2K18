#ifndef motorDiver_h
#define motorDiver_h
#include <Wire.h>

class motorDriver {
private:
    uint8_t _addr;
public:
    motorDriver(uint8_t addr);
    void sendRPM(int rpm);
    void sendPWM(int pwm);
};
#endif
