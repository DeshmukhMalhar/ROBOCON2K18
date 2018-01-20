#ifndef AUTOPID_H
#define AUTOPID_H
#include <Arduino.h>

class AutoPID {
  private:
  
    double _Kp, _Ki, _Kd;
    double _integral, _previousError;

    long *_input, *_setpoint;
    uint8_t _outputMin, _outputMax, *_output;
    unsigned long _timeStep, _lastStep;

  public:
     AutoPID(long *input, long *setpoint, uint8_t *output, uint8_t outputMin, uint8_t outputMax, double Kp, double Ki, double Kd);
     void setGains(double Kp, double Ki, double Kd);
     void setOutputRange(uint8_t outputMin,uint8_t outputMax);
     void setTimeStep(unsigned long timeStep);
     void run();
     void reset();


};//class AutoPID

#endif
