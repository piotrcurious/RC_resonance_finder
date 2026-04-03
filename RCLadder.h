#ifndef RCLadder_h
#define RCLadder_h
#include "Arduino.h"
class RCLadder {
  public:
    RCLadder(int pinOut, int pinIn, float r1, float c1, float c2, float r0 = 250.0);
    float measureVpp(float freq);
    float solveR2(float freq, float vpp);
    float findCrossingMicros(float freq, int threshold);
    void updateKalman(float measurement);
    float getEstimate();
    float getConfidence();
    void safeDelayMicros(unsigned long us);
  private:
    int _pinOut, _pinIn;
    float _r1, _c1, _c2, _r0;
    float _x[2], _P[2][2], _Q[2][2], _R;
};
#endif
