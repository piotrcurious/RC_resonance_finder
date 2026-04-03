/*
 * RCLadder.h - Library for 2nd-order RC ladder resistance measurement.
 */

#ifndef RCLadder_h
#define RCLadder_h

#include "Arduino.h"

class RCLadder {
  public:
    RCLadder(int pinOut, int pinIn, float r1, float c1, float c2, float r0 = 250.0);

    // Core Measurement
    float measureVpp(float freq);
    float solveR2(float freq, float vpp);

    // Filtering
    void updateKalman(float measurement);
    float getEstimate();
    float getConfidence();

    // Utilities
    void safeDelayMicros(unsigned long us);

  private:
    int _pinOut, _pinIn;
    float _r1, _c1, _c2, _r0;

    // Kalman State
    float _x[2]; // {R2, dR2}
    float _P[2][2];
    float _Q[2][2];
    float _R;
};

#endif
