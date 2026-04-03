/*
 * RCLadder.cpp - Library for 2nd-order RC ladder resistance measurement.
 */

#include "RCLadder.h"
#include <math.h>

RCLadder::RCLadder(int pinOut, int pinIn, float r1, float c1, float c2, float r0) {
  _pinOut = pinOut;
  _pinIn = pinIn;
  _r1 = r1;
  _c1 = c1;
  _c2 = c2;
  _r0 = r0;

  pinMode(_pinOut, OUTPUT);
  pinMode(_pinIn, INPUT);

  // Initialize Kalman
  _x[0] = 1000.0; _x[1] = 0.0;
  _P[0][0] = 1e6; _P[0][1] = 0; _P[1][0] = 0; _P[1][1] = 1e3;
  _Q[0][0] = 10.0; _Q[0][1] = 0; _Q[1][0] = 0; _Q[1][1] = 1.0;
  _R = 500.0;
}

void RCLadder::safeDelayMicros(unsigned long us) {
  if (us > 16000) { delay(us/1000); delayMicroseconds(us%1000); }
  else delayMicroseconds(us);
}

float RCLadder::measureVpp(float freq) {
  if (freq < 0.01) freq = 0.01;
  unsigned long period = 1000000.0 / freq;
  unsigned long half = period / 2;

  float vSumMax = 0, vSumMin = 0;
  int count = 0;
  unsigned long window = 300; if (period > 150000) window = (period * 2.0) / 1000;

  unsigned long start = millis();
  while(millis() - start < window) {
    digitalWrite(_pinOut, HIGH); safeDelayMicros(half);
    vSumMax += (analogRead(_pinIn) * 5.0) / 1023.0;
    digitalWrite(_pinOut, LOW); safeDelayMicros(half);
    vSumMin += (analogRead(_pinIn) * 5.0) / 1023.0;
    count++;
    if (millis() - start > 5000) break;
  }
  return (count > 0) ? (vSumMax - vSumMin) / count : 0;
}

float RCLadder::solveR2(float freq, float vpp) {
  float ratio = vpp / 5.0;
  if (ratio >= 0.99) return 1.0;
  float artanh_val = 0.5 * log((1.0 + ratio) / (1.0 - ratio));
  float tau = 1.0 / (4.0 * freq * artanh_val);
  float ra = _r0 + _r1;
  float r2 = (tau - ra * (_c1 + _c2)) / _c2;
  return (r2 < 0.1) ? 0.1 : r2;
}

void RCLadder::updateKalman(float measurement) {
  _x[0] += _x[1];
  _P[0][0] += _P[1][1] + _Q[0][0];
  _P[1][1] += _Q[1][1];
  float S = _P[0][0] + _R;
  float K0 = _P[0][0] / S;
  float K1 = _P[1][0] / S;
  float y = measurement - _x[0];
  _x[0] += K0 * y;
  _x[1] += K1 * y;
  float p01 = _P[0][1];
  _P[0][0] *= (1.0 - K0);
  _P[1][1] -= K1 * p01;
  _P[0][1] *= (1.0 - K0);
  _P[1][0] = _P[0][1];
}

float RCLadder::getEstimate() { return _x[0]; }

float RCLadder::getConfidence() {
  float conf = 100.0 - sqrt(_P[0][0]);
  return (conf < 0) ? 0 : conf;
}
