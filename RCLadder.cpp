#include "RCLadder.h"
#include <math.h>
RCLadder::RCLadder(int pinOut, int pinIn, float r1, float c1, float c2, float r0) {
  _pinOut = pinOut; _pinIn = pinIn; _r1 = r1; _c1 = c1; _c2 = c2; _r0 = r0;
  pinMode(_pinOut, OUTPUT); pinMode(_pinIn, INPUT);
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
  unsigned long lastSwitch = micros();
  int state = LOW;

  while(millis() - start < window) {
    unsigned long now = micros();
    if (now - lastSwitch >= half) {
      state = (state == LOW) ? HIGH : LOW;
      digitalWrite(_pinOut, state);
      lastSwitch = now;

      // Sample once after state has stabilized (wait 10% of half-period)
      safeDelayMicros(half / 10);
      float v = (analogRead(_pinIn) * 5.0) / 1023.0;
      if (state == HIGH) vSumMax += v;
      else { vSumMin += v; count++; }
    }
    if (millis() - start > 5000) break;
  }
  return (count > 0) ? (vSumMax - vSumMin) / count : 0;
}

float RCLadder::findCrossingMicros(float freq, int threshold) {
  unsigned long period = 1000000.0 / freq;
  unsigned long half = period / 2;

  // 1. Stabilize
  for(int i=0; i<3; i++) {
    digitalWrite(_pinOut, HIGH); safeDelayMicros(half);
    digitalWrite(_pinOut, LOW); safeDelayMicros(half);
  }

  // 2. Measure with timestamping
  unsigned long start = micros();
  digitalWrite(_pinOut, HIGH);

  int v_prev = analogRead(_pinIn);
  unsigned long t_prev = micros();

  while(micros() - start < period) {
    int v_curr = analogRead(_pinIn);
    unsigned long t_curr = micros();

    if (v_prev < threshold && v_curr >= threshold) {
      // Linear interpolation for sub-microsecond precision
      float fraction = (float)(threshold - v_prev) / (v_curr - v_prev);
      digitalWrite(_pinOut, LOW);
      return (t_prev - start) + fraction * (t_curr - t_prev);
    }
    v_prev = v_curr;
    t_prev = t_curr;
  }
  digitalWrite(_pinOut, LOW);
  return 0;
}
float RCLadder::solveR2(float freq, float vpp) {
  float ratio = vpp / 5.0;
  if (ratio >= 0.99) return 1.0;
  if (ratio <= 0.01) return 1e6;
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
  _x[0] += K0 * y; _x[1] += K1 * y;
  float p01 = _P[0][1];
  _P[0][0] *= (1.0 - K0); _P[1][1] -= K1 * p01;
  _P[0][1] *= (1.0 - K0); _P[1][0] = _P[0][1];
}
float RCLadder::getEstimate() { return _x[0]; }
float RCLadder::getConfidence() {
  float conf = 100.0 - sqrt(_P[0][0]);
  return (conf < 0) ? 0 : conf;
}
