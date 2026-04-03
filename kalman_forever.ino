float measureVpp(float freq);
#include "RCConfig.h"
/*
 * Differentiated Kalman: kalman_forever.ino.
 * Uses an adaptive Kalman filter that increases Q when a large residual is detected,
 * allowing it to track sudden changes while remaining stable during steady state.
 */

#include <math.h>

const int PIN_OUT_VAL = 3;
const int PIN_IN_VAL = A0;

// Kalman State
float R2_est = 1000.0;
float P = 1000.0;
float Q_steady = 1.0;
float R_meas = 500.0;

void setup() {
  pinMode(PIN_OUT_VAL, OUTPUT);
  pinMode(PIN_IN_VAL, INPUT);
  Serial.begin(9600);
}

float measureVpp(float freq) {
  unsigned long half = 500000.0 / freq;
  float vSumMax = 0, vSumMin = 0;
  int count = 0;
  unsigned long start = millis();
  unsigned long window = 200; if (half > 133333) window = (half * 1.5) / 1000;
  while(millis() - start < window) {
    digitalWrite(PIN_OUT_VAL, HIGH);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    vSumMax += (analogRead(PIN_IN_VAL) * VCC) / 1023.0;
    digitalWrite(PIN_OUT_VAL, LOW);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    vSumMin += (analogRead(PIN_IN_VAL) * VCC) / 1023.0;
    count++;
  }
  return (vSumMax - vSumMin) / (count + 1e-6);
}

void loop() {
  // Frequency Search
  float fLow = 0.1, fHigh = 1000.0;
  for(int i=0; i<10; i++) {
    float fMid = (fLow + fHigh) / 2.0;
    if (measureVpp(fMid) > 3.6) fLow = fMid; else fHigh = fMid;
  }
  float f = (fLow + fHigh) / 2.0;
  float vpp = measureVpp(f);
  float ratio = vpp / VCC;

  if (ratio < 0.99) {
    float artanh_val = 0.5 * log((1.0 + ratio) / (1.0 - ratio));
    float tau = 1.0 / (4.0 * f * artanh_val);
    float measR = (tau - (R0+R1)*(C1+C2)) / C2;

    // Adaptive Kalman Logic
    float residual = abs(measR - R2_est);
    float Q_adaptive = Q_steady;
    if (residual > 200.0) Q_adaptive = residual * 2.0; // Bump Q for sudden shifts

    P = P + Q_adaptive;
    float K = P / (P + R_meas);
    R2_est = R2_est + K * (measR - R2_est);
    P = (1.0 - K) * P;

    Serial.print("R2_Est:"); Serial.println(R2_est);
  }

  delay(60000);
}
