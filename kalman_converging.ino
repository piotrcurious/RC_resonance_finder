float measureVpp(float freq);
#include "src/RCConfig.h"
/*
 * Differentiated Kalman: kalman_converging.ino.
 * Uses a decaying process noise (Q) to prioritize fast initial convergence
 * and then maximum steady-state precision.
 */

#include <math.h>
// Redefinition of PIN_OUT removed
// Redefinition of PIN_IN removed

// Kalman State
float R2_Est = 1000.0;
float P = 1e6;
float Q_base = 1.0;
float Q_initial = 1000.0;
float R_meas = 500.0;
int iter_count = 0;

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  Serial.begin(9600);
}

float measureVpp(float freq) {
  unsigned long half = 500000.0 / freq;
  float vSumMax = 0, vSumMin = 0;
  int count = 0;
  unsigned long start = millis();
  unsigned long window = 200; if (half > 133333) window = (half * 1.5) / 1000;
  while(millis() - start < window) {
    digitalWrite(PIN_OUT, HIGH);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    vSumMax += (analogRead(PIN_IN) * VCC) / 1023.0;
    digitalWrite(PIN_OUT, LOW);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    vSumMin += (analogRead(PIN_IN) * VCC) / 1023.0;
    count++;
  }
  return (vSumMax - vSumMin) / (count + 1e-6);
}

void loop() {
  float currentFreq = 22.0; // Seed
  // Simple search for target 3.6V
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

    // Converging Kalman Logic: Decay Q
    float Q_current = Q_base + Q_initial * exp(-0.1 * iter_count);
    P = P + Q_current;
    float K = P / (P + R_meas);
    R2_Est = R2_Est + K * (measR - R2_Est);
    P = (1.0 - K) * P;

    Serial.print("Iter:"); Serial.print(iter_count);
    Serial.print(" R2_Est:"); Serial.println(R2_Est);
    iter_count++;
  }

  delay(10000); // 10s for faster debug convergence
}
