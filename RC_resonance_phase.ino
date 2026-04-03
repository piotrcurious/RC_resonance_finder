float measurePhase(float f);
const int PIN_OUT_VAL = 3;
const int PIN_IN_VAL = A0;
#include "RCConfig.h"
/*
 * Measurement using Phase Shift in 2nd-order RC ladder.
 * Tracks frequency where phase shift is 90 degrees.
 */

const float C1_val = 0.5e-6;
const float C2_val = 10.0e-6;
const float R0_val = 250.0;
const float R1_val = 1.0;

void setup() {
  pinMode(PIN_OUT_VAL, OUTPUT);
  pinMode(PIN_IN_VAL, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Find frequency where phase is 90 deg (half-period delay)
  // For 2nd order, 90 deg phase is a stable and unique point.
  float fLow = 1.0, fHigh = 1000.0;
  for(int i=0; i<12; i++) {
    float fMid = (fLow + fHigh) / 2.0;
    float phase = measurePhase(fMid);
    if (phase < 90.0) fLow = fMid;
    else fHigh = fMid;
  }

  float targetF = (fLow + fHigh) / 2.0;
  // At 90 deg phase for 2nd order RC:
  // w^2 Ra R2 C1 C2 = 1 -> R2 = 1 / (w^2 Ra C1 C2)
  float w = 2.0 * 3.1415926535 * targetF;
  float r2 = 1.0 / (w * w * (R0+R1) * C1 * C2);

  Serial.print("90deg Phase Freq: "); Serial.print(targetF);
  Serial.print(" Hz, Calc R2: "); Serial.println(r2);

  delay(60000);
}

float measurePhase(float f) {
  unsigned long period = 1000000.0 / f;
  unsigned long halfPeriod = period / 2;

  // Stabilize for a few cycles
  for (int i=0; i<3; i++) {
    digitalWrite(PIN_OUT_VAL, HIGH);
    if (halfPeriod > 16000) delay(halfPeriod/1000); else delayMicroseconds(halfPeriod);
    digitalWrite(PIN_OUT_VAL, LOW);
    if (halfPeriod > 16000) delay(halfPeriod/1000); else delayMicroseconds(halfPeriod);
  }

  unsigned long start = micros();
  digitalWrite(PIN_OUT_VAL, HIGH);

  // Use a small moving average to detect crossing (noise immunity)
  int avg = 0;
  while(true) {
    avg = (avg * 3 + analogRead(PIN_IN_VAL)) >> 2;
    if (avg >= 512) break;
    if (micros() - start > period) return 0;
  }
  unsigned long crossing = micros() - start;

  if (halfPeriod > 16000) delay(halfPeriod/1000); else delayMicroseconds(halfPeriod);
  digitalWrite(PIN_OUT_VAL, LOW);

  return (crossing * 360.0) / period;
}
