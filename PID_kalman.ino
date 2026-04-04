// Redefinition of PIN_OUT removed
// Redefinition of PIN_IN removed
float measureVpp(float freq);
#include "src/RCConfig.h"
/*
 * PID-controlled R2 measurement for 2nd-order RC ladder.
 * Uses a PID loop to adjust frequency until Vpp matches target.
 */
// Redefinition of C1 removed
// Redefinition of C2 removed
// Redefinition of R0 removed
// Redefinition of R1 removed
// #define VCC removed

// PID Constants
float Kp = 5.0, Ki = 1.0, Kd = 0.1;
float targetVpp = 3.6;
float currentFreq = 22.0;
float integral = 0, lastError = 0;

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  Serial.begin(9600);
}

float measureVpp(float freq) {
  if (freq < 0.1) freq = 0.1;
  unsigned long halfPeriod = 500000.0 / freq;
  float vMax = 0, vMin = 5.0;
  unsigned long start = millis();
  while(millis() - start < 100) {
    digitalWrite(PIN_OUT, HIGH);
    if (halfPeriod > 16000) delay(halfPeriod/1000); else delayMicroseconds(halfPeriod);
    float v = analogRead(PIN_IN) * (VCC / 1023.0);
    if (v > vMax) vMax = v;
    digitalWrite(PIN_OUT, LOW);
    if (halfPeriod > 16000) delay(halfPeriod/1000); else delayMicroseconds(halfPeriod);
    v = analogRead(PIN_IN) * (VCC / 1023.0);
    if (v < vMin) vMin = v;
  }
  return vMax - vMin;
}

void loop() {
  float vpp = measureVpp(currentFreq);
  float error = targetVpp - vpp;

  integral += error;
  float derivative = error - lastError;
  lastError = error;

  // Adjust frequency (higher frequency -> lower Vpp)
  currentFreq -= (Kp * error + Ki * integral + Kd * derivative);
  if (currentFreq < 0.1) currentFreq = 0.1;
  if (currentFreq > 5000) currentFreq = 5000;

  // Every few iterations, calculate R2
  static int count = 0;
  if (++count % 10 == 0) {
    float artanh_val = 0.5 * log((1.0 + vpp/VCC) / (1.0 - vpp/VCC));
    float tau = 1.0 / (4.0 * currentFreq * artanh_val);
    float r2 = (tau - (R0+R1)*(C1+C2)) / C2;
    Serial.print("Freq_Hz: "); Serial.print(currentFreq);
    Serial.print(" Hz, Vpp: "); Serial.print(vpp);
    Serial.print(" V, R2_Est:"); Serial.println(r2);
  }

  delay(100);
}
