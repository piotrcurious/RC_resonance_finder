/*
 * Specialized Two-Frequency Measurement in dual_RC_convergent.ino.
 * Uses a high frequency to characterize the first stage (R0, R1, C1)
 * and a low frequency for the total ladder (R2, C2).
 * This version dynamically estimates Ra from high-frequency data.
 */

#include <math.h>

const int PIN_OUT = 9;
const int PIN_IN = A0;
const float R1 = 1.0, C1 = 0.5e-6, C2 = 10.0e-6, VCC = 5.0;

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
  while(millis() - start < 300) {
    digitalWrite(PIN_OUT, HIGH);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    vSumMax += (analogRead(PIN_IN) * VCC) / 1023.0;
    digitalWrite(PIN_OUT, LOW);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    vSumMin += (analogRead(PIN_IN) * VCC) / 1023.0;
    count++;
  }
  return (vSumMax - vSumMin) / count;
}

void loop() {
  // 1. High Frequency measurement (~2000 Hz)
  // C2 is approx a short. Vpp_node1 = Vcc * (1/(w*C1)) / sqrt(Ra^2 + (1/wC1)^2)
  float fHigh = 2000.0;
  float vpp_high = measureVpp(fHigh);
  float gHigh = vpp_high / VCC;
  float wHigh = 2.0 * PI * fHigh;
  // Estimation of Ra: Ra = sqrt((1/gHigh)^2 - 1) / (wHigh * C1)
  float Ra_est = sqrt(1.0/(gHigh*gHigh + 1e-6) - 1.0) / (wHigh * C1);
  if (Ra_est < 100.0) Ra_est = 251.0; // Fallback

  // 2. Low Frequency measurement (~20 Hz)
  float fLow = 20.0;
  float vpp_low = measureVpp(fLow);
  float gLow = vpp_low / VCC;

  // Solve for R2 using the estimated Ra
  if (gLow < 0.99) {
    float artanh_val = 0.5 * log((1.0 + gLow) / (1.0 - gLow));
    float tau = 1.0 / (4.0 * fLow * artanh_val);
    float r2 = (tau - Ra_est * (C1 + C2)) / C2;

    Serial.print("Ra_Est_Ohm:"); Serial.print(Ra_est);
    Serial.print(" Vpp_Low_V:"); Serial.print(vpp_low);
    Serial.print(" R2_Ohm:"); Serial.println(r2);
  }

  delay(60000);
}
