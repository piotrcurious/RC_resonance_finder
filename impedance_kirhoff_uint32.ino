/*
 * Corrected Kirchhoff-based R2 measurement in impedance_kirhoff_uint32.ino.
 * Uses 32-bit timing and 2nd-order RC ladder model for higher precision.
 */

#include <math.h>

const int PIN_OUT = 9;
const int PIN_IN = A0;
const float R0 = 250.0, R1 = 1.0, C1 = 0.5e-6, C2 = 10.0e-6, VCC = 5.0;

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Use a fixed frequency sweep for stability
  float bestR = 1000.0;
  float targetVpp = 3.6;

  float fLow = 0.5, fHigh = 1000.0;
  for (int i=0; i<12; i++) {
    float fMid = (fLow + fHigh) / 2.0;
    float vpp = measureVpp(fMid);
    if (vpp > targetVpp) fLow = fMid; else fHigh = fMid;
  }
  
  float freq = (fLow + fHigh) / 2.0;
  float vpp = measureVpp(freq);
  float ratio = vpp / VCC;

  if (ratio < 0.99) {
    float artanh_val = 0.5 * log((1.0 + ratio) / (1.0 - ratio));
    float tau = 1.0 / (4.0 * freq * artanh_val);
    bestR = (tau - (R0+R1)*(C1+C2)) / C2;
    Serial.print("Freq: "); Serial.print(freq);
    Serial.print(" Hz, R2: "); Serial.println(bestR);
  }
  delay(60000);
}

float measureVpp(float freq) {
  uint32_t half = 500000.0 / freq;
  float vMax = 0, vMin = 5.0;
  uint32_t start = millis();
  unsigned long window = 200; if (half > 133333) window = (half * 1.5) / 1000; while(millis() - start < window) {
    digitalWrite(PIN_OUT, HIGH);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    float v = analogRead(PIN_IN) * (VCC/1023.0);
    if (v > vMax) vMax = v;
    digitalWrite(PIN_OUT, LOW);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    v = analogRead(PIN_IN) * (VCC/1023.0);
    if (v < vMin) vMin = v;
  }
  return vMax - vMin;
}
