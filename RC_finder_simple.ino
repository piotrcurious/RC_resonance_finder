const int PIN_OUT_VAL = 3;
const int PIN_IN_VAL = A0;
float measureVpp(float freq);
#include "RCConfig.h"
/*
 * Simple RC Resistance Finder.
 * Binary search for a stable gain point and OCTC calculation.
 */

const float C1_val = 0.5e-6;
const float C2_val = 10.0e-6;
#define RA_INO (250.0 + 1.0)
#define VCC 5.0

void setup() {
  pinMode(PIN_OUT_VAL, OUTPUT);
  pinMode(PIN_IN_VAL, INPUT);
  Serial.begin(9600);
}

float measureVpp(float freq) {
  unsigned long half = 500000.0 / freq;
  float vMax = 0, vMin = 5.0;
  unsigned long s = millis();
  while(millis() - s < 100) {
    digitalWrite(PIN_OUT_VAL, HIGH);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    float v = analogRead(PIN_IN_VAL) * (VCC/1023.0);
    if (v > vMax) vMax = v;
    digitalWrite(PIN_OUT_VAL, LOW);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    v = analogRead(PIN_IN_VAL) * (VCC/1023.0);
    if (v < vMin) vMin = v;
  }
  return vMax - vMin;
}

void loop() {
  float fL = 0.1, fH = 1000.0;
  for(int i=0; i<10; i++) {
    float fM = (fL + fH) / 2.0;
    if (measureVpp(fM) > 3.6) fL = fM; else fH = fM;
  }
  float f = (fL + fH) / 2.0;
  float vpp = measureVpp(f);
  float artanh_val = 0.5 * log((1.0 + vpp/VCC) / (1.0 - vpp/VCC));
  float tau = 1.0 / (4.0 * f * artanh_val);
  float r2 = (tau - RA_INO*(C1+C2)) / C2;

  Serial.print("R2: "); Serial.println(r2);
  delay(60000);
}
