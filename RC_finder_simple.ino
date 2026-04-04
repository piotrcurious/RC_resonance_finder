// Redefinition of PIN_OUT removed
// Redefinition of PIN_IN removed
float measureVpp(float freq);
#include "src/RCConfig.h"
/*
 * Simple RC Resistance Finder.
 * Binary search for a stable gain point and OCTC calculation.
 */
// Redefinition of C1 removed
// Redefinition of C2 removed
// #define RA_INO removed
// #define VCC removed

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  Serial.begin(9600);
}

float measureVpp(float freq) {
  unsigned long half = 500000.0 / freq;
  float vMax = 0, vMin = 5.0;
  unsigned long s = millis();
  while(millis() - s < 100) {
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
  float r2 = (tau - (R0+R1)*(C1+C2)) / C2;

  Serial.print("R2_Est:"); Serial.println(r2);
  delay(60000);
}
