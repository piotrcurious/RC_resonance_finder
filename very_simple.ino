float measureVpp(float freq);
#include "RCConfig.h"
/*
 * Simplified R2 measurement in very_simple.ino.
 * Corrected to account for the 2nd-order RC ladder and the digital pin's internal resistance R0.
 * Uses a single frequency measurement and square-wave solver.
 */

#include <math.h>

const int PIN_OUT_VAL = 3;
const int PIN_IN_VAL = A0;

void setup() {
  pinMode(PIN_OUT_VAL, OUTPUT);
  pinMode(PIN_IN_VAL, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Use a fixed frequency for a single R2 measurement
  float freq = 20.0;
  float vpp = measureVpp(freq);
  float ratio = vpp / VCC;
  
  if (ratio < 0.99) {
    float artanh_val = 0.5 * log((1.0 + ratio) / (1.0 - ratio));
    float tau = 1.0 / (4.0 * freq * artanh_val);
    float r2 = (tau - (R0+R1)*(C1+C2)) / C2;
    Serial.print("R2: "); Serial.println(r2);
  }
  delay(60000);
}

float measureVpp(float freq) {
  uint32_t half = 500000.0 / freq;
  float vMax = 0, vMin = 5.0;
  uint32_t start = millis();
  unsigned long window = 200; if (half > 133333) window = (half * 1.5) / 1000; while(millis() - start < window) {
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
