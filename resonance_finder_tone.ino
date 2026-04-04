/*
 * Improved precision in resonance_finder_tone.ino.
 * Uses a logarithmic sweep with parabolic interpolation to find the corner frequency.
 */

#include "src/RCConfig.h"

float measureVpp(float freq);

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  Serial.begin(9600);
}

float measureVpp(float freq) {
  unsigned long period = 1000000.0 / freq;
  unsigned long half = period / 2;
  float vMax = -1.0, vMin = 6.0;
  unsigned long start = millis();
  unsigned long window = 300; if (period > 150000) window = (period * 2.0) / 1000;
  while(millis() - start < window) {
    digitalWrite(PIN_OUT, HIGH);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    float v = (analogRead(PIN_IN) * 5.0) / 1023.0;
    if (v > vMax) vMax = v;
    digitalWrite(PIN_OUT, LOW);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    v = (analogRead(PIN_IN) * 5.0) / 1023.0;
    if (v < vMin) vMin = v;
  }
  return vMax - vMin;
}

void loop() {
  float targetVpp = 3.535; // 5.0 * 0.707 (Corner)
  float bestF = 1.0;
  float minErr = 5.0;
  float f_list[15], err_list[15];
  int idx = 0;

  for (float f = 0.5; f < 500.0; f *= 1.5) {
    float vpp = measureVpp(f);
    float err = abs(vpp - targetVpp);
    f_list[idx] = f; err_list[idx] = err;
    if (err < minErr) { minErr = err; bestF = f; }
    if (++idx >= 15) break;
  }
  
  int b_idx = 0;
  for(int i=0; i<idx; i++) if(f_list[i] == bestF) b_idx = i;

  float preciseF = bestF;
  if (b_idx > 0 && b_idx < idx - 1) {
    float x1 = f_list[b_idx-1], x2 = f_list[b_idx], x3 = f_list[b_idx+1];
    float y1 = err_list[b_idx-1], y2 = err_list[b_idx], y3 = err_list[b_idx+1];
    float denom = (x1-x2)*(x1-x3)*(x2-x3);
    float A = (x3*(y2-y1) + x2*(y1-y3) + x1*(y3-y2)) / (denom + 1e-9);
    float B = (x3*x3*(y1-y2) + x2*x2*(y3-y1) + x1*x1*(y2-y3)) / (denom + 1e-9);
    if (abs(A) > 1e-12) preciseF = -B / (2.0 * A);
  }

  float w = 2.0 * 3.14159265 * preciseF;
  float r2 = (1.0/w - (R0+R1)*(C1+C2)) / C2;

  Serial.print("CornerFreq_Hz:"); Serial.print(preciseF);
  Serial.print(" R2_Est:"); Serial.println(r2);

  delay(60000);
}
