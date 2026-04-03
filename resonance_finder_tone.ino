float measureVpp(float freq);
#include "RCConfig.h"
/*
 * Sub-step precision measurement in resonance_finder_tone.ino.
 * Uses a logarithmic sweep with parabolic interpolation to find the corner
 * frequency (Gain ~ 0.707) with high precision.
 */

#include <math.h>

const int PIN_OUT_VAL = 3;
const int PIN_IN_VAL = A0;

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
  float targetVpp = 3.535; // 5.0 * 0.707
  float bestF = 1.0;
  float minErr = 5.0;

  // 1. Logarithmic Sweep to find coarse best
  float f_list[10];
  float err_list[10];
  int idx = 0;
  for (float f = 1.0; f < 500.0; f *= 1.8) {
    float vpp = measureVpp(f);
    float err = abs(vpp - targetVpp);
    f_list[idx] = f;
    err_list[idx] = err;
    if (err < minErr) {
      minErr = err;
      bestF = f;
    }
    idx++;
    if (idx >= 10) break;
  }
  
  // 2. Parabolic Interpolation for sub-step precision
  // Find index of bestF
  int b_idx = 0;
  for(int i=0; i<idx; i++) if(f_list[i] == bestF) b_idx = i;

  float preciseF = bestF;
  if (b_idx > 0 && b_idx < idx - 1) {
    float x1 = f_list[b_idx-1], x2 = f_list[b_idx], x3 = f_list[b_idx+1];
    float y1 = err_list[b_idx-1], y2 = err_list[b_idx], y3 = err_list[b_idx+1];
    // Vertex of parabola through (x1,y1), (x2,y2), (x3,y3)
    float denom = (x1-x2)*(x1-x3)*(x2-x3);
    float A = (x3*(y2-y1) + x2*(y1-y3) + x1*(y3-y2)) / denom;
    float B = (x3*x3*(y1-y2) + x2*x2*(y3-y1) + x1*x1*(y2-y3)) / denom;
    if (abs(A) > 1e-9) preciseF = -B / (2.0 * A);
  }

  // Solve for R2 using OCTC at corner frequency
  float w = 2.0 * 3.1415926535 * preciseF;
  float r2 = (1.0/w - (R0+R1)*(C1+C2)) / C2;

  Serial.print("CornerFreq:"); Serial.print(preciseF);
  Serial.print(" R2_Ohm:"); Serial.println(r2);

  delay(60000);
}
