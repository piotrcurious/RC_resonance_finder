float measureVpp(float freq);
#include "RCConfig.h"
/*
 * Improved search in kirhoff_binary.ino.
 * Uses a Golden Section Search to find the target frequency for R2 estimation.
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
  // Golden Section Search for frequency where Vpp ~ 3.6V
  float a = 0.1, b = 1000.0;
  const float invphi = 0.61803398875;
  const float invphi2 = 0.38196601125;

  float target = 3.6;
  float h = b - a;
  int n = 12; // iterations

  float c = a + invphi2 * h;
  float d = a + invphi * h;
  float yc = abs(measureVpp(c) - target);
  float yd = abs(measureVpp(d) - target);

  for (int k = 0; k < n; k++) {
    if (yc < yd) {
      b = d; d = c; yd = yc;
      h = b - a;
      c = a + invphi2 * h;
      yc = abs(measureVpp(c) - target);
    } else {
      a = c; c = d; yc = yd;
      h = b - a;
      d = a + invphi * h;
      yd = abs(measureVpp(d) - target);
    }
  }

  float bestF = (a + b) / 2.0;
  float vpp = measureVpp(bestF);
  float ratio = vpp / VCC;

  if (ratio < 0.99) {
    float artanh_val = 0.5 * log((1.0 + ratio) / (1.0 - ratio));
    float tau = 1.0 / (4.0 * bestF * artanh_val);
    float r2 = (tau - (R0+R1)*(C1+C2)) / C2;
    Serial.print("Freq_Hz:"); Serial.print(bestF);
    Serial.print(" R2_Ohm:"); Serial.println(r2);
  }
  delay(60000);
}
