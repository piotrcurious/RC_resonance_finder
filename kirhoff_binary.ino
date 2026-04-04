/*
 * Golden Section Search for R2 estimation in kirhoff_binary.ino.
 * Rapidly finds target gain frequency for maximum precision.
 */

#include "RCConfig.h"

float measureVpp(float freq);

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  Serial.begin(9600);
}

float measureVpp(float freq) {
  unsigned long period = 1000000.0 / freq;
  unsigned long half = period / 2;
  float vSumMax = 0, vSumMin = 0;
  int count = 0;
  unsigned long start = millis();
  unsigned long window = 300; if (period > 150000) window = (period * 2.0) / 1000;
  while(millis() - start < window) {
    digitalWrite(PIN_OUT, HIGH);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    vSumMax += (analogRead(PIN_IN) * 5.0) / 1023.0;
    digitalWrite(PIN_OUT, LOW);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    vSumMin += (analogRead(PIN_IN) * 5.0) / 1023.0;
    count++;
  }
  return (count > 0) ? (vSumMax - vSumMin) / count : 0;
}

void loop() {
  float target = 3.6;
  float a = 0.05, b = 2000.0;
  const float invphi = 0.618033988;
  const float invphi2 = 0.381966011;

  float h = b - a;
  float c = a + invphi2 * h;
  float d = a + invphi * h;
  float yc = abs(measureVpp(c) - target);
  float yd = abs(measureVpp(d) - target);

  for (int i = 0; i < 12; i++) {
    if (yc < yd) {
      b = d; d = c; yd = yc; h = b - a;
      c = a + invphi2 * h; yc = abs(measureVpp(c) - target);
    } else {
      a = c; c = d; yc = yd; h = b - a;
      d = a + invphi * h; yd = abs(measureVpp(d) - target);
    }
  }

  float bestF = (a + b) / 2.0;
  float vpp = measureVpp(bestF);
  float g = vpp / 5.0;
  if (g < 0.99 && g > 0.01) {
    float artanh_val = 0.5 * log((1.0 + g) / (1.0 - g));
    float tau = 1.0 / (4.0 * bestF * artanh_val);
    float r2 = (tau - (R0+R1)*(C1+C2)) / C2;
    Serial.print("Freq_Hz:"); Serial.print(bestF);
    Serial.print(" R2_Ohm:"); Serial.println(r2);
  }
  delay(60000);
}
