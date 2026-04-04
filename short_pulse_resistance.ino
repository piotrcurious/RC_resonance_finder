#include "src/RCConfig.h"
/*
 * Advanced Transient Analysis in short_pulse_resistance.ino.
 * Samples multiple points during exponential discharge and uses
 * linear regression on log-transformed data to extract tau.
 */

#include <math.h>
// Redefinition of PIN_OUT removed
// Redefinition of PIN_IN removed

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Discharge
  digitalWrite(PIN_OUT, LOW);
  delay(1000);

  // Charge
  digitalWrite(PIN_OUT, HIGH);
  delay(500);

  // Measure decay
  digitalWrite(PIN_OUT, LOW);
  unsigned long startT = micros();

  float y_sum = 0, x_sum = 0, xy_sum = 0, x2_sum = 0;
  float cy = 0, cx = 0, cxy = 0, cx2 = 0; // Kahan compensators
  int n = 0;

  for (int i = 0; i < 20; i++) {
    int raw = analogRead(PIN_IN);
    unsigned long t = micros() - startT;
    float v = (raw * VCC) / 1023.0;

    if (v > 0.2) {
      float lnV = log(v);
      float timeS = (float)t / 1e6;

      // Kahan y_sum
      float dy = lnV - cy; float ty = y_sum + dy; cy = (ty - y_sum) - dy; y_sum = ty;
      // Kahan x_sum
      float dx = timeS - cx; float tx = x_sum + dx; cx = (tx - x_sum) - dx; x_sum = tx;
      // Kahan xy_sum
      float dxy = (lnV * timeS) - cxy; float txy = xy_sum + dxy; cxy = (txy - xy_sum) - dxy; xy_sum = txy;
      // Kahan x2_sum
      float dx2 = (timeS * timeS) - cx2; float tx2 = x2_sum + dx2; cx2 = (tx2 - x2_sum) - dx2; x2_sum = tx2;

      n++;
    }
    delay(1);
  }

  if (n > 5) {
    // Linear regression: ln(V) = ln(V0) - (1/tau) * t
    // Slope m = (n*xy - x*y) / (n*x2 - x^2)
    float slope = (n * xy_sum - x_sum * y_sum) / (n * x2_sum - x_sum * x_sum);
    float tau = -1.0 / slope;

    float r2 = (tau - (R0+R1)*(C1+C2)) / C2;
    Serial.print("Tau_s:"); Serial.print(tau);
    Serial.print(" R2_Est:"); Serial.println(r2);
  } else {
    Serial.println("Error: Insufficient Data");
  }

  delay(60000);
}
