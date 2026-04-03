#include "RCConfig.h"
/*
 * Advanced Transient Analysis in short_pulse_resistance.ino.
 * Samples multiple points during exponential discharge and uses
 * linear regression on log-transformed data to extract tau.
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
  // Discharge
  digitalWrite(PIN_OUT_VAL, LOW);
  delay(1000);

  // Charge
  digitalWrite(PIN_OUT_VAL, HIGH);
  delay(500);

  // Measure decay
  digitalWrite(PIN_OUT_VAL, LOW);
  unsigned long startT = micros();

  float y_sum = 0, x_sum = 0, xy_sum = 0, x2_sum = 0;
  int n = 0;

  for (int i = 0; i < 20; i++) {
    int raw = analogRead(PIN_IN);
    unsigned long t = micros() - startT; // High-precision timestamp
    float v = (raw * VCC) / 1023.0;

    if (v > 0.2) { // Log valid range
      float lnV = log(v);
      float timeS = (float)t / 1e6;

      y_sum += lnV;
      x_sum += timeS;
      xy_sum += lnV * timeS;
      x2_sum += timeS * timeS;
      n++;
    }
    delay(10); // Sample every 10ms
  }

  if (n > 5) {
    // Linear regression: ln(V) = ln(V0) - (1/tau) * t
    // Slope m = (n*xy - x*y) / (n*x2 - x^2)
    float slope = (n * xy_sum - x_sum * y_sum) / (n * x2_sum - x_sum * x_sum);
    float tau = -1.0 / slope;

    float r2 = (tau - (R0+R1)*(C1+C2)) / C2;
    Serial.print("Tau_s:"); Serial.print(tau);
    Serial.print(" R2_Ohm:"); Serial.println(r2);
  } else {
    Serial.println("Error: Insufficient Data");
  }

  delay(60000);
}
