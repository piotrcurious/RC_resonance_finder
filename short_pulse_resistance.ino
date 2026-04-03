/*
 * Advanced Transient Analysis in short_pulse_resistance.ino.
 * Samples multiple points during exponential discharge and uses
 * linear regression on log-transformed data to extract tau.
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
  int n = 0;

  for (int i = 0; i < 20; i++) {
    unsigned long t = micros() - startT;
    float v = (analogRead(PIN_IN) * VCC) / 1023.0;

    if (v > 0.2) { // Log valid range
      float lnV = log(v);
      float timeS = t / 1e6;

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
