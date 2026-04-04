// Redefinition of PIN_OUT removed
// Redefinition of PIN_IN removed
#include "src/RCConfig.h"
/*
 * Simple resistance measurement using charging time.
 * V(t) = Vcc * (1 - exp(-t/tau))
 * Measures time to reach a threshold and solves for R2.
 */
// Redefinition of C1 removed
// Redefinition of C2 removed
// Redefinition of R0 removed
// Redefinition of R1 removed

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Discharge capacitors first
  pinMode(PIN_OUT, OUTPUT);
  digitalWrite(PIN_OUT, LOW);
  delay(1000);

  // Start charging
  unsigned long start = micros();
  digitalWrite(PIN_OUT, HIGH);

  float threshold = 3.16; // 5.0 * (1 - 1/e) approx
  while(analogRead(PIN_IN) * (5.0/1023.0) < threshold) {
    if (micros() - start > 1000000) break; // Increased timeout for 100k
  }
  unsigned long duration = micros() - start;

  // tau = duration (approx if threshold is 1-1/e)
  // tau_eff = Ra(C1+C2) + R2*C2
  float tau = (float)duration / 1e6;
  float r2 = (tau - (R0+R1)*(C1+C2)) / C2;

  Serial.print("Charge time: "); Serial.print(duration);
  Serial.print(" us, R2_Est:"); Serial.println(r2);

  delay(60000);
}
