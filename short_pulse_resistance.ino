/*
 * Specialized transient analysis in short_pulse_resistance.ino.
 * Sends a single charging pulse and analyzes the discharge curve.
 * Uses the log-ratio of two points to determine the time constant tau.
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
  // Discharge capacitors
  digitalWrite(PIN_OUT, LOW);
  delay(1000);

  // Charge briefly
  digitalWrite(PIN_OUT, HIGH);
  delay(200);

  // Start discharge and measure
  digitalWrite(PIN_OUT, LOW);
  unsigned long t0 = micros();
  float v0 = (analogRead(PIN_IN) * VCC) / 1023.0;

  delay(50); // Wait for some decay

  unsigned long t1 = micros();
  float v1 = (analogRead(PIN_IN) * VCC) / 1023.0;

  if (v0 > v1 && v1 > 0.1) {
    // V(t) = V0 * exp(-t/tau) -> ln(V0/V1) = (t1-t0)/tau
    float dt = (t1 - t0) / 1e6;
    float tau = dt / log(v0 / v1);

    // tau_eff = (R0+R1)(C1+C2) + R2*C2
    float r2 = (tau - (R0+R1)*(C1+C2)) / C2;

    Serial.print("V0:"); Serial.print(v0);
    Serial.print(" V1:"); Serial.print(v1);
    Serial.print(" Tau_s:"); Serial.print(tau);
    Serial.print(" R2_Ohm:"); Serial.println(r2);
  } else {
    Serial.println("Measurement Failed: Insufficient Decay");
  }

  delay(60000);
}
