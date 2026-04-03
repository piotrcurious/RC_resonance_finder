/*
 * Simple resistance measurement using charging time.
 * V(t) = Vcc * (1 - exp(-t/tau))
 * Measures time to reach a threshold and solves for R2.
 */

#define PIN_OUT 9
#define PIN_IN A0
#define C1 0.5e-6
#define C2 10e-6
#define R0 250.0
#define R1 1.0

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
    if (micros() - start > 5000000) break; // Timeout
  }
  unsigned long duration = micros() - start;

  // tau = duration (approx if threshold is 1-1/e)
  // tau_eff = Ra(C1+C2) + R2*C2
  float tau = (float)duration / 1e6;
  float r2 = (tau - (R0+R1)*(C1+C2)) / C2;

  Serial.print("Charge time: "); Serial.print(duration);
  Serial.print(" us, Calc R2: "); Serial.println(r2);

  delay(60000);
}
