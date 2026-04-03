/*
 * Measurement using Phase Shift in 2nd-order RC ladder.
 * Tracks frequency where phase shift is 90 degrees.
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
  // Find frequency where phase is 90 deg (half-period delay)
  // For 2nd order, 90 deg phase is a stable and unique point.
  float fLow = 1.0, fHigh = 1000.0;
  for(int i=0; i<12; i++) {
    float fMid = (fLow + fHigh) / 2.0;
    float phase = measurePhase(fMid);
    if (phase < 90.0) fLow = fMid;
    else fHigh = fMid;
  }

  float targetF = (fLow + fHigh) / 2.0;
  // At 90 deg phase for 2nd order RC:
  // w^2 Ra R2 C1 C2 = 1 -> R2 = 1 / (w^2 Ra C1 C2)
  float w = 2.0 * PI * targetF;
  float r2 = 1.0 / (w * w * (R0+R1) * C1 * C2);

  Serial.print("90deg Phase Freq: "); Serial.print(targetF);
  Serial.print(" Hz, Calc R2: "); Serial.println(r2);

  delay(60000);
}

float measurePhase(float f) {
  unsigned long period = 1000000.0 / f;
  unsigned long halfPeriod = period / 2;
  unsigned long start = micros();
  digitalWrite(PIN_OUT, HIGH);
  
  // Wait for rising edge crossing Vcc/2
  while(analogRead(PIN_IN) < 512) {
    if (micros() - start > period) return 0;
  }
  unsigned long crossing = micros() - start;

  // Reset pin
  delayMicroseconds(halfPeriod);
  digitalWrite(PIN_OUT, LOW);

  return (crossing * 360.0) / period;
}
