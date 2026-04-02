/*
 * PID-controlled R2 measurement for 2nd-order RC ladder.
 * Uses a PID loop to adjust frequency until Vpp matches target.
 */

#define PIN_OUT 9
#define PIN_IN A0
#define C1 0.5e-6
#define C2 10e-6
#define R0 250.0
#define R1 1.0
#define VCC 5.0

// PID Constants
float Kp = 5.0, Ki = 1.0, Kd = 0.1;
float targetVpp = 3.6;
float currentFreq = 22.0;
float integral = 0, lastError = 0;

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  Serial.begin(9600);
}

float measureVpp(float freq) {
  if (freq < 0.1) freq = 0.1;
  unsigned long halfPeriod = 500000.0 / freq;
  float vMax = 0, vMin = 5.0;
  unsigned long start = millis();
  while(millis() - start < 100) {
    digitalWrite(PIN_OUT, HIGH);
    if (halfPeriod > 16000) delay(halfPeriod/1000); else delayMicroseconds(halfPeriod);
    float v = analogRead(PIN_IN) * (VCC / 1023.0);
    if (v > vMax) vMax = v;
    digitalWrite(PIN_OUT, LOW);
    if (halfPeriod > 16000) delay(halfPeriod/1000); else delayMicroseconds(halfPeriod);
    v = analogRead(PIN_IN) * (VCC / 1023.0);
    if (v < vMin) vMin = v;
  }
  return vMax - vMin;
}

void loop() {
  float vpp = measureVpp(currentFreq);
  float error = targetVpp - vpp;

  integral += error;
  float derivative = error - lastError;
  lastError = error;

  // Adjust frequency (higher frequency -> lower Vpp)
  currentFreq -= (Kp * error + Ki * integral + Kd * derivative);
  if (currentFreq < 0.1) currentFreq = 0.1;
  if (currentFreq > 5000) currentFreq = 5000;

  // Every few iterations, calculate R2
  static int count = 0;
  if (++count % 10 == 0) {
    float artanh_val = 0.5 * log((1.0 + vpp/VCC) / (1.0 - vpp/VCC));
    float tau = 1.0 / (4.0 * currentFreq * artanh_val);
    float r2 = (tau - (R0+R1)*(C1+C2)) / C2;
    Serial.print("Freq: "); Serial.print(currentFreq);
    Serial.print(" Hz, Vpp: "); Serial.print(vpp);
    Serial.print(" V, R2: "); Serial.println(r2);
  }

  delay(100);
}
