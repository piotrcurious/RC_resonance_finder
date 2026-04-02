/*
 * Kalman Filter with Seeding for 2nd-order RC circuit.
 * Designed for continuous monitoring and fast convergence.
 */

#define PIN_OUT 9
#define PIN_IN A0
#define C1 0.5e-6
#define C2 10e-6
#define R0 250.0
#define R1 1.0
#define V_MAX 5.0
#define T_INTERVAL 60000

float estimatedR = 1000.0;
float currentFreq = 22.0;

// Kalman Filter variables (R2 and dR2)
float x[2] = {1000.0, 0.0};
float P[2][2] = {{1e6, 0}, {0, 1e3}};
float Q[2][2] = {{10.0, 0}, {0, 1.0}};
float R_noise = 500.0;

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  Serial.begin(9600);
}

void safeDelayMicros(unsigned long us) {
  if (us > 16000) { delay(us/1000); delayMicroseconds(us%1000); }
  else delayMicroseconds(us);
}

float measureVpp(float freq) {
  unsigned long halfPeriod = 500000.0 / freq;
  float vMax = 0, vMin = 5.0;
  unsigned long start = millis();
  while(millis() - start < 150) {
    digitalWrite(PIN_OUT, HIGH); safeDelayMicros(halfPeriod);
    float v = analogRead(PIN_IN) * (V_MAX / 1023.0);
    if (v > vMax) vMax = v;
    digitalWrite(PIN_OUT, LOW); safeDelayMicros(halfPeriod);
    v = analogRead(PIN_IN) * (V_MAX / 1023.0);
    if (v < vMin) vMin = v;
  }
  return vMax - vMin;
}

void updateKalman(float z) {
  x[0] += x[1];
  P[0][0] += P[1][1] + Q[0][0];
  P[1][1] += Q[1][1];
  float S = P[0][0] + R_noise;
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;
  float y = z - x[0];
  x[0] += K0 * y;
  x[1] += K1 * y;
  float p01 = P[0][1];
  P[0][0] *= (1.0 - K0);
  P[1][1] -= K1 * p01;
  P[0][1] *= (1.0 - K0);
  P[1][0] = P[0][1];
}

void loop() {
  static unsigned long lastTime = 0;
  if (millis() - lastTime >= T_INTERVAL || lastTime == 0) {
    lastTime = millis();
    float fLow = currentFreq * 0.5, fHigh = currentFreq * 2.0;
    if (fLow < 0.1) fLow = 0.1; if (fHigh > 5000.0) fHigh = 5000.0;
    if (measureVpp(fLow) < 3.6 || measureVpp(fHigh) > 3.6) { fLow = 0.05; fHigh = 2000.0; }
    for(int i=0; i<10; i++) {
      float fMid = (fLow + fHigh) / 2.0;
      if (measureVpp(fMid) > 3.6) fLow = fMid; else fHigh = fMid;
    }
    currentFreq = (fLow + fHigh) / 2.0;
    float vpp = measureVpp(currentFreq);
    float artanh_val = 0.5 * log((1.0 + vpp/V_MAX) / (1.0 - vpp/V_MAX));
    float tau = 1.0 / (4.0 * currentFreq * artanh_val);
    float measR = (tau - (R0+R1)*(C1+C2)) / C2;
    updateKalman(measR);
    Serial.print("R: "); Serial.println(x[0]);
  }
}
