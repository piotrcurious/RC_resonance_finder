/*
 * Kalman Filter for 2nd-order RC circuit.
 * Tracks R and frequency using measurements of Vpp and Period.
 */

#define PIN_OUT 13
#define PIN_IN A0
#define C1 0.5e-6
#define C2 10e-6
#define R0 250.0
#define R1 1.0
#define V_MAX 5.0
#define T_INTERVAL 60000

float estimatedR = 1000.0;
float currentFreq = 22.0;

// Simple 1D Kalman for R estimation
float P = 1000000.0;
float Q = 10.0;
float R_noise = 500.0;

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  Serial.begin(9600);
}

float measureVpp(float freq) {
  unsigned long halfPeriod = 500000.0 / freq;
  float vMax = 0, vMin = 5.0;
  unsigned long start = millis();
  unsigned long window = 200; if (halfPeriod > 133333) window = (halfPeriod * 1.5) / 1000; while(millis() - start < window) {
    digitalWrite(PIN_OUT, HIGH);
    if (halfPeriod > 16000) delay(halfPeriod/1000); else delayMicroseconds(halfPeriod);
    float v = analogRead(PIN_IN) * (V_MAX / 1023.0);
    if (v > vMax) vMax = v;
    digitalWrite(PIN_OUT, LOW);
    if (halfPeriod > 16000) delay(halfPeriod/1000); else delayMicroseconds(halfPeriod);
    v = analogRead(PIN_IN) * (V_MAX / 1023.0);
    if (v < vMin) vMin = v;
  }
  return vMax - vMin;
}

float solveR(float f, float vpp) {
  float ratio = vpp / V_MAX;
  if (ratio >= 0.99) return estimatedR;
  float artanh_val = 0.5 * log((1.0 + ratio) / (1.0 - ratio));
  float tau = 1.0 / (4.0 * f * artanh_val);
  return (tau - (R0+R1)*(C1+C2)) / C2;
}

void loop() {
  static unsigned long lastTime = 0;
  if (millis() - lastTime >= T_INTERVAL || lastTime == 0) {
    lastTime = millis();
    
    // Find frequency for Vpp ~ 3.6V
    float fLow = 0.1, fHigh = 1000.0;
    for(int i=0; i<10; i++) {
      float fMid = (fLow + fHigh) / 2.0;
      if (measureVpp(fMid) > 3.6) fLow = fMid;
      else fHigh = fMid;
    }
    currentFreq = (fLow + fHigh) / 2.0;
    float vpp = measureVpp(currentFreq);
    float measR = solveR(currentFreq, vpp);

    // Kalman update
    P = P + Q;
    float K = P / (P + R_noise);
    estimatedR = estimatedR + K * (measR - estimatedR);
    P = (1 - K) * P;
    
    Serial.print("R_est: "); Serial.println(estimatedR);
  }
}
