float measureVpp(float freq);
#include "RCConfig.h"
/*
 * Corrected Kalman filter measurement in CRC_circuit_kalman.ino.
 * Uses 2nd-order RC ladder theory and 2nd-order Kalman filter for R2 tracking.
 */

#include <math.h>

const int PIN_OUT_VAL = 3;
const int PIN_IN_VAL = A0;

// Kalman Filter variables (R2 and dR2)
float x_state[2] = {1000.0, 0.0};
float P_cov[2][2] = {{1e6, 0}, {0, 1e3}};
float Q_proc[2][2] = {{10.0, 0}, {0, 1.0}};
float R_meas = 500.0;

void setup() {
  pinMode(PIN_OUT_VAL, OUTPUT);
  pinMode(PIN_IN_VAL, INPUT);
  Serial.begin(9600);
}

void safeDelayMicros(unsigned long us) {
  if (us > 16000) { delay(us/1000); delayMicroseconds(us%1000); }
  else delayMicroseconds(us);
}

float measureVpp(float freq) {
  unsigned long half = 500000.0 / freq;
  float vMax = 0, vMin = 5.0;
  unsigned long s = millis();
  while(millis() - s < 150) {
    digitalWrite(PIN_OUT_VAL, HIGH); safeDelayMicros(half);
    float v = analogRead(PIN_IN_VAL) * (VCC/1023.0);
    if (v > vMax) vMax = v;
    digitalWrite(PIN_OUT_VAL, LOW); safeDelayMicros(half);
    v = analogRead(PIN_IN_VAL) * (VCC/1023.0);
    if (v < vMin) vMin = v;
  }
  return vMax - vMin;
}

void updateKalman(float z) {
  x_state[0] += x_state[1];
  P_cov[0][0] += P_cov[1][1] + Q_proc[0][0];
  P_cov[1][1] += Q_proc[1][1];
  float S = P_cov[0][0] + R_meas;
  float K0 = P_cov[0][0] / S;
  float K1 = P_cov[1][0] / S;
  float y = z - x_state[0];
  x_state[0] += K0 * y;
  x_state[1] += K1 * y;
  float p01 = P_cov[0][1];
  P_cov[0][0] *= (1.0 - K0);
  P_cov[1][1] -= K1 * p01;
  P_cov[0][1] *= (1.0 - K0);
  P_cov[1][0] = P_cov[0][1];
}

void loop() {
  static float lastF = 22.0;
  float fLow = lastF * 0.5, fHigh = lastF * 2.0;
  if (fLow < 0.1) fLow = 0.1; if (fHigh > 2000.0) fHigh = 2000.0;
  if (measureVpp(fLow) < 3.6 || measureVpp(fHigh) > 3.6) { fLow = 0.05; fHigh = 2000.0; }
  for(int i=0; i<10; i++) {
    float fMid = (fLow + fHigh) / 2.0;
    if (measureVpp(fMid) > 3.6) fLow = fMid; else fHigh = fMid;
  }
  lastF = (fLow + fHigh) / 2.0;
  float vpp = measureVpp(lastF);
  float ratio = vpp / VCC;
  if (ratio < 0.99) {
    float artanh_val = 0.5 * log((1.0 + ratio) / (1.0 - ratio));
    float tau = 1.0 / (4.0 * lastF * artanh_val);
    float measR = (tau - (R0+R1)*(C1+C2)) / C2;
    updateKalman(measR);
    Serial.print("R2_Est: "); Serial.println(x_state[0]);
  }
  delay(60000);
}
