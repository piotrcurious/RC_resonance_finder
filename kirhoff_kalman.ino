float measureVpp(float freq);
#include "RCConfig.h"
/*
 * Differentiated Kalman: kirhoff_kalman.ino.
 * Uses a 2nd-order Kalman filter (R2 and dR2) with direct voltage measurement updates.
 */

#include <math.h>

const int PIN_OUT_VAL = 3;
const int PIN_IN_VAL = A0;

// Kalman Filter variables
float x_state[2] = {1000.0, 0.0};
float P_cov[2][2] = {{1e6, 0}, {0, 1e3}};
float Q_proc[2][2] = {{10.0, 0}, {0, 1.0}};
float R_meas = 500.0;

void setup() {
  pinMode(PIN_OUT_VAL, OUTPUT);
  pinMode(PIN_IN_VAL, INPUT);
  Serial.begin(9600);
}

float measureVpp(float freq) {
  unsigned long half = 500000.0 / freq;
  float vSumMax = 0, vSumMin = 0;
  int count = 0;
  unsigned long start = millis();
  unsigned long window = 200; if (half > 133333) window = (half * 1.5) / 1000;
  while(millis() - start < window) {
    digitalWrite(PIN_OUT_VAL, HIGH);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    vSumMax += (analogRead(PIN_IN_VAL) * VCC) / 1023.0;
    digitalWrite(PIN_OUT_VAL, LOW);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    vSumMin += (analogRead(PIN_IN_VAL) * VCC) / 1023.0;
    count++;
  }
  return (vSumMax - vSumMin) / (count + 1e-6);
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
  float freq = 20.0;
  float vpp = measureVpp(freq);
  float ratio = vpp / VCC;

  if (ratio < 0.99) {
    float artanh_val = 0.5 * log((1.0 + ratio) / (1.0 - ratio));
    float tau = 1.0 / (4.0 * freq * artanh_val);
    float measR = (tau - (R0+R1)*(C1+C2)) / C2;
    updateKalman(measR);
    Serial.print("R_est:"); Serial.println(x_state[0]);
  }
  delay(60000);
}
