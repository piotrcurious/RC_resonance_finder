/*
 * Corrected PID and Autotuning in CRC_kalman_pid_autotune.ino.
 * Uses 2nd-order RC ladder theory and 2nd-order Kalman filter for R2 tracking.
 * PID loop adjusts frequency to maintain target Vpp.
 */

#include <math.h>

const int PIN_OUT = 9;
const int PIN_IN = A0;
const float R0 = 250.0, R1 = 1.0, C1 = 0.5e-6, C2 = 10.0e-6, VCC = 5.0;

// PID and Kalman variables
float currentFreq = 22.0;
float estimatedR2 = 1000.0;
float x_state[2] = {1000.0, 0.0};
float P_cov[2][2] = {{1e6, 0}, {0, 1e3}};
float Q_proc[2][2] = {{10.0, 0}, {0, 1.0}};
float R_meas = 500.0;

float Kp = 5.0, Ki = 1.0, Kd = 0.1;
float targetVpp = 3.6;
float integral = 0, lastError = 0;

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
  unsigned long half = 500000.0 / freq;
  float vMax = 0, vMin = 5.0;
  unsigned long s = millis();
  while(millis() - s < 100) {
    digitalWrite(PIN_OUT, HIGH); safeDelayMicros(half);
    float v = analogRead(PIN_IN) * (VCC/1023.0);
    if (v > vMax) vMax = v;
    digitalWrite(PIN_OUT, LOW); safeDelayMicros(half);
    v = analogRead(PIN_IN) * (VCC/1023.0);
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
  float vpp = measureVpp(currentFreq);
  float error = targetVpp - vpp;
  integral += error;
  float derivative = error - lastError;
  lastError = error;
  currentFreq -= (Kp * error + Ki * integral + Kd * derivative);
  if (currentFreq < 0.1) currentFreq = 0.1;
  if (currentFreq > 2000) currentFreq = 2000;

  static int count = 0;
  if (++count % 10 == 0) {
    float ratio = vpp / VCC;
    if (ratio < 0.99) {
      float artanh_val = 0.5 * log((1.0 + ratio) / (1.0 - ratio));
      float tau = 1.0 / (4.0 * currentFreq * artanh_val);
      float measR = (tau - (R0+R1)*(C1+C2)) / C2;
      updateKalman(measR);
      Serial.print("Freq: "); Serial.print(currentFreq);
      Serial.print(" Hz, R2_est: "); Serial.println(x_state[0]);
    }
  }
  delay(100);
}
