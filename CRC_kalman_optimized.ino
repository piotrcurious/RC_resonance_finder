/*
 * Improved solution for RC Resistance Measurement using 2nd-order RC ladder theory
 * and square-wave response analysis.
 * Circuit: Pin (R0=250) -- R1(1) --+-- R2(Unknown) --+-- A0
 *                                  |                 |
 *                                 C1(0.5uF)         C2(10uF)
 *                                  |                 |
 *                                 GND               GND
 *
 * This version uses a Frequency-Sweep/Binary Search with seeding to find the
 * frequency where the gain is approx 0.72 (Vpp ~ 3.6V).
 * This target is more suitable for square-wave response than the 0.5 sine-wave target.
 * R2 is calculated using the square-wave response model Vpp = Vcc * tanh(1 / (4*f*tau)).
 */

#include <math.h>

// Pin Definitions
const int PIN_OUT = 3;
const int PIN_IN = A0;

// Known Constants
const float R0 = 250.0;    // Pin internal resistance
const float R1 = 1.0;      // First stage resistor
const float C1 = 0.5e-6;   // First stage capacitor
const float C2 = 10.0e-6;  // Second stage capacitor
const float VCC = 5.0;

// Variables for measurement and search
float estimatedR2 = 1000.0; // Kalman state: estimate of R2
float lastResonantFreq = 22.0; // Seed for binary search (target frequency)

// Kalman Filter variables (2-state tracking: R2 and dR2/dt)
float x_state[2] = {1000.0, 0.0}; // {R2, dR2}
float P_cov[2][2] = {{1000000.0, 0.0}, {0.0, 1000.0}};
float Q_proc[2][2] = {{10.0, 0.0}, {0.0, 1.0}};
float R_meas = 500.0;

// Time tracking
unsigned long lastMeasureTime = 0;
const unsigned long INTERVAL = 60000; // 60 seconds

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  Serial.begin(9600);
  Serial.println("RC Resistance Meter Starting...");
}

// Helper for delays longer than delayMicroseconds limit
void safeDelayMicros(unsigned long us) {
  if (us > 16000) {
    delay(us / 1000);
    delayMicroseconds(us % 1000);
  } else {
    delayMicroseconds(us);
  }
}

// Measure Vpp with square wave
float measureVpp(float freq) {
  if (freq < 0.05) freq = 0.05;
  unsigned long period = 1000000.0 / freq;
  unsigned long halfPeriod = period / 2;

  float vMax = 0;
  float vMin = 5.0;

  unsigned long startTime = millis();
  while (millis() - startTime < 200) {
    digitalWrite(PIN_OUT, HIGH);
    safeDelayMicros(halfPeriod);
    float v = analogRead(PIN_IN) * (VCC / 1023.0);
    if (v > vMax) vMax = v;

    digitalWrite(PIN_OUT, LOW);
    safeDelayMicros(halfPeriod);
    v = analogRead(PIN_IN) * (VCC / 1023.0);
    if (v < vMin) vMin = v;

    if (period > 400000) break;
  }
  return vMax - vMin;
}

// Solve for R2 using Square Wave model: Vpp = Vcc * tanh(1 / (4*f*RC))
float solveForR2(float freq, float vpp) {
  if (vpp <= 0.1) return 1000000.0;
  float ratio = vpp / VCC;
  if (ratio >= 0.99) return 1.0;

  // artanh(ratio) = 0.5 * ln((1+ratio)/(1-ratio))
  float artanh_val = 0.5 * log((1.0 + ratio) / (1.0 - ratio));
  float tau = 1.0 / (4.0 * freq * artanh_val);

  float Ra = R0 + R1;
  // Refined model: tau_eff = Ra(C1+C2) + R2*C2
  float r2 = (tau - Ra * (C1 + C2)) / C2;

  if (r2 < 0.1) r2 = 0.1;
  return r2;
}

// 2nd-order Kalman filter for R2
void updateKalman(float measurement) {
  // Predict
  x_state[0] = x_state[0] + x_state[1]; // x = x + dx
  P_cov[0][0] += P_cov[1][1] + Q_proc[0][0];
  P_cov[1][1] += Q_proc[1][1];

  // Update
  float K[2];
  float S = P_cov[0][0] + R_meas;
  K[0] = P_cov[0][0] / S;
  K[1] = P_cov[1][0] / S;

  float y = measurement - x_state[0];
  x_state[0] += K[0] * y;
  x_state[1] += K[1] * y;

  float P00_temp = P_cov[0][0];
  P_cov[0][0] = (1.0 - K[0]) * P_cov[0][0];
  P_cov[1][1] = P_cov[1][1] - K[1] * P_cov[0][1];
  P_cov[0][1] = (1.0 - K[0]) * P_cov[0][1];
  P_cov[1][0] = P_cov[0][1];

  estimatedR2 = x_state[0];
}

void loop() {
  unsigned long now = millis();
  if (now - lastMeasureTime >= INTERVAL || lastMeasureTime == 0) {
    lastMeasureTime = now;

    // Binary search for frequency that gives Gain ~ 0.72 (Vpp ~ 3.6V)
    float fLow = lastResonantFreq * 0.5;
    float fHigh = lastResonantFreq * 2.0;
    if (fLow < 0.1) fLow = 0.1;
    if (fHigh > 5000.0) fHigh = 5000.0;

    float targetVpp = 3.6;

    if (measureVpp(fLow) < targetVpp || measureVpp(fHigh) > targetVpp) {
      fLow = 0.05;
      fHigh = 2000.0;
    }

    for (int i = 0; i < 10; i++) {
      float fMid = (fLow + fHigh) / 2.0;
      float vpp = measureVpp(fMid);
      if (vpp > targetVpp) fLow = fMid;
      else fHigh = fMid;
    }

    float bestFreq = (fLow + fHigh) / 2.0;
    lastResonantFreq = bestFreq;
    float measuredVpp = measureVpp(bestFreq);

    float currentR2 = solveForR2(bestFreq, measuredVpp);
    updateKalman(currentR2);

    Serial.print("Freq: "); Serial.print(bestFreq);
    Serial.print(" Hz, Vpp: "); Serial.print(measuredVpp);
    Serial.print(" V, Measured R2: "); Serial.print(currentR2);
    Serial.print(" Ohms, Filtered R2: "); Serial.println(estimatedR2);
  }
}
