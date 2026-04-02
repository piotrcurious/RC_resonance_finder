/*
 * Optimized solution for RC Resistance Measurement using 2nd-order RC ladder theory.
 * Circuit: Pin (R0=250) -- R1(1) --+-- R2(Unknown) --+-- A0
 *                                  |                 |
 *                                 C1(0.5uF)         C2(10uF)
 *                                  |                 |
 *                                 GND               GND
 *
 * This version uses a Frequency-Sweep/Binary Search to find the frequency where
 * the gain is 0.5 (half-voltage). This is a stable target for estimation.
 * R2 is calculated by solving the quadratic equation derived from the 2nd-order transfer function.
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

// Kalman Filter variables
float P_cov = 1000000.0; // Estimate error covariance
float Q_proc = 100.0;     // Process noise covariance (assume R2 might drift)
float R_meas = 500.0;    // Measurement noise covariance (in terms of R2 estimate variance)

// Time tracking
unsigned long lastMeasureTime = 0;
const unsigned long INTERVAL = 60000; // 60 seconds

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  Serial.begin(9600);
  Serial.println("RC Resistance Meter Starting...");
}

// Helper to handle delays longer than 16383us safely (common limit for delayMicroseconds)
void safeDelayMicros(unsigned long us) {
  if (us > 16000) {
    delay(us / 1000);
    delayMicroseconds(us % 1000);
  } else {
    delayMicroseconds(us);
  }
}

// Generate a square wave and measure Vpp
// Note: Sine-wave gain theory is used as a first-order approximation for Vpp of the square wave.
float measureVpp(float freq) {
  if (freq < 0.1) freq = 0.1;
  unsigned long period = 1000000.0 / freq;
  unsigned long halfPeriod = period / 2;

  float vMax = 0;
  float vMin = 5.0;

  // Spend some time stabilizing and measuring
  unsigned long startTime = millis();
  while (millis() - startTime < 150) { // Measure for 150ms
    digitalWrite(PIN_OUT, HIGH);
    safeDelayMicros(halfPeriod);
    float v = analogRead(PIN_IN) * (VCC / 1023.0);
    if (v > vMax) vMax = v;

    digitalWrite(PIN_OUT, LOW);
    safeDelayMicros(halfPeriod);
    v = analogRead(PIN_IN) * (VCC / 1023.0);
    if (v < vMin) vMin = v;

    if (period > 200000) break; // Don't hang on extremely low frequencies
  }

  return vMax - vMin;
}

// Solve for R2 using the 2nd-order transfer function gain formula
float solveForR2(float freq, float vpp) {
  if (vpp <= 0.1) return 1000000.0; // Avoid division by zero/extreme values
  float gain = vpp / VCC;
  if (gain > 0.99) return 1.0; // Very low R2 or freq

  float w = 2.0 * PI * freq;
  float Ra = R0 + R1;

  // From |H(jw)| = 1 / sqrt((1 - w^2 Ra R2 C1 C2)^2 + (w (R2 C2 + Ra C1 + Ra C2))^2)
  // We get a quadratic in R2: AR^2 + BR + C = 0
  float a_coeff = w * w * Ra * C1 * C2;
  float b_coeff = w * C2;
  float c_coeff = w * Ra * (C1 + C2);
  float ratio_sq = 1.0 / (gain * gain);

  float A = a_coeff * a_coeff + b_coeff * b_coeff;
  float B = 2.0 * b_coeff * c_coeff - 2.0 * a_coeff;
  float C = 1.0 + c_coeff * c_coeff - ratio_sq;

  float delta = B * B - 4.0 * A * C;
  if (delta < 0) return estimatedR2; // Something went wrong, keep old estimate

  float r2 = (-B + sqrt(delta)) / (2.0 * A);
  return r2;
}

void updateKalman(float measurement) {
  // Predict
  P_cov = P_cov + Q_proc;

  // Update
  float K = P_cov / (P_cov + R_meas);
  estimatedR2 = estimatedR2 + K * (measurement - estimatedR2);
  P_cov = (1.0 - K) * P_cov;
}

void loop() {
  unsigned long now = millis();
  if (now - lastMeasureTime >= INTERVAL || lastMeasureTime == 0) {
    lastMeasureTime = now;

    // Binary search for frequency that gives Gain ~ 0.5 (Vpp ~ 2.5V)
    // Range selected to cover 100 ohms to 100k ohms approx
    float fLow = 0.5;
    float fHigh = 1000.0;
    float targetVpp = 2.5;

    for (int i = 0; i < 12; i++) {
      float fMid = (fLow + fHigh) / 2.0;
      float vpp = measureVpp(fMid);
      if (vpp > targetVpp) {
        fLow = fMid;
      } else {
        fHigh = fMid;
      }
    }

    float bestFreq = (fLow + fHigh) / 2.0;
    float measuredVpp = measureVpp(bestFreq);

    // Calculate R2 based on this best frequency
    float currentR2 = solveForR2(bestFreq, measuredVpp);

    // Update estimate with Kalman Filter
    updateKalman(currentR2);

    Serial.print("Freq: "); Serial.print(bestFreq);
    Serial.print(" Hz, Vpp: "); Serial.print(measuredVpp);
    Serial.print(" V, Measured R2: "); Serial.print(currentR2);
    Serial.print(" Ohms, Filtered R2: "); Serial.println(estimatedR2);
  }
}
