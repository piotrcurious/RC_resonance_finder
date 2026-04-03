#include <Arduino.h>
#include <math.h>

// ----------------------------
// Hardware configuration
// ----------------------------
constexpr int OUT_PIN   = 26;   // Digital output driving the RC network
constexpr int SENSE_PIN = 34;   // ADC input measuring capacitor voltage

// ----------------------------
// Assumptions / constants
// ----------------------------
constexpr float VCC_VOLTS = 3.3f;

// The prompt says "internal resistance defined by max current of 20 mA".
// This is only a coarse model of the ESP32 output source impedance.
constexpr float I_MAX_A = 0.020f;
constexpr float R_SOURCE_OHM = VCC_VOLTS / I_MAX_A;

// Known stage-1 components
constexpr float R1_OHM = 1.0f;
constexpr float C1_F   = 0.5e-6f;

// Unknown stage-2 capacitor is known
constexpr float C2_F   = 10.0e-6f;

// Stage-1 time constant, in microseconds.
// tau1 = R1 * C1
constexpr float TAU1_US = R1_OHM * C1_F * 1e6f;

// Search / update cadence
constexpr uint32_t UPDATE_PERIOD_MS = 60000UL;

// Theoretical 63.2% point for a first-order RC charge
constexpr float RC_TARGET_FRACTION = 0.6321205588f;

// ----------------------------
// Persistent state
// ----------------------------
RTC_DATA_ATTR float lastTauUs = NAN;
RTC_DATA_ATTR float lastR2Ohm  = NAN;
RTC_DATA_ATTR uint32_t lastRunMs = 0;

// ----------------------------
// Small utilities
// ----------------------------
static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static uint32_t clampu32(uint32_t x, uint32_t lo, uint32_t hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void waitMicros(uint32_t us) {
  while (us >= 1000UL) {
    uint32_t ms = us / 1000UL;
    delay(ms);
    us -= ms * 1000UL;
  }
  if (us > 0) {
    delayMicroseconds(us);
  }
}

static float readVoltageAvg(int pin, int samples = 8) {
  long mvSum = 0;
  for (int i = 0; i < samples; ++i) {
    mvSum += analogReadMilliVolts(pin);
    delayMicroseconds(80);
  }
  return (mvSum / (float)samples) / 1000.0f;
}

static float median3(float a, float b, float c) {
  float v[3] = {a, b, c};
  int n = 0;
  float w[3];

  for (int i = 0; i < 3; ++i) {
    if (!isnan(v[i])) {
      w[n++] = v[i];
    }
  }

  if (n == 0) return NAN;
  if (n == 1) return w[0];
  if (n == 2) return 0.5f * (w[0] + w[1]);

  // n == 3, sort three values
  if (w[0] > w[1]) { float t = w[0]; w[0] = w[1]; w[1] = t; }
  if (w[1] > w[2]) { float t = w[1]; w[1] = w[2]; w[2] = t; }
  if (w[0] > w[1]) { float t = w[0]; w[0] = w[1]; w[1] = t; }
  return w[1];
}

static uint32_t dischargeTimeUsFromSeed(float tauGuessUs) {
  // Enough to push the node down, but not absurdly long.
  float us = isnan(tauGuessUs) ? 50000.0f : tauGuessUs * 5.0f;
  us = clampf(us, 5000.0f, 2000000.0f); // 5 ms .. 2 s
  return (uint32_t)us;
}

// ----------------------------
// Measurement model
// ----------------------------
// For a charging RC:
//   Vc(t) = V0 + (VCC - V0) * (1 - exp(-t / tau))
//
// Rearranged:
//   tau = -t / ln((VCC - Vc(t)) / (VCC - V0))
//
// This sketch uses that equation on the second capacitor.
// The first RC stage is much faster, so it is treated as a fast correction term.
static float tauFromSampleUs(float pulseUs, float v0, float v1) {
  float denom = VCC_VOLTS - v0;
  float numer = VCC_VOLTS - v1;

  // Valid charge condition:
  //   0 < numer < denom
  if (denom <= 0.01f) return NAN;
  if (numer <= 0.0f)  return NAN;
  if (numer >= denom) return NAN;

  float ratio = numer / denom;
  if (ratio <= 0.0f || ratio >= 1.0f) return NAN;

  return -pulseUs / logf(ratio);
}

static void samplePulse(float pulseUs, float &v0, float &v1) {
  // Force the output low, then wait a short discharge interval.
  // If the previous estimate exists, use it to choose a sensible discharge time.
  digitalWrite(OUT_PIN, LOW);
  waitMicros(dischargeTimeUsFromSeed(lastTauUs));

  // Measure starting voltage.
  v0 = readVoltageAvg(SENSE_PIN, 8);

  // Apply the test pulse.
  digitalWrite(OUT_PIN, HIGH);
  waitMicros((uint32_t)clampf(pulseUs, 1.0f, 2000000.0f));

  // Read the capacitor near the end of the pulse.
  v1 = readVoltageAvg(SENSE_PIN, 8);

  digitalWrite(OUT_PIN, LOW);
}

// ----------------------------
// Kalman filter for tau
// State vector:
//   x[0] = tau_us
//   x[1] = d(tau_us)/dt   [us/s]
//
// Model:
//   x_k = F * x_{k-1} + w
//   z_k = H * x_k + v
//
// where H = [1, 0]
// ----------------------------
struct KalmanTau {
  bool initialized = false;
  float x0 = 0.0f;
  float x1 = 0.0f;

  float P00 = 1e6f;
  float P01 = 0.0f;
  float P10 = 0.0f;
  float P11 = 1e4f;

  void init(float tauUs) {
    initialized = true;
    x0 = tauUs;
    x1 = 0.0f;
    P00 = 1e5f;
    P01 = 0.0f;
    P10 = 0.0f;
    P11 = 1e4f;
  }

  void predict(float dtSec) {
    if (!initialized) return;
    if (dtSec <= 0.0f) return;

    // x = F*x
    x0 = x0 + x1 * dtSec;

    // F = [1 dt; 0 1]
    // P' = F P F^T + Q
    float p00 = P00 + dtSec * (P10 + P01) + dtSec * dtSec * P11;
    float p01 = P01 + dtSec * P11;
    float p10 = P10 + dtSec * P11;
    float p11 = P11;

    // Process noise:
    // tau can drift slowly, rate can drift a bit faster.
    float qTau  = 50.0f * dtSec;
    float qRate = 5.0f * dtSec;

    P00 = p00 + qTau;
    P01 = p01;
    P10 = p10;
    P11 = p11 + qRate;
  }

  void update(float zTauUs) {
    if (!initialized) {
      init(zTauUs);
      return;
    }

    // Measurement noise. Larger tau -> slightly larger tolerance.
    float Rm = fmaxf(100.0f, 0.01f * zTauUs * zTauUs);

    float y = zTauUs - x0;           // innovation
    float S = P00 + Rm;               // innovation covariance
    if (S <= 0.0f) return;

    float K0 = P00 / S;
    float K1 = P10 / S;

    float oldP00 = P00;
    float oldP01 = P01;
    float oldP10 = P10;
    float oldP11 = P11;

    // State update
    x0 += K0 * y;
    x1 += K1 * y;

    // Covariance update: P = (I - K H) P, with H = [1, 0]
    P00 = (1.0f - K0) * oldP00;
    P01 = (1.0f - K0) * oldP01;
    P10 = oldP10 - K1 * oldP00;
    P11 = oldP11 - K1 * oldP01;

    // Keep the matrix reasonably symmetric
    float sym = 0.5f * (P01 + P10);
    P01 = sym;
    P10 = sym;
  }

  bool converged() const {
    // Small drift and low variance means the estimate is stable.
    return initialized && (fabsf(x1) < 1.0f) && (P00 < 1e4f);
  }
};

KalmanTau kf;

// ----------------------------
// Estimation routine
// ----------------------------
static float estimateTauUs() {
  // Seed search window from previous run.
  float seed = isnan(lastTauUs) ? 20000.0f : lastTauUs;

  float lo = clampf(seed * 0.25f, 50.0f, 2000000.0f);
  float hi = clampf(seed * 4.0f, lo * 1.25f, 2000000.0f);

  float v0 = 0.0f, v1 = 0.0f;

  // Ensure the lower bound is below the 63.2% point.
  for (int i = 0; i < 6; ++i) {
    samplePulse(lo, v0, v1);
    float target = v0 + RC_TARGET_FRACTION * (VCC_VOLTS - v0);
    if (v1 < target) break;
    lo = clampf(lo * 0.5f, 20.0f, hi * 0.95f);
  }

  // Ensure the upper bound is above the 63.2% point.
  for (int i = 0; i < 6; ++i) {
    samplePulse(hi, v0, v1);
    float target = v0 + RC_TARGET_FRACTION * (VCC_VOLTS - v0);
    if (v1 >= target) break;
    hi = clampf(hi * 2.0f, lo * 1.05f, 2000000.0f);
  }

  // Binary search for pulse width near the 63.2% point.
  // This is the time constant estimate for a first-order RC response.
  for (int iter = 0; iter < 14; ++iter) {
    float mid = 0.5f * (lo + hi);
    samplePulse(mid, v0, v1);
    float target = v0 + RC_TARGET_FRACTION * (VCC_VOLTS - v0);

    if (v1 < target) {
      lo = mid;
    } else {
      hi = mid;
    }
  }

  float probe = 0.5f * (lo + hi);

  // Refinement step:
  // measure around the binary-search solution and compute tau from each point.
  // Median-of-three reduces the effect of ADC noise.
  float t1, t2, t3;
  float vv0, vv1;

  samplePulse(probe * 0.90f, vv0, vv1);
  t1 = tauFromSampleUs(probe * 0.90f, vv0, vv1);

  samplePulse(probe * 1.00f, vv0, vv1);
  t2 = tauFromSampleUs(probe * 1.00f, vv0, vv1);

  samplePulse(probe * 1.10f, vv0, vv1);
  t3 = tauFromSampleUs(probe * 1.10f, vv0, vv1);

  float refined = median3(t1, t2, t3);
  if (isnan(refined)) {
    refined = probe;
  }

  return refined;
}

static void runResistanceCheck() {
  uint32_t nowMs = millis();
  float dtSec = 0.0f;

  if (lastRunMs != 0) {
    dtSec = (nowMs - lastRunMs) / 1000.0f;
  }
  lastRunMs = nowMs;

  float tauMeasUs = estimateTauUs();

  if (isnan(tauMeasUs) || tauMeasUs <= 0.0f) {
    Serial.println("Measurement failed: tau estimate invalid.");
    return;
  }

  // Kalman tracking of tau
  if (!kf.initialized) {
    kf.init(tauMeasUs);
  } else {
    kf.predict(dtSec);
    kf.update(tauMeasUs);
  }

  float tauFiltUs = kf.x0;

  // Convert tau to resistance:
  //   tau = R_total * C2
  //   R_total = tau / C2
  //
  // Then remove known series terms:
  //   R_unknown = R_total - R_source - R1
  float rTotalOhm = (tauFiltUs * 1e-6f) / C2_F;
  float r2Ohm = rTotalOhm - R_SOURCE_OHM - R1_OHM;
  if (r2Ohm < 0.0f) r2Ohm = 0.0f;

  lastTauUs = tauFiltUs;
  lastR2Ohm = r2Ohm;

  Serial.println();
  Serial.println("=== RC resistance estimate ===");
  Serial.printf("Stage-1 tau assumption: %.3f us\n", TAU1_US);
  Serial.printf("Measured tau (raw):      %.3f us\n", tauMeasUs);
  Serial.printf("Measured tau (Kalman):   %.3f us\n", tauFiltUs);
  Serial.printf("Total series R estimate: %.3f ohm\n", rTotalOhm);
  Serial.printf("Source resistance model: %.3f ohm\n", R_SOURCE_OHM);
  Serial.printf("Known R1:                %.3f ohm\n", R1_OHM);
  Serial.printf("Unknown R2 estimate:     %.3f ohm\n", r2Ohm);
  Serial.printf("Tau drift rate:          %.4f us/s\n", kf.x1);
  Serial.printf("Converged:               %s\n", kf.converged() ? "yes" : "no");
}

// ----------------------------
// Arduino setup / loop
// ----------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);

  pinMode(SENSE_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation((adc1_channel_t)0, ADC_11db); // harmless on many cores
  analogSetPinAttenuation(SENSE_PIN, ADC_11db);

  Serial.println("ESP32 RC estimator started.");
  Serial.printf("Assumed source resistance: %.3f ohm\n", R_SOURCE_OHM);
  Serial.printf("Known stage-1 tau:         %.3f us\n", TAU1_US);

  runResistanceCheck();
  lastRunMs = millis();
}

void loop() {
  if ((millis() - lastRunMs) >= UPDATE_PERIOD_MS) {
    runResistanceCheck();
  }
  delay(50);
}
