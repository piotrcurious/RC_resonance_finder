#include <Arduino.h>
#include <math.h>

// ============================================================
// ESP32 RC estimator with multi-point curve fitting
// ------------------------------------------------------------
// Improvements over the previous version:
// 1) Uses several ADC samples during one charge pulse
// 2) Fits the exponential curve instead of using one point
// 3) Repeats measurements and rejects weak fits
// 4) Kalman filter smooths the fitted time constant
// 5) Seeds each new run from the last estimate
//
// Physical model:
//   Vc(t) = V0 + (VCC - V0) * (1 - exp(-t / tau))
//
// Rearranged:
//   y(t) = ln((VCC - Vc(t)) / (VCC - V0)) = -t / tau
//
// So if we fit y = m*t, then tau = -1/m
//
// Resistance estimate:
//   tau = R_total * C2
//   R_unknown ≈ tau / C2 - R_source - R1
//
// This is still an approximation because the first RC stage
// influences the edge shape, but for C1 << C2 it is useful.
// ============================================================

// ----------------------------
// Pins
// ----------------------------
constexpr int OUT_PIN   = 26;
constexpr int SENSE_PIN = 34;

// ----------------------------
// Assumptions / constants
// ----------------------------
constexpr float VCC_VOLTS = 3.3f;
constexpr float I_MAX_A = 0.020f;
constexpr float R_SOURCE_OHM = VCC_VOLTS / I_MAX_A;

constexpr float R1_OHM = 1.0f;
constexpr float C1_F   = 0.5e-6f;
constexpr float C2_F   = 10.0e-6f;

constexpr float TAU1_US = (R1_OHM * C1_F) * 1e6f;

// Run once every 60 seconds
constexpr uint32_t UPDATE_PERIOD_MS = 60000UL;

// Pulse limits
constexpr float MIN_PULSE_US = 50.0f;
constexpr float MAX_PULSE_US = 2000000.0f;

// Persistent state
RTC_DATA_ATTR float lastTauUs = NAN;
RTC_DATA_ATTR float lastR2Ohm  = NAN;
RTC_DATA_ATTR uint32_t lastRunMs = 0;

// ----------------------------
// Utility helpers
// ----------------------------
static float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static uint32_t clampu32(uint32_t x, uint32_t lo, uint32_t hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static void waitMicros(uint32_t us) {
  while (us >= 1000UL) {
    delay(us / 1000UL);
    us %= 1000UL;
  }
  if (us > 0) delayMicroseconds(us);
}

static float safeLogRatio(float num, float den) {
  if (num <= 0.0f || den <= 0.0f) return NAN;
  float r = num / den;
  if (r <= 0.0f || r >= 1.0f) return NAN;
  return logf(r);
}

static float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// ----------------------------
// ADC helpers
// ----------------------------
static float readVoltageAvg(int pin, uint8_t samples = 12) {
  // throw away first conversion after mux activity
  (void)analogReadMilliVolts(pin);

  long mvSum = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    mvSum += analogReadMilliVolts(pin);
    delayMicroseconds(60);
  }
  return (mvSum / (float)samples) / 1000.0f;
}

static void dischargeNode(uint32_t us) {
  digitalWrite(OUT_PIN, LOW);
  waitMicros(us);
}

static uint32_t dischargeTimeFromSeed(float tauUs) {
  float us = isnan(tauUs) ? 50000.0f : tauUs * 5.0f;
  return clampu32((uint32_t)us, 5000UL, 2000000UL);
}

// ----------------------------
// Curve fit result
// ----------------------------
struct TauFitResult {
  bool valid = false;
  float tauUs = NAN;
  float sigmaUs = NAN;
  float slope = NAN;
  float r2 = NAN;
  uint8_t usedPoints = 0;
};

// Fit y = m*t through the origin, where:
//   y = ln((VCC - Vc) / (VCC - V0))
//   m = -1/tau
static TauFitResult fitTauFromSamples(const float *tUs, const float *v, uint8_t n, float v0) {
  TauFitResult out;
  if (n < 3) return out;

  const float denV = VCC_VOLTS - v0;
  if (denV <= 0.02f) return out;

  float sum_tt = 0.0f;
  float sum_ty = 0.0f;
  float sum_yy = 0.0f;

  float y[8];
  if (n > 8) n = 8;

  uint8_t good = 0;
  for (uint8_t i = 0; i < n; ++i) {
    float numV = VCC_VOLTS - v[i];
    float ly = safeLogRatio(numV, denV);
    if (isnan(ly) || ly >= 0.0f) continue;

    y[good] = ly;
    sum_tt += tUs[i] * tUs[i];
    sum_ty += tUs[i] * ly;
    sum_yy += ly * ly;
    good++;
  }

  out.usedPoints = good;
  if (good < 3 || sum_tt <= 0.0f) return out;

  float m = sum_ty / sum_tt;
  if (!(m < 0.0f)) return out;

  float tauUs = -1.0f / m;
  if (!(tauUs > 0.0f)) return out;

  // Residuals and R^2-like score
  float ss_res = 0.0f;
  float ss_tot = 0.0f;
  float y_mean = 0.0f;
  for (uint8_t i = 0; i < good; ++i) y_mean += y[i];
  y_mean /= good;

  // Reconstruct and evaluate residuals
  uint8_t k = 0;
  for (uint8_t i = 0; i < n; ++i) {
    float numV = VCC_VOLTS - v[i];
    float ly = safeLogRatio(numV, denV);
    if (isnan(ly) || ly >= 0.0f) continue;

    float pred = m * tUs[i];
    float e = ly - pred;
    ss_res += e * e;
    ss_tot += (ly - y_mean) * (ly - y_mean);
    k++;
  }

  float rms = sqrtf(ss_res / (float)good);
  float r2 = (ss_tot > 1e-12f) ? (1.0f - ss_res / ss_tot) : 0.0f;

  out.valid = true;
  out.tauUs = tauUs;
  out.slope = m;
  out.sigmaUs = fmaxf(100.0f, rms * tauUs * tauUs); // rough uncertainty mapping
  out.r2 = r2;
  return out;
}

// ----------------------------
// Capture a pulse and sample the charging curve
// ----------------------------
static TauFitResult measureCurveForPulse(uint32_t pulseUs) {
  // Sample positions within the pulse.
  // More points near the middle/late part help identify tau.
  constexpr uint8_t N = 6;
  const float frac[N] = {0.12f, 0.24f, 0.40f, 0.58f, 0.76f, 0.92f};

  float tUs[N];
  float v[N];

  // Start from a discharged node.
  dischargeNode(dischargeTimeFromSeed(lastTauUs));

  float v0 = readVoltageAvg(SENSE_PIN, 16);

  uint32_t tStart = micros();
  digitalWrite(OUT_PIN, HIGH);

  for (uint8_t i = 0; i < N; ++i) {
    uint32_t targetOffsetUs = (uint32_t)clampf(pulseUs * frac[i], MIN_PULSE_US, MAX_PULSE_US);

    while ((uint32_t)(micros() - tStart) < targetOffsetUs) {
      // busy wait for timing
    }

    tUs[i] = (float)((uint32_t)(micros() - tStart));
    v[i] = readVoltageAvg(SENSE_PIN, 8);
  }

  digitalWrite(OUT_PIN, LOW);

  TauFitResult fit = fitTauFromSamples(tUs, v, N, v0);

  // Reject obviously weak fits.
  if (fit.valid) {
    if (fit.r2 < 0.80f || fit.tauUs <= 0.0f || fit.tauUs > 5000000.0f) {
      fit.valid = false;
    }
  }

  return fit;
}

// ----------------------------
// Adaptive pulse search
// ----------------------------
static uint32_t chooseSeedPulseUs() {
  if (isnan(lastTauUs)) return 20000UL;
  float p = lastTauUs * 1.25f;
  return clampu32((uint32_t)p, (uint32_t)MIN_PULSE_US, (uint32_t)MAX_PULSE_US);
}

static TauFitResult estimateTau() {
  // Try several pulse widths around the last estimate and keep the best fit.
  uint32_t seed = chooseSeedPulseUs();

  const float probes[5] = {0.65f, 0.85f, 1.00f, 1.20f, 1.50f};

  TauFitResult best;
  float bestScore = -1e9f;

  for (int pass = 0; pass < 2; ++pass) {
    for (int i = 0; i < 5; ++i) {
      uint32_t pulseUs = (uint32_t)clampf(seed * probes[i], MIN_PULSE_US, MAX_PULSE_US);
      TauFitResult r = measureCurveForPulse(pulseUs);

      if (!r.valid) continue;

      // Score favors good fit quality, enough points, and stable tau.
      float score = 100.0f * r.r2 + 2.0f * r.usedPoints - 0.0001f * r.sigmaUs;

      if (score > bestScore) {
        bestScore = score;
        best = r;
      }
    }

    // If nothing good was found, widen the search for the second pass.
    seed = clampu32((uint32_t)(seed * 2.0f), (uint32_t)MIN_PULSE_US, (uint32_t)MAX_PULSE_US);
  }

  return best;
}

// ----------------------------
// Kalman filter
// State: [tau_us, drift_us_per_sec]
// ----------------------------
struct KalmanTau {
  bool initialized = false;
  float x0 = 0.0f;
  float x1 = 0.0f;
  float P00 = 1e6f, P01 = 0.0f, P10 = 0.0f, P11 = 1e5f;

  void init(float tauUs) {
    initialized = true;
    x0 = tauUs;
    x1 = 0.0f;
    P00 = 1e5f;
    P01 = P10 = 0.0f;
    P11 = 1e5f;
  }

  void predict(float dtSec) {
    if (!initialized || dtSec <= 0.0f) return;

    x0 += x1 * dtSec;

    float p00 = P00 + dtSec * (P10 + P01) + dtSec * dtSec * P11;
    float p01 = P01 + dtSec * P11;
    float p10 = P10 + dtSec * P11;
    float p11 = P11;

    // Process noise
    float qTau  = 50.0f * dtSec;
    float qRate = 10.0f * dtSec;

    P00 = p00 + qTau;
    P01 = p01;
    P10 = p10;
    P11 = p11 + qRate;
  }

  void update(float zTauUs, float sigmaUs) {
    if (!initialized) {
      init(zTauUs);
      return;
    }

    float Rm = fmaxf(sigmaUs * sigmaUs, 2500.0f);

    float y = zTauUs - x0;
    float S = P00 + Rm;
    if (S <= 0.0f) return;

    // Outlier gate
    float nis = (y * y) / S;
    if (nis > 25.0f) return;

    float K0 = P00 / S;
    float K1 = P10 / S;

    float oP00 = P00, oP01 = P01, oP10 = P10, oP11 = P11;

    x0 += K0 * y;
    x1 += K1 * y;

    P00 = (1.0f - K0) * oP00;
    P01 = (1.0f - K0) * oP01;
    P10 = oP10 - K1 * oP00;
    P11 = oP11 - K1 * oP01;

    float sym = 0.5f * (P01 + P10);
    P01 = sym;
    P10 = sym;
  }

  bool converged() const {
    return initialized && fabsf(x1) < 1.0f && P00 < 1e4f;
  }
};

KalmanTau kf;

// ----------------------------
// Main estimation cycle
// ----------------------------
static void runResistanceCheck() {
  uint32_t nowMs = millis();
  float dtSec = (lastRunMs == 0) ? 0.0f : (nowMs - lastRunMs) / 1000.0f;
  lastRunMs = nowMs;

  TauFitResult fit = estimateTau();
  if (!fit.valid) {
    Serial.println("Measurement failed: no reliable exponential fit.");
    return;
  }

  if (!kf.initialized) {
    kf.init(fit.tauUs);
  } else {
    kf.predict(dtSec);
    kf.update(fit.tauUs, fit.sigmaUs);
  }

  float tauFiltUs = kf.x0;

  // Convert tau to resistance
  float rTotalOhm = (tauFiltUs * 1e-6f) / C2_F;
  float rUnknownOhm = rTotalOhm - R_SOURCE_OHM - R1_OHM;
  if (rUnknownOhm < 0.0f) rUnknownOhm = 0.0f;

  lastTauUs = tauFiltUs;
  lastR2Ohm = rUnknownOhm;

  Serial.println();
  Serial.println("=== RC estimate ===");
  Serial.printf("Stage-1 tau assumption:  %.3f us\n", TAU1_US);
  Serial.printf("Raw tau fit:             %.3f us\n", fit.tauUs);
  Serial.printf("Fit sigma:               %.3f us\n", fit.sigmaUs);
  Serial.printf("Fit slope:               %.8f 1/us\n", fit.slope);
  Serial.printf("Fit R2 score:            %.4f\n", fit.r2);
  Serial.printf("Used points:             %u\n", fit.usedPoints);
  Serial.printf("Filtered tau:            %.3f us\n", tauFiltUs);
  Serial.printf("Total series R:          %.3f ohm\n", rTotalOhm);
  Serial.printf("Model source resistance:  %.3f ohm\n", R_SOURCE_OHM);
  Serial.printf("Known R1:                %.3f ohm\n", R1_OHM);
  Serial.printf("Unknown R2 estimate:     %.3f ohm\n", rUnknownOhm);
  Serial.printf("Tau drift rate:          %.4f us/s\n", kf.x1);
  Serial.printf("Converged:               %s\n", kf.converged() ? "yes" : "no");
}

// ----------------------------
// Arduino entry points
// ----------------------------
void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);

  pinMode(SENSE_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(SENSE_PIN, ADC_11db);

  Serial.println("ESP32 RC estimator started.");
  Serial.printf("Source resistance model: %.3f ohm\n", R_SOURCE_OHM);
  Serial.printf("Known stage-1 tau:       %.3f us\n", TAU1_US);

  runResistanceCheck();
  lastRunMs = millis();
}

void loop() {
  if ((millis() - lastRunMs) >= UPDATE_PERIOD_MS) {
    runResistanceCheck();
  }
  delay(50);
}
