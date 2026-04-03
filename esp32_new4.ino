#include <Arduino.h>
#include <math.h>

// ============================================================
// Dual-node ESP32 RC estimator
// ------------------------------------------------------------
// Wiring assumption:
//   OUT_PIN   -> drives the series RC network
//   SENSE1_PIN -> node after the first RC section
//   SENSE2_PIN -> node after the second capacitor
//
// The code estimates:
//   tau1 for node 1  -> used to refine source resistance
//   tau2 for node 2  -> used to estimate unknown R2
//
// Physical model used for each node:
//   Vc(t) = V0 + (VCC - V0) * (1 - exp(-t / tau))
//
// Rearranged:
//   ln((VCC - Vc) / (VCC - V0)) = a + b*t
//   where b ≈ -1/tau
//
// Then:
//   tau = -1 / b
//
// Final resistance estimate for the second stage:
//   tau2 = (Rsource + R1 + R2) * C2
//   R2   = tau2 / C2 - Rsource - R1
//
// Rsource is refined from stage 1:
//   tau1 = (Rsource + R1) * C1
//   Rsource = tau1 / C1 - R1
//
// Important practical note:
// This is still a first-order approximation, but it is much better
// than using one ADC point or pretending an RC network has resonance.
// ============================================================

// ----------------------------
// Pins
// ----------------------------
constexpr int OUT_PIN    = 26;
constexpr int SENSE1_PIN = 34;
constexpr int SENSE2_PIN = 35;

// ----------------------------
// Assumptions / constants
// ----------------------------
constexpr float VCC_VOLTS = 3.3f;

// A rough prior from the 20 mA limit.
// The code will refine this from stage-1 measurement.
constexpr float R_SOURCE_PRIOR_OHM = VCC_VOLTS / 0.020f;

// Known stage-1 parts
constexpr float R1_OHM = 1.0f;
constexpr float C1_F   = 0.5e-6f;

// Known stage-2 capacitor
constexpr float C2_F   = 10.0e-6f;

// Run once per minute
constexpr uint32_t UPDATE_PERIOD_MS = 60000UL;

// ----------------------------
// Persistent state
// ----------------------------
RTC_DATA_ATTR float lastTau1Us = NAN;
RTC_DATA_ATTR float lastTau2Us = NAN;
RTC_DATA_ATTR float lastRsourceOhm = NAN;
RTC_DATA_ATTR float lastR2Ohm = NAN;
RTC_DATA_ATTR uint32_t lastRunMs = 0;

// ----------------------------
// Helpers
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

static float readVoltageAvg(int pin, uint8_t samples = 12) {
  // First read after mux changes is often a little dirty.
  (void)analogReadMilliVolts(pin);

  long mvSum = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    mvSum += analogReadMilliVolts(pin);
    delayMicroseconds(60);
  }
  return (mvSum / (float)samples) / 1000.0f;
}

// ----------------------------
// Linear fit result
// ----------------------------
struct FitResult {
  bool valid = false;
  float tauUs = NAN;
  float slope = NAN;
  float intercept = NAN;
  float sigmaUs = NAN;
  float r2 = NAN;
  uint8_t usedPoints = 0;
};

// Fit y = a + b*t where
//   y = ln((VCC - Vc)/(VCC - V0))
//   b ≈ -1/tau
static FitResult fitExponential(const float *tUs, const float *v, uint8_t n, float v0) {
  FitResult out;
  if (n < 3) return out;

  const float denV = VCC_VOLTS - v0;
  if (denV <= 0.02f) return out;

  float x[16];
  float y[16];
  uint8_t good = 0;

  for (uint8_t i = 0; i < n && good < 16; ++i) {
    float numV = VCC_VOLTS - v[i];
    float ly = safeLogRatio(numV, denV);
    if (isnan(ly)) continue;
    if (ly >= 0.0f) continue; // charging curve should produce negative y
    x[good] = tUs[i];
    y[good] = ly;
    good++;
  }

  out.usedPoints = good;
  if (good < 3) return out;

  float sumX = 0.0f, sumY = 0.0f, sumXX = 0.0f, sumXY = 0.0f, sumYY = 0.0f;
  for (uint8_t i = 0; i < good; ++i) {
    sumX  += x[i];
    sumY  += y[i];
    sumXX += x[i] * x[i];
    sumXY += x[i] * y[i];
    sumYY += y[i] * y[i];
  }

  float nF = (float)good;
  float denom = (nF * sumXX - sumX * sumX);
  if (fabsf(denom) < 1e-9f) return out;

  float slope = (nF * sumXY - sumX * sumY) / denom;
  float intercept = (sumY - slope * sumX) / nF;

  if (!(slope < 0.0f)) return out;

  float tauUs = -1.0f / slope;
  if (!(tauUs > 0.0f)) return out;

  // Residual stats and R^2
  float yMean = sumY / nF;
  float ssRes = 0.0f;
  float ssTot = 0.0f;
  for (uint8_t i = 0; i < good; ++i) {
    float pred = intercept + slope * x[i];
    float e = y[i] - pred;
    ssRes += e * e;
    float d = y[i] - yMean;
    ssTot += d * d;
  }

  float r2 = (ssTot > 1e-12f) ? (1.0f - ssRes / ssTot) : 0.0f;
  float rms = sqrtf(ssRes / nF);

  out.valid = true;
  out.tauUs = tauUs;
  out.slope = slope;
  out.intercept = intercept;
  out.r2 = r2;
  out.sigmaUs = fmaxf(100.0f, rms * tauUs * tauUs);
  return out;
}

// ----------------------------
// Kalman filter for tau
// State:
//   x0 = tau_us
//   x1 = drift_us_per_sec
// ----------------------------
struct KalmanTau {
  bool initialized = false;
  float x0 = 0.0f;
  float x1 = 0.0f;

  float P00 = 1e6f;
  float P01 = 0.0f;
  float P10 = 0.0f;
  float P11 = 1e5f;

  void init(float tauUs) {
    initialized = true;
    x0 = tauUs;
    x1 = 0.0f;
    P00 = 1e5f;
    P01 = 0.0f;
    P10 = 0.0f;
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

    // Gate outliers
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

KalmanTau kf1;
KalmanTau kf2;

// ----------------------------
// Sampling profile
// ----------------------------
// These fractions sample early, middle, and late parts of the pulse.
// They are used to reconstruct the exponential curve.
constexpr uint8_t SAMPLE_COUNT = 7;
constexpr float SAMPLE_FRAC[SAMPLE_COUNT] = {
  0.10f, 0.20f, 0.35f, 0.52f, 0.70f, 0.86f, 0.96f
};

struct StageMeasureConfig {
  uint32_t minPulseUs;
  uint32_t maxPulseUs;
  uint32_t minDischargeUs;
  uint32_t maxDischargeUs;
};

static FitResult measureTauOnPin(int pin, float seedTauUs, const StageMeasureConfig &cfg) {
  // Pulse width chosen from the previous estimate.
  // Using around 3 tau gives enough curve shape for a solid fit.
  uint32_t pulseUs = clampu32((uint32_t)(seedTauUs * 3.0f), cfg.minPulseUs, cfg.maxPulseUs);
  uint32_t dischargeUs = clampu32((uint32_t)(seedTauUs * 5.0f), cfg.minDischargeUs, cfg.maxDischargeUs);

  float tUs[SAMPLE_COUNT];
  float v[SAMPLE_COUNT];

  // Reset node before measurement.
  digitalWrite(OUT_PIN, LOW);
  waitMicros(dischargeUs);

  // Baseline before the pulse.
  float v0 = readVoltageAvg(pin, 16);

  // Apply step and sample through the rise.
  uint32_t tStart = micros();
  digitalWrite(OUT_PIN, HIGH);

  for (uint8_t i = 0; i < SAMPLE_COUNT; ++i) {
    uint32_t targetUs = (uint32_t)clampf(pulseUs * SAMPLE_FRAC[i], 1.0f, (float)pulseUs);
    while ((uint32_t)(micros() - tStart) < targetUs) {
      // busy wait for timing
    }
    tUs[i] = (float)((uint32_t)(micros() - tStart));
    v[i] = readVoltageAvg(pin, 8);
  }

  digitalWrite(OUT_PIN, LOW);

  FitResult fit = fitExponential(tUs, v, SAMPLE_COUNT, v0);

  // Reject weak fits.
  if (fit.valid) {
    if (fit.r2 < 0.80f || fit.tauUs <= 0.0f || fit.tauUs > 5000000.0f) {
      fit.valid = false;
    }
  }

  return fit;
}

// ----------------------------
// Stage-specific measurements
// ----------------------------
static FitResult measureStage1Tau() {
  // If unknown, the stage-1 tau is usually close to the source + R1 prior.
  float seedTauUs;
  if (isnan(lastTau1Us)) {
    seedTauUs = (R_SOURCE_PRIOR_OHM + R1_OHM) * C1_F * 1e6f;
  } else {
    seedTauUs = lastTau1Us;
  }

  StageMeasureConfig cfg;
  cfg.minPulseUs = 50;
  cfg.maxPulseUs = 5000;
  cfg.minDischargeUs = 500;
  cfg.maxDischargeUs = 20000;

  FitResult fit = measureTauOnPin(SENSE1_PIN, seedTauUs, cfg);

  // Retry once with a longer pulse if the fit is weak.
  if (!fit.valid) {
    seedTauUs *= 1.8f;
    fit = measureTauOnPin(SENSE1_PIN, seedTauUs, cfg);
  }

  return fit;
}

static FitResult measureStage2Tau() {
  float seedTauUs;
  if (isnan(lastTau2Us)) {
    seedTauUs = 20000.0f;
  } else {
    seedTauUs = lastTau2Us;
  }

  StageMeasureConfig cfg;
  cfg.minPulseUs = 1000;
  cfg.maxPulseUs = 2000000;
  cfg.minDischargeUs = 5000;
  cfg.maxDischargeUs = 2000000;

  FitResult fit = measureTauOnPin(SENSE2_PIN, seedTauUs, cfg);

  // Retry with a wider pulse if the fit is weak.
  if (!fit.valid) {
    seedTauUs *= 2.0f;
    fit = measureTauOnPin(SENSE2_PIN, seedTauUs, cfg);
  }

  return fit;
}

// ----------------------------
// Measurement cycle
// ----------------------------
static void runResistanceCheck() {
  uint32_t nowMs = millis();
  float dtSec = (lastRunMs == 0) ? 0.0f : (nowMs - lastRunMs) / 1000.0f;
  lastRunMs = nowMs;

  // Measure stage 1 first.
  FitResult fit1 = measureStage1Tau();
  if (!fit1.valid) {
    Serial.println("Stage 1 measurement failed.");
    return;
  }

  if (!kf1.initialized) kf1.init(fit1.tauUs);
  else {
    kf1.predict(dtSec);
    kf1.update(fit1.tauUs, fit1.sigmaUs);
  }

  float tau1FiltUs = kf1.x0;
  lastTau1Us = tau1FiltUs;

  // Convert stage 1 tau into source resistance estimate.
  // tau1 = (Rsource + R1) * C1
  float rSourceRaw = (tau1FiltUs * 1e-6f) / C1_F - R1_OHM;
  if (rSourceRaw < 0.0f) rSourceRaw = 0.0f;

  // Smooth the source resistance estimate across runs.
  if (isnan(lastRsourceOhm)) lastRsourceOhm = rSourceRaw;
  else lastRsourceOhm = 0.75f * lastRsourceOhm + 0.25f * rSourceRaw;

  // Measure stage 2.
  FitResult fit2 = measureStage2Tau();
  if (!fit2.valid) {
    Serial.println("Stage 2 measurement failed.");
    return;
  }

  if (!kf2.initialized) kf2.init(fit2.tauUs);
  else {
    kf2.predict(dtSec);
    kf2.update(fit2.tauUs, fit2.sigmaUs);
  }

  float tau2FiltUs = kf2.x0;
  lastTau2Us = tau2FiltUs;

  // Final resistance estimate for the unknown resistor.
  // tau2 = (Rsource + R1 + R2) * C2
  float r2Raw = (tau2FiltUs * 1e-6f) / C2_F - lastRsourceOhm - R1_OHM;
  if (r2Raw < 0.0f) r2Raw = 0.0f;

  if (isnan(lastR2Ohm)) lastR2Ohm = r2Raw;
  else lastR2Ohm = 0.70f * lastR2Ohm + 0.30f * r2Raw;

  Serial.println();
  Serial.println("=== Dual-node RC estimate ===");
  Serial.printf("Stage-1 raw tau:         %.3f us\n", fit1.tauUs);
  Serial.printf("Stage-1 fit sigma:       %.3f us\n", fit1.sigmaUs);
  Serial.printf("Stage-1 fit R2:          %.4f\n", fit1.r2);
  Serial.printf("Stage-1 filtered tau:    %.3f us\n", tau1FiltUs);
  Serial.printf("Rsource raw:             %.3f ohm\n", rSourceRaw);
  Serial.printf("Rsource filtered:        %.3f ohm\n", lastRsourceOhm);

  Serial.printf("Stage-2 raw tau:         %.3f us\n", fit2.tauUs);
  Serial.printf("Stage-2 fit sigma:       %.3f us\n", fit2.sigmaUs);
  Serial.printf("Stage-2 fit R2:          %.4f\n", fit2.r2);
  Serial.printf("Stage-2 filtered tau:    %.3f us\n", tau2FiltUs);

  Serial.printf("Unknown R2 raw:          %.3f ohm\n", r2Raw);
  Serial.printf("Unknown R2 filtered:     %.3f ohm\n", lastR2Ohm);
  Serial.printf("Tau1 drift rate:         %.4f us/s\n", kf1.x1);
  Serial.printf("Tau2 drift rate:         %.4f us/s\n", kf2.x1);
  Serial.printf("Stage-1 converged:       %s\n", kf1.converged() ? "yes" : "no");
  Serial.printf("Stage-2 converged:       %s\n", kf2.converged() ? "yes" : "no");
}

// ----------------------------
// Arduino entry points
// ----------------------------
void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);

  pinMode(SENSE1_PIN, INPUT);
  pinMode(SENSE2_PIN, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(SENSE1_PIN, ADC_11db);
  analogSetPinAttenuation(SENSE2_PIN, ADC_11db);

  Serial.println("ESP32 dual-node RC estimator started.");
  Serial.printf("Source resistance prior: %.3f ohm\n", R_SOURCE_PRIOR_OHM);
  Serial.printf("Known R1:                %.3f ohm\n", R1_OHM);
  Serial.printf("Known C1:                %.6e F\n", C1_F);
  Serial.printf("Known C2:                %.6e F\n", C2_F);

  runResistanceCheck();
  lastRunMs = millis();
}

void loop() {
  if ((millis() - lastRunMs) >= UPDATE_PERIOD_MS) {
    runResistanceCheck();
  }
  delay(50);
}
