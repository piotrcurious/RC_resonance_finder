#include <Arduino.h>
#include <math.h>
#include <esp_timer.h>

// ============================================================
// Dual-node ESP32 RC estimator, improved
// ------------------------------------------------------------
// What it does:
//   1) Drives a step on OUT_PIN
//   2) Measures charge curves on two ADC nodes
//   3) Fits exponential response to each node
//   4) Estimates tau1 and tau2 separately
//   5) Uses tau1 to refine source resistance
//   6) Uses tau2 and refined source resistance to estimate R2
//
// Physical model:
//   Vc(t) = V0 + (VCC - V0) * (1 - exp(-t / tau))
//
// Linearized:
//   ln((VCC - Vc) / (VCC - V0)) = a + b*t
//   with b ≈ -1/tau
//
// Then:
//   tau = -1 / b
//
// Resistance relations:
//   tau1 = (Rsource + R1) * C1
//   Rsource = tau1 / C1 - R1
//
//   tau2 = (Rsource + R1 + R2) * C2
//   R2 = tau2 / C2 - Rsource - R1
//
// Notes:
//   - RC networks do not have true resonance.
//   - This code estimates time constants from step response.
//   - Stage 1 can be too fast for the ADC; if so, the code
//     detects weak fit quality and falls back to the last
//     trusted source-resistance estimate.
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

// A rough prior from a 20 mA source limit.
constexpr float R_SOURCE_PRIOR_OHM = VCC_VOLTS / 0.020f;

// Known stage 1
constexpr float R1_OHM = 1.0f;
constexpr float C1_F   = 0.5e-6f;

// Known stage 2 capacitor
constexpr float C2_F   = 10.0e-6f;

// Measurement cadence
constexpr uint32_t UPDATE_PERIOD_MS = 60000UL;

// Sample pattern across one pulse
constexpr uint8_t SAMPLE_COUNT = 7;
constexpr float SAMPLE_FRAC[SAMPLE_COUNT] = {
  0.10f, 0.20f, 0.35f, 0.52f, 0.70f, 0.86f, 0.96f
};

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

static int64_t nowUs() {
  return esp_timer_get_time();
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

static void sortFloat3(float &a, float &b, float &c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
}

static float median3(float a, float b, float c) {
  sortFloat3(a, b, c);
  return b;
}

static float readVoltageAvg(int pin, uint8_t samples = 16) {
  // Throw away first conversion after pin activity.
  (void)analogReadMilliVolts(pin);

  long mvSum = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    mvSum += analogReadMilliVolts(pin);
    delayMicroseconds(60);
  }
  return (mvSum / (float)samples) / 1000.0f;
}

static uint32_t dischargeTimeFromSeed(float tauUs) {
  // Ensure the node is reset before each measurement.
  // Faster stage gets a shorter discharge, slower stage a longer one.
  float us = isnan(tauUs) ? 50000.0f : tauUs * 5.0f;
  return clampu32((uint32_t)us, 5000UL, 2000000UL);
}

// ----------------------------
// Fit result
// ----------------------------
struct FitResult {
  bool valid = false;
  float tauUs = NAN;
  float slope = NAN;
  float intercept = NAN;
  float sigmaUs = NAN;
  float r2 = NAN;
  float rmse = NAN;
  uint8_t usedPoints = 0;
};

// Linear fit y = a + b*x
// Here:
//   x = time_us
//   y = ln((VCC - Vc)/(VCC - V0))
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
    if (ly >= 0.0f) continue;   // charging curve should be negative
    x[good] = tUs[i];
    y[good] = ly;
    good++;
  }

  out.usedPoints = good;
  if (good < 3) return out;

  auto doFit = [&](float *xx, float *yy, uint8_t m) -> FitResult {
    FitResult r;
    float sumX = 0.0f, sumY = 0.0f, sumXX = 0.0f, sumXY = 0.0f, sumYY = 0.0f;
    for (uint8_t i = 0; i < m; ++i) {
      sumX  += xx[i];
      sumY  += yy[i];
      sumXX += xx[i] * xx[i];
      sumXY += xx[i] * yy[i];
      sumYY += yy[i] * yy[i];
    }

    float nf = (float)m;
    float denom = nf * sumXX - sumX * sumX;
    if (fabsf(denom) < 1e-9f) return r;

    float slope = (nf * sumXY - sumX * sumY) / denom;
    float intercept = (sumY - slope * sumX) / nf;
    if (!(slope < 0.0f)) return r;

    float tauUs = -1.0f / slope;
    if (!(tauUs > 0.0f)) return r;

    float yMean = sumY / nf;
    float ssRes = 0.0f;
    float ssTot = 0.0f;
    for (uint8_t i = 0; i < m; ++i) {
      float pred = intercept + slope * xx[i];
      float e = yy[i] - pred;
      ssRes += e * e;
      float d = yy[i] - yMean;
      ssTot += d * d;
    }

    float r2 = (ssTot > 1e-12f) ? (1.0f - ssRes / ssTot) : 0.0f;
    float rmse = sqrtf(ssRes / nf);

    r.valid = true;
    r.tauUs = tauUs;
    r.slope = slope;
    r.intercept = intercept;
    r.r2 = r2;
    r.rmse = rmse;
    // Rough uncertainty model: larger RMSE -> larger tau uncertainty.
    r.sigmaUs = fmaxf(100.0f, rmse * tauUs * tauUs);
    return r;
  };

  // First fit
  FitResult base = doFit(x, y, good);
  if (!base.valid) return out;

  // One-pass outlier rejection: remove worst point if residual is clearly bad.
  if (good >= 4) {
    float residuals[16];
    float worstAbs = -1.0f;
    int worstIdx = -1;
    for (uint8_t i = 0; i < good; ++i) {
      float pred = base.intercept + base.slope * x[i];
      residuals[i] = y[i] - pred;
      float a = fabsf(residuals[i]);
      if (a > worstAbs) {
        worstAbs = a;
        worstIdx = i;
      }
    }

    // If one point is much worse than the rest, refit without it.
    if (worstIdx >= 0 && worstAbs > 3.5f * base.rmse) {
      float xx[16], yy[16];
      uint8_t m = 0;
      for (uint8_t i = 0; i < good; ++i) {
        if ((int)i == worstIdx) continue;
        xx[m] = x[i];
        yy[m] = y[i];
        m++;
      }
      FitResult refit = doFit(xx, yy, m);
      if (refit.valid && refit.r2 >= base.r2) {
        refit.usedPoints = m;
        return refit;
      }
    }
  }

  return base;
}

// ----------------------------
// Scalar Kalman filter
// ----------------------------
struct ScalarKalman {
  bool initialized = false;
  float x = 0.0f;
  float P = 1e6f;

  void init(float initialX) {
    initialized = true;
    x = initialX;
    P = 1e5f;
  }

  void predict(float q) {
    if (!initialized) return;
    P += q;
  }

  void update(float z, float r) {
    if (!initialized) {
      init(z);
      return;
    }
    float S = P + r;
    if (S <= 0.0f) return;

    float innov = z - x;
    float nis = (innov * innov) / S;
    if (nis > 25.0f) return;   // reject clear outlier

    float K = P / S;
    x += K * innov;
    P = (1.0f - K) * P;
  }

  bool converged() const {
    return initialized && P < 1e4f;
  }
};

struct TrendKalman {
  bool initialized = false;
  float x0 = 0.0f;  // value
  float x1 = 0.0f;  // drift
  float P00 = 1e6f, P01 = 0.0f, P10 = 0.0f, P11 = 1e5f;

  void init(float value) {
    initialized = true;
    x0 = value;
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

    float qVal  = 50.0f * dtSec;
    float qDrift = 10.0f * dtSec;

    P00 = p00 + qVal;
    P01 = p01;
    P10 = p10;
    P11 = p11 + qDrift;
  }

  void update(float z, float sigma) {
    if (!initialized) {
      init(z);
      return;
    }

    float R = fmaxf(sigma * sigma, 2500.0f);
    float y = z - x0;
    float S = P00 + R;
    if (S <= 0.0f) return;

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
};

static TrendKalman tau1Filter;
static TrendKalman tau2Filter;
static ScalarKalman rsourceFilter;
static ScalarKalman r2Filter;

// ----------------------------
// Curve sampling
// ----------------------------
static FitResult measureTauOnPin(int pin, float seedTauUs, uint32_t minPulseUs, uint32_t maxPulseUs) {
  uint32_t pulseUs = clampu32((uint32_t)(seedTauUs * 3.0f), minPulseUs, maxPulseUs);
  uint32_t dischargeUs = dischargeTimeFromSeed(seedTauUs);

  float tUs[SAMPLE_COUNT];
  float v[SAMPLE_COUNT];

  // Reset node.
  digitalWrite(OUT_PIN, LOW);
  waitMicros(dischargeUs);

  float v0 = readVoltageAvg(pin, 16);

  int64_t tStart = nowUs();
  digitalWrite(OUT_PIN, HIGH);

  for (uint8_t i = 0; i < SAMPLE_COUNT; ++i) {
    uint32_t targetUs = (uint32_t)clampf(pulseUs * SAMPLE_FRAC[i], minPulseUs, maxPulseUs);
    while ((uint32_t)(nowUs() - tStart) < targetUs) {
      // busy wait for timing
    }
    tUs[i] = (float)((uint32_t)(nowUs() - tStart));
    v[i] = readVoltageAvg(pin, 8);
  }

  digitalWrite(OUT_PIN, LOW);

  FitResult fit = fitExponential(tUs, v, SAMPLE_COUNT, v0);

  // Quality gates.
  if (fit.valid) {
    if (fit.r2 < 0.80f || fit.tauUs <= 0.0f || fit.tauUs > 5000000.0f) {
      fit.valid = false;
    }
  }

  return fit;
}

static FitResult bestOfSeveral(int pin, float seedTauUs, uint32_t minPulseUs, uint32_t maxPulseUs) {
  const float factors[] = {0.60f, 0.85f, 1.00f, 1.25f, 1.60f};
  FitResult best;
  float bestScore = -1e9f;

  for (int pass = 0; pass < 2; ++pass) {
    for (float f : factors) {
      float s = seedTauUs * f;
      FitResult r = measureTauOnPin(pin, s, minPulseUs, maxPulseUs);
      if (!r.valid) continue;

      // Score: favor quality and enough points.
      float score = 100.0f * r.r2 + 2.0f * r.usedPoints - 0.0001f * r.sigmaUs;
      if (score > bestScore) {
        bestScore = score;
        best = r;
      }
    }
    seedTauUs *= 2.0f;
  }

  return best;
}

// ----------------------------
// Main cycle
// ----------------------------
static void runResistanceCheck() {
  uint32_t nowMs = millis();
  float dtSec = (lastRunMs == 0) ? 0.0f : (nowMs - lastRunMs) / 1000.0f;
  lastRunMs = nowMs;

  // ---------- Stage 1 ----------
  float stage1SeedUs;
  if (isnan(lastTau1Us)) {
    stage1SeedUs = (R_SOURCE_PRIOR_OHM + R1_OHM) * C1_F * 1e6f;
  } else {
    stage1SeedUs = lastTau1Us;
  }

  FitResult fit1 = bestOfSeveral(SENSE1_PIN, stage1SeedUs, 20, 5000);

  float rawRsource = NAN;
  bool stage1Trusted = false;

  if (fit1.valid) {
    if (!tau1Filter.initialized) tau1Filter.init(fit1.tauUs);
    else {
      tau1Filter.predict(dtSec);
      tau1Filter.update(fit1.tauUs, fit1.sigmaUs);
    }

    lastTau1Us = tau1Filter.x;
    rawRsource = (lastTau1Us * 1e-6f) / C1_F - R1_OHM;
    if (rawRsource < 0.0f) rawRsource = 0.0f;

    // Stage-1 trust depends on fit quality.
    stage1Trusted = (fit1.r2 >= 0.90f && fit1.usedPoints >= 5);

    if (stage1Trusted) {
      if (!rsourceFilter.initialized) rsourceFilter.init(rawRsource);
      else rsourceFilter.update(rawRsource, fmaxf(0.5f * fit1.sigmaUs / C1_F, 0.5f));
      lastRsourceOhm = rsourceFilter.x;
    } else {
      // Stage 1 is too fast or too noisy; keep the best known prior.
      if (!isnan(lastRsourceOhm)) {
        lastRsourceOhm = 0.85f * lastRsourceOhm + 0.15f * rawRsource;
      } else {
        lastRsourceOhm = rawRsource;
      }
    }
  } else {
    // No valid stage-1 measurement. Use prior or last trusted value.
    if (isnan(lastRsourceOhm)) lastRsourceOhm = R_SOURCE_PRIOR_OHM;
  }

  // ---------- Stage 2 ----------
  float stage2SeedUs;
  if (isnan(lastTau2Us)) {
    stage2SeedUs = 20000.0f;
  } else {
    stage2SeedUs = lastTau2Us;
  }

  FitResult fit2 = bestOfSeveral(SENSE2_PIN, stage2SeedUs, 500, 2000000);

  if (!fit2.valid) {
    Serial.println("Stage 2 measurement failed.");
    return;
  }

  if (!tau2Filter.initialized) tau2Filter.init(fit2.tauUs);
  else {
    tau2Filter.predict(dtSec);
    tau2Filter.update(fit2.tauUs, fit2.sigmaUs);
  }

  lastTau2Us = tau2Filter.x;

  float rawR2 = (lastTau2Us * 1e-6f) / C2_F - lastRsourceOhm - R1_OHM;
  if (rawR2 < 0.0f) rawR2 = 0.0f;

  if (!r2Filter.initialized) r2Filter.init(rawR2);
  else {
    // Rough sigma in ohms from tau uncertainty mapped through C2.
    float sigmaR2 = fmaxf(fit2.sigmaUs * 1e-6f / C2_F, 0.5f);
    r2Filter.update(rawR2, sigmaR2);
  }
  lastR2Ohm = r2Filter.x;

  // ---------- Print results ----------
  Serial.println();
  Serial.println("=== Dual-node RC estimate ===");

  if (fit1.valid) {
    Serial.printf("Stage-1 raw tau:         %.3f us\n", fit1.tauUs);
    Serial.printf("Stage-1 fit R2:          %.4f\n", fit1.r2);
    Serial.printf("Stage-1 fit sigma:       %.3f us\n", fit1.sigmaUs);
    Serial.printf("Stage-1 points:          %u\n", fit1.usedPoints);
    Serial.printf("Stage-1 filtered tau:    %.3f us\n", lastTau1Us);
    Serial.printf("Rsource raw:             %.3f ohm\n", rawRsource);
    Serial.printf("Rsource filtered:        %.3f ohm\n", lastRsourceOhm);
    Serial.printf("Stage-1 trusted:         %s\n", stage1Trusted ? "yes" : "no");
  } else {
    Serial.println("Stage-1: invalid / unresolved; using prior source-resistance estimate.");
    Serial.printf("Rsource used:            %.3f ohm\n", lastRsourceOhm);
  }

  Serial.printf("Stage-2 raw tau:         %.3f us\n", fit2.tauUs);
  Serial.printf("Stage-2 fit R2:          %.4f\n", fit2.r2);
  Serial.printf("Stage-2 fit sigma:       %.3f us\n", fit2.sigmaUs);
  Serial.printf("Stage-2 points:          %u\n", fit2.usedPoints);
  Serial.printf("Stage-2 filtered tau:    %.3f us\n", lastTau2Us);

  Serial.printf("Unknown R2 raw:          %.3f ohm\n", rawR2);
  Serial.printf("Unknown R2 filtered:     %.3f ohm\n", lastR2Ohm);

  Serial.printf("Tau1 drift:              %.4f us/s\n", tau1Filter.x1);
  Serial.printf("Tau2 drift:              %.4f us/s\n", tau2Filter.x1);
  Serial.printf("Rsource converged:       %s\n", rsourceFilter.converged() ? "yes" : "no");
  Serial.printf("R2 converged:            %s\n", r2Filter.converged() ? "yes" : "no");
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
