#include <Arduino.h>
#include <math.h>
#include <esp_timer.h>

// ============================================================
// Dual-node ESP32 RC estimator with stronger calibration
// ------------------------------------------------------------
// Improvements in this version:
//   - explicit per-node ADC calibration state
//   - gain/offset correction hooks
//   - automatic re-calibration when node baseline drifts
//   - monotonicity checks on capture curves
//   - fallback logic when stage 1 is too fast to resolve
//   - cleaner separation of measurement, fitting, and filtering
//
// Physical model:
//   Vc(t) = V0 + (VCC - V0) * (1 - exp(-t / tau))
//
// Linearized:
//   ln((VCC - Vc) / (VCC - V0)) = a + b*t
//   where b ≈ -1/tau
//
// Stage 1:
//   tau1 = (Rsource + R1) * C1
//   Rsource = tau1 / C1 - R1
//
// Stage 2:
//   tau2 = (Rsource + R1 + R2) * C2
//   R2 = tau2 / C2 - Rsource - R1
// ============================================================

// ----------------------------
// Pins
// ----------------------------
constexpr int OUT_PIN    = 26;
constexpr int SENSE1_PIN = 34;
constexpr int SENSE2_PIN = 35;

// ----------------------------
// Known / assumed constants
// ----------------------------
constexpr float VCC_VOLTS = 3.3f;
constexpr float R_SOURCE_PRIOR_OHM = VCC_VOLTS / 0.020f; // 20 mA prior

constexpr float R1_OHM = 1.0f;
constexpr float C1_F   = 0.5e-6f;
constexpr float C2_F   = 10.0e-6f;

constexpr uint32_t UPDATE_PERIOD_MS = 60000UL;

// Sampling profile for one capture
constexpr uint8_t SAMPLE_COUNT = 8;
constexpr float SAMPLE_FRAC[SAMPLE_COUNT] = {
  0.08f, 0.16f, 0.28f, 0.42f, 0.58f, 0.72f, 0.86f, 0.96f
};

constexpr uint8_t CAPTURES_PER_NODE = 3;

// Calibration thresholds
constexpr float REBASE_THRESHOLD_MV = 18.0f;
constexpr float MIN_VALID_VSPAN_MV   = 50.0f;

// ----------------------------
// Persistent state
// ----------------------------
RTC_DATA_ATTR float lastTau1Us = NAN;
RTC_DATA_ATTR float lastTau2Us = NAN;
RTC_DATA_ATTR float lastRsourceOhm = NAN;
RTC_DATA_ATTR float lastR2Ohm = NAN;
RTC_DATA_ATTR uint32_t lastRunMs = 0;

// ----------------------------
// Utilities
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

static void insertionSort(float *a, uint8_t n) {
  for (uint8_t i = 1; i < n; ++i) {
    float key = a[i];
    int j = (int)i - 1;
    while (j >= 0 && a[j] > key) {
      a[j + 1] = a[j];
      --j;
    }
    a[j + 1] = key;
  }
}

static float medianOfArray(float *arr, uint8_t n) {
  if (n == 0) return NAN;
  insertionSort(arr, n);
  if (n & 1) return arr[n / 2];
  return 0.5f * (arr[n / 2 - 1] + arr[n / 2]);
}

static float readVoltageAvgMv(int pin, uint8_t samples = 16) {
  // analogReadMilliVolts() is the calibrated conversion path on ESP32 Arduino.
  (void)analogReadMilliVolts(pin); // discard first post-mux read

  long mvSum = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    mvSum += analogReadMilliVolts(pin);
    delayMicroseconds(60);
  }
  return mvSum / (float)samples;
}

static uint32_t dischargeTimeFromSeed(float tauUs) {
  float us = isnan(tauUs) ? 50000.0f : tauUs * 5.0f;
  return clampu32((uint32_t)us, 5000UL, 2000000UL);
}

// ----------------------------
// ADC calibration
// ----------------------------
struct AdcCal {
  bool valid = false;
  float zeroOffsetMv = 0.0f;    // discharged baseline
  float gain = 1.0f;            // optional correction factor
  float noiseMv = 5.0f;         // observed noise floor
  float baselineMv = 0.0f;      // last measured baseline
  uint32_t lastUpdateMs = 0;
};

static AdcCal cal1;
static AdcCal cal2;

static float applyAdcCal(float mv, const AdcCal &cal) {
  float x = (mv - cal.zeroOffsetMv) * cal.gain;
  return (x < 0.0f) ? 0.0f : x;
}

static AdcCal calibrateNodeZero(int pin, float dischargeTauUs) {
  AdcCal cal;
  constexpr uint8_t rounds = 12;

  digitalWrite(OUT_PIN, LOW);
  waitMicros(dischargeTimeFromSeed(dischargeTauUs));

  float samples[rounds];
  for (uint8_t i = 0; i < rounds; ++i) {
    samples[i] = readVoltageAvgMv(pin, 12);
    delayMicroseconds(120);
  }

  float tmp[rounds];
  for (uint8_t i = 0; i < rounds; ++i) tmp[i] = samples[i];
  float med = medianOfArray(tmp, rounds);

  float mad = 0.0f;
  for (uint8_t i = 0; i < rounds; ++i) mad += fabsf(samples[i] - med);
  mad /= rounds;

  cal.valid = true;
  cal.zeroOffsetMv = med;
  cal.gain = 1.0f;
  cal.noiseMv = fmaxf(2.0f, 1.4826f * mad);
  cal.baselineMv = med;
  cal.lastUpdateMs = millis();
  return cal;
}

static void maybeRecalibrateNode(int pin, AdcCal &cal, float seedTauUs) {
  // Recalibrate if the baseline drifts by a noticeable amount.
  float currentBaseline = readVoltageAvgMv(pin, 8);

  if (!cal.valid || fabsf(currentBaseline - cal.baselineMv) >= REBASE_THRESHOLD_MV) {
    cal = calibrateNodeZero(pin, seedTauUs);
    return;
  }

  // Slowly update baseline estimate even if we do not fully recapture.
  cal.baselineMv = 0.90f * cal.baselineMv + 0.10f * currentBaseline;
  cal.lastUpdateMs = millis();
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
  float vSpanMv = NAN;
};

static FitResult fitExponential(const float *tUs, const float *vVolts, uint8_t n, float v0Volts) {
  FitResult out;
  if (n < 3) return out;

  const float denV = VCC_VOLTS - v0Volts;
  if (denV <= 0.02f) return out;

  float x[16];
  float y[16];
  uint8_t good = 0;

  for (uint8_t i = 0; i < n && good < 16; ++i) {
    float numV = VCC_VOLTS - vVolts[i];
    float ly = safeLogRatio(numV, denV);
    if (isnan(ly) || ly >= 0.0f) continue;
    x[good] = tUs[i];
    y[good] = ly;
    good++;
  }

  out.usedPoints = good;
  if (good < 3) return out;

  auto doFit = [&](float *xx, float *yy, uint8_t m) -> FitResult {
    FitResult r;
    float sumX = 0.0f, sumY = 0.0f, sumXX = 0.0f, sumXY = 0.0f;
    for (uint8_t i = 0; i < m; ++i) {
      sumX  += xx[i];
      sumY  += yy[i];
      sumXX += xx[i] * xx[i];
      sumXY += xx[i] * yy[i];
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
    r.sigmaUs = fmaxf(100.0f, rmse * tauUs * tauUs);
    return r;
  };

  FitResult base = doFit(x, y, good);
  if (!base.valid) return out;

  if (good >= 4) {
    float worstAbs = -1.0f;
    int worstIdx = -1;
    for (uint8_t i = 0; i < good; ++i) {
      float pred = base.intercept + base.slope * x[i];
      float e = fabsf(y[i] - pred);
      if (e > worstAbs) {
        worstAbs = e;
        worstIdx = (int)i;
      }
    }

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
// Filters
// ----------------------------
struct TrendKalman {
  bool initialized = false;
  float x0 = 0.0f;
  float x1 = 0.0f;
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

    float qVal = 50.0f * dtSec;
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

struct ScalarKalman {
  bool initialized = false;
  float x = 0.0f;
  float P = 1e6f;

  void init(float value) {
    initialized = true;
    x = value;
    P = 1e5f;
  }

  void update(float z, float sigma) {
    if (!initialized) {
      init(z);
      return;
    }

    float R = fmaxf(sigma * sigma, 1.0f);
    float S = P + R;
    if (S <= 0.0f) return;

    float innov = z - x;
    if ((innov * innov) / S > 25.0f) return;

    float K = P / S;
    x += K * innov;
    P = (1.0f - K) * P;
  }
};

static TrendKalman tau1Filter;
static TrendKalman tau2Filter;
static ScalarKalman rsourceFilter;
static ScalarKalman r2Filter;

// ----------------------------
// Capture helpers
// ----------------------------
static FitResult captureOneNode(int pin, const AdcCal &cal, float seedTauUs, uint32_t minPulseUs, uint32_t maxPulseUs) {
  uint32_t pulseUs = clampu32((uint32_t)(seedTauUs * 3.0f), minPulseUs, maxPulseUs);
  uint32_t dischargeUs = dischargeTimeFromSeed(seedTauUs);

  float tUs[SAMPLE_COUNT];
  float vVolts[SAMPLE_COUNT];

  digitalWrite(OUT_PIN, LOW);
  waitMicros(dischargeUs);

  float v0 = applyAdcCal(readVoltageAvgMv(pin, 16), cal) / 1000.0f;

  // Guard against unresolved baseline.
  if ((readVoltageAvgMv(pin, 4) - cal.zeroOffsetMv) > MIN_VALID_VSPAN_MV) {
    // If node is still high, extend discharge once and retry baseline.
    waitMicros(dischargeUs);
    v0 = applyAdcCal(readVoltageAvgMv(pin, 16), cal) / 1000.0f;
  }

  int64_t tStart = nowUs();
  digitalWrite(OUT_PIN, HIGH);

  for (uint8_t i = 0; i < SAMPLE_COUNT; ++i) {
    uint32_t targetUs = (uint32_t)clampf(pulseUs * SAMPLE_FRAC[i], minPulseUs, maxPulseUs);
    while ((uint32_t)(nowUs() - tStart) < targetUs) {
      // timing wait
    }
    tUs[i] = (float)((uint32_t)(nowUs() - tStart));
    vVolts[i] = applyAdcCal(readVoltageAvgMv(pin, 8), cal) / 1000.0f;
  }

  digitalWrite(OUT_PIN, LOW);

  // Reject non-monotonic curves: charge should not collapse randomly.
  int monotonicViolations = 0;
  for (uint8_t i = 1; i < SAMPLE_COUNT; ++i) {
    if (vVolts[i] + 0.015f < vVolts[i - 1]) monotonicViolations++;
  }
  if (monotonicViolations >= 2) {
    FitResult bad;
    return bad;
  }

  FitResult fit = fitExponential(tUs, vVolts, SAMPLE_COUNT, v0);
  if (fit.valid) {
    float vMin = vVolts[0], vMax = vVolts[0];
    for (uint8_t i = 1; i < SAMPLE_COUNT; ++i) {
      if (vVolts[i] < vMin) vMin = vVolts[i];
      if (vVolts[i] > vMax) vMax = vVolts[i];
    }
    fit.vSpanMv = (vMax - vMin) * 1000.0f;

    if (fit.r2 < 0.80f || fit.tauUs <= 0.0f || fit.tauUs > 5000000.0f || fit.vSpanMv < MIN_VALID_VSPAN_MV) {
      fit.valid = false;
    }
  }

  return fit;
}

static FitResult bestOfSeveral(int pin, const AdcCal &cal, float seedTauUs, uint32_t minPulseUs, uint32_t maxPulseUs) {
  const float factors[] = {0.60f, 0.85f, 1.00f, 1.25f, 1.60f};
  FitResult best;
  float bestScore = -1e9f;

  for (int pass = 0; pass < 2; ++pass) {
    for (float f : factors) {
      FitResult r = captureOneNode(pin, cal, seedTauUs * f, minPulseUs, maxPulseUs);
      if (!r.valid) continue;

      float score = 100.0f * r.r2 + 2.0f * r.usedPoints - 0.0001f * r.sigmaUs + 0.01f * r.vSpanMv;
      if (score > bestScore) {
        bestScore = score;
        best = r;
      }
    }
    seedTauUs *= 2.0f;
  }

  return best;
}

static float medianTauAcrossCaptures(int pin, const AdcCal &cal, float seedTauUs, uint32_t minPulseUs, uint32_t maxPulseUs, float &bestSigma, float &bestR2, uint8_t &bestPoints) {
  float taus[CAPTURES_PER_NODE];
  float sigmas[CAPTURES_PER_NODE];
  float r2s[CAPTURES_PER_NODE];
  uint8_t pts[CAPTURES_PER_NODE];
  uint8_t n = 0;

  for (uint8_t i = 0; i < CAPTURES_PER_NODE; ++i) {
    FitResult r = bestOfSeveral(pin, cal, seedTauUs, minPulseUs, maxPulseUs);
    if (!r.valid) continue;
    taus[n] = r.tauUs;
    sigmas[n] = r.sigmaUs;
    r2s[n] = r.r2;
    pts[n] = r.usedPoints;
    n++;
  }

  if (n == 0) {
    bestSigma = NAN;
    bestR2 = NAN;
    bestPoints = 0;
    return NAN;
  }

  float tauMed = medianOfArray(taus, n);

  uint8_t bestIdx = 0;
  for (uint8_t i = 1; i < n; ++i) {
    if (r2s[i] > r2s[bestIdx]) bestIdx = i;
  }

  bestSigma = sigmas[bestIdx];
  bestR2 = r2s[bestIdx];
  bestPoints = pts[bestIdx];
  return tauMed;
}

// ----------------------------
// Measurement cycle
// ----------------------------
static void runResistanceCheck() {
  uint32_t nowMs = millis();
  float dtSec = (lastRunMs == 0) ? 0.0f : (nowMs - lastRunMs) / 1000.0f;
  lastRunMs = nowMs;

  // Update ADC calibration before every run.
  float tau1GuessUs = isnan(lastTau1Us) ? (R_SOURCE_PRIOR_OHM + R1_OHM) * C1_F * 1e6f : lastTau1Us;
  float tau2GuessUs = isnan(lastTau2Us) ? 20000.0f : lastTau2Us;

  maybeRecalibrateNode(SENSE1_PIN, cal1, tau1GuessUs);
  maybeRecalibrateNode(SENSE2_PIN, cal2, tau2GuessUs);

  // -------- Stage 1 --------
  float s1Sigma = NAN, s1R2 = NAN;
  uint8_t s1Pts = 0;
  float tau1Med = medianTauAcrossCaptures(SENSE1_PIN, cal1, tau1GuessUs, 20, 5000, s1Sigma, s1R2, s1Pts);

  bool stage1Trusted = false;
  float rSourceRaw = NAN;

  if (!isnan(tau1Med)) {
    if (!tau1Filter.initialized) tau1Filter.init(tau1Med);
    else {
      tau1Filter.predict(dtSec);
      tau1Filter.update(tau1Med, s1Sigma);
    }

    lastTau1Us = tau1Filter.x;
    rSourceRaw = (lastTau1Us * 1e-6f) / C1_F - R1_OHM;
    if (rSourceRaw < 0.0f) rSourceRaw = 0.0f;

    stage1Trusted = (s1R2 >= 0.90f && s1Pts >= 5);

    if (stage1Trusted) {
      if (!rsourceFilter.initialized) rsourceFilter.init(rSourceRaw);
      else rsourceFilter.update(rSourceRaw, fmaxf(0.5f * s1Sigma * 1e-6f / C1_F, 0.5f));
      lastRsourceOhm = rsourceFilter.x;
    } else {
      if (isnan(lastRsourceOhm)) lastRsourceOhm = rSourceRaw;
      else lastRsourceOhm = 0.85f * lastRsourceOhm + 0.15f * rSourceRaw;
    }
  } else {
    if (isnan(lastRsourceOhm)) lastRsourceOhm = R_SOURCE_PRIOR_OHM;
  }

  // -------- Stage 2 --------
  float s2Sigma = NAN, s2R2 = NAN;
  uint8_t s2Pts = 0;
  float tau2Med = medianTauAcrossCaptures(SENSE2_PIN, cal2, tau2GuessUs, 500, 2000000, s2Sigma, s2R2, s2Pts);

  if (isnan(tau2Med)) {
    Serial.println("Stage 2 measurement failed.");
    return;
  }

  if (!tau2Filter.initialized) tau2Filter.init(tau2Med);
  else {
    tau2Filter.predict(dtSec);
    tau2Filter.update(tau2Med, s2Sigma);
  }

  lastTau2Us = tau2Filter.x;

  float rawR2 = (lastTau2Us * 1e-6f) / C2_F - lastRsourceOhm - R1_OHM;
  if (rawR2 < 0.0f) rawR2 = 0.0f;

  if (!r2Filter.initialized) r2Filter.init(rawR2);
  else {
    float sigmaR2 = fmaxf(s2Sigma * 1e-6f / C2_F, 0.5f);
    r2Filter.update(rawR2, sigmaR2);
  }
  lastR2Ohm = r2Filter.x;

  // -------- Print --------
  Serial.println();
  Serial.println("=== Dual-node RC estimate ===");
  Serial.printf("ADC S1 zero:             %.2f mV, noise: %.2f mV\n", cal1.zeroOffsetMv, cal1.noiseMv);
  Serial.printf("ADC S2 zero:             %.2f mV, noise: %.2f mV\n", cal2.zeroOffsetMv, cal2.noiseMv);

  if (!isnan(tau1Med)) {
    Serial.printf("Stage-1 tau median:      %.3f us\n", tau1Med);
    Serial.printf("Stage-1 fit R2:          %.4f\n", s1R2);
    Serial.printf("Stage-1 fit sigma:       %.3f us\n", s1Sigma);
    Serial.printf("Stage-1 points:          %u\n", s1Pts);
    Serial.printf("Stage-1 filtered tau:    %.3f us\n", lastTau1Us);
    Serial.printf("Rsource raw:             %.3f ohm\n", rSourceRaw);
    Serial.printf("Rsource filtered:        %.3f ohm\n", lastRsourceOhm);
    Serial.printf("Stage-1 trusted:         %s\n", stage1Trusted ? "yes" : "no");
  } else {
    Serial.println("Stage-1 unresolved; using previous source-resistance estimate.");
    Serial.printf("Rsource used:            %.3f ohm\n", lastRsourceOhm);
  }

  Serial.printf("Stage-2 tau median:      %.3f us\n", tau2Med);
  Serial.printf("Stage-2 fit R2:          %.4f\n", s2R2);
  Serial.printf("Stage-2 fit sigma:       %.3f us\n", s2Sigma);
  Serial.printf("Stage-2 points:          %u\n", s2Pts);
  Serial.printf("Stage-2 filtered tau:    %.3f us\n", lastTau2Us);

  Serial.printf("Unknown R2 raw:          %.3f ohm\n", rawR2);
  Serial.printf("Unknown R2 filtered:     %.3f ohm\n", lastR2Ohm);
  Serial.printf("Tau1 drift:              %.4f us/s\n", tau1Filter.x1);
  Serial.printf("Tau2 drift:              %.4f us/s\n", tau2Filter.x1);
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
  Serial.printf("Rsource prior:           %.3f ohm\n", R_SOURCE_PRIOR_OHM);
  Serial.printf("Known R1:                %.3f ohm\n", R1_OHM);
  Serial.printf("Known C1:                %.6e F\n", C1_F);
  Serial.printf("Known C2:                %.6e F\n", C2_F);

  cal1 = calibrateNodeZero(SENSE1_PIN, (R_SOURCE_PRIOR_OHM + R1_OHM) * C1_F * 1e6f);
  cal2 = calibrateNodeZero(SENSE2_PIN, 20000.0f);

  Serial.printf("Initial S1 zero:         %.2f mV, noise: %.2f mV\n", cal1.zeroOffsetMv, cal1.noiseMv);
  Serial.printf("Initial S2 zero:         %.2f mV, noise: %.2f mV\n", cal2.zeroOffsetMv, cal2.noiseMv);

  runResistanceCheck();
  lastRunMs = millis();
}

void loop() {
  if ((millis() - lastRunMs) >= UPDATE_PERIOD_MS) {
    runResistanceCheck();
  }
  delay(50);
}
