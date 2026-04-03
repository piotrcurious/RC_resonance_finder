#include <Arduino.h>
#include <math.h>
#include <esp_timer.h>

// ============================================================
// ESP32 dual-node RC estimator using ADC continuous mode
// ------------------------------------------------------------
// Key change:
//   - The code now samples both sense nodes with continuous ADC,
//     instead of polling analogRead / analogReadMilliVolts.
//
// Measurement model:
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
//
// Calibration:
//   Each capture begins with OUT low and a short baseline
//   acquisition window. The median of those baseline samples is
//   used as V0 for that capture.
// ============================================================

// ----------------------------
// Pins
// ----------------------------
// ADC1 pins on classic ESP32; continuous mode example in the docs
// uses ADC1 pins only.
constexpr int OUT_PIN    = 26;
constexpr int SENSE1_PIN = 34;
constexpr int SENSE2_PIN = 35;

// ----------------------------
// Known constants
// ----------------------------
constexpr float VCC_VOLTS = 3.3f;
constexpr float R_SOURCE_PRIOR_OHM = VCC_VOLTS / 0.020f; // 20 mA prior

constexpr float R1_OHM = 1.0f;
constexpr float C1_F   = 0.5e-6f;
constexpr float C2_F   = 10.0e-6f;

// Continuous ADC configuration.
// Tweak sample rate if your board/core combination prefers a different value.
constexpr uint32_t ADC_SAMPLE_FREQ_HZ = 25000;

// Main update interval
constexpr uint32_t UPDATE_PERIOD_MS = 60000UL;

// Capture knobs
constexpr uint8_t MAX_CAPTURE_POINTS = 80;
constexpr uint8_t BASELINE_SAMPLES    = 8;
constexpr uint8_t CAPTURE_REPEATS     = 2;

// Capture windows
constexpr uint32_t SHORT_WINDOW_MIN_US = 1000;
constexpr uint32_t SHORT_WINDOW_MAX_US = 30000;
constexpr uint32_t LONG_WINDOW_MIN_US  = 8000;
constexpr uint32_t LONG_WINDOW_MAX_US  = 800000;

// Fit quality thresholds
constexpr float MIN_R2_TRUST = 0.90f;
constexpr float MIN_R2_VALID = 0.80f;

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

static void sortArray(float *a, uint8_t n) {
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

static float medianOfArray(float *a, uint8_t n) {
  if (n == 0) return NAN;
  sortArray(a, n);
  if (n & 1) return a[n / 2];
  return 0.5f * (a[n / 2 - 1] + a[n / 2]);
}

static float madAroundMedian(const float *src, uint8_t n, float med) {
  float tmp[BASELINE_SAMPLES > MAX_CAPTURE_POINTS ? MAX_CAPTURE_POINTS : BASELINE_SAMPLES];
  uint8_t m = n;
  if (m > sizeof(tmp) / sizeof(tmp[0])) m = sizeof(tmp) / sizeof(tmp[0]);

  for (uint8_t i = 0; i < m; ++i) {
    tmp[i] = fabsf(src[i] - med);
  }
  return medianOfArray(tmp, m);
}

static uint32_t dischargeTimeFromSeed(float tauUs) {
  // Long enough to reset a node, short enough to avoid wasting time.
  float us = isnan(tauUs) ? 50000.0f : tauUs * 5.0f;
  return clampu32((uint32_t)us, 5000UL, 2000000UL);
}

// ----------------------------
// Continuous ADC support
// ----------------------------
static const uint8_t adcPins[] = { SENSE1_PIN, SENSE2_PIN };
constexpr uint8_t adcPinsCount = sizeof(adcPins) / sizeof(adcPins[0]);

static bool continuousReady = false;

void ARDUINO_ISR_ATTR onAdcDone() {
  continuousReady = true;
}

static bool initContinuous() {
  analogContinuousSetWidth(12);
  analogContinuousSetAtten(ADC_11db);

  // Configures ADC continuous on the selected pins.
  // The Arduino docs note the continuous example uses ADC1 pins.
  bool ok = analogContinuous(adcPins, adcPinsCount, 1, ADC_SAMPLE_FREQ_HZ, &onAdcDone);
  if (!ok) {
    Serial.println("analogContinuous() failed.");
    return false;
  }
  return true;
}

static bool readContinuousPair(float &mv1, float &mv2, uint32_t timeoutMs) {
  adc_continuous_result_t *result = nullptr;
  if (!analogContinuousRead(&result, timeoutMs) || result == nullptr) {
    return false;
  }

  bool got1 = false;
  bool got2 = false;

  for (uint8_t i = 0; i < adcPinsCount; ++i) {
    if (result[i].pin == SENSE1_PIN) {
      mv1 = (float)result[i].avg_read_mvolts;
      got1 = true;
    } else if (result[i].pin == SENSE2_PIN) {
      mv2 = (float)result[i].avg_read_mvolts;
      got2 = true;
    }
  }

  return got1 && got2;
}

// ----------------------------
// Capture structures
// ----------------------------
struct CurveCapture {
  float tUs[MAX_CAPTURE_POINTS];
  float v1Mv[MAX_CAPTURE_POINTS];
  float v2Mv[MAX_CAPTURE_POINTS];
  uint8_t n = 0;

  float baseline1Mv = NAN;
  float baseline2Mv = NAN;
  float noise1Mv = NAN;
  float noise2Mv = NAN;
};

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

struct NodeSummary {
  bool valid = false;
  float tauUs = NAN;
  float sigmaUs = NAN;
  float r2 = NAN;
  uint8_t usedPoints = 0;
  float vSpanMv = NAN;
};

// ----------------------------
// Exponential fit
// ----------------------------
static FitResult fitExponentialMv(const float *tUs, const float *vMv, uint8_t n, float baselineMv) {
  FitResult out;
  if (n < 3) return out;

  const float v0 = baselineMv / 1000.0f;
  const float denV = VCC_VOLTS - v0;
  if (denV <= 0.02f) return out;

  float x[16];
  float y[16];
  uint8_t good = 0;

  for (uint8_t i = 0; i < n && good < 16; ++i) {
    float vc = vMv[i] / 1000.0f;
    float ly = safeLogRatio(VCC_VOLTS - vc, denV);
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

  // One-pass outlier rejection.
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

static NodeSummary summarizeFit(const FitResult &fit) {
  NodeSummary s;
  if (!fit.valid) return s;
  if (fit.r2 < MIN_R2_VALID) return s;

  s.valid = true;
  s.tauUs = fit.tauUs;
  s.sigmaUs = fit.sigmaUs;
  s.r2 = fit.r2;
  s.usedPoints = fit.usedPoints;
  s.vSpanMv = fit.vSpanMv;
  return s;
}

// ----------------------------
// Continuous capture
// ----------------------------
static bool readBaselinePair(float &b1, float &b2, float &noise1, float &noise2) {
  float s1[BASELINE_SAMPLES];
  float s2[BASELINE_SAMPLES];
  uint8_t n = 0;

  while (n < BASELINE_SAMPLES) {
    float mv1 = NAN, mv2 = NAN;
    if (!readContinuousPair(mv1, mv2, 5)) {
      continue;
    }
    s1[n] = mv1;
    s2[n] = mv2;
    n++;
  }

  float c1[BASELINE_SAMPLES];
  float c2[BASELINE_SAMPLES];
  for (uint8_t i = 0; i < BASELINE_SAMPLES; ++i) {
    c1[i] = s1[i];
    c2[i] = s2[i];
  }

  b1 = medianOfArray(c1, BASELINE_SAMPLES);
  b2 = medianOfArray(c2, BASELINE_SAMPLES);

  float mad1 = madAroundMedian(s1, BASELINE_SAMPLES, b1);
  float mad2 = madAroundMedian(s2, BASELINE_SAMPLES, b2);

  noise1 = fmaxf(2.0f, 1.4826f * mad1);
  noise2 = fmaxf(2.0f, 1.4826f * mad2);

  return true;
}

static bool captureCurve(uint32_t windowUs, CurveCapture &cap, float seedTauUsForDischarge) {
  cap.n = 0;
  cap.baseline1Mv = NAN;
  cap.baseline2Mv = NAN;
  cap.noise1Mv = NAN;
  cap.noise2Mv = NAN;

  // Make sure the ADC continuous engine is running before we capture.
  continuousReady = false;
  if (!analogContinuousStart()) {
    Serial.println("analogContinuousStart() failed.");
    return false;
  }

  // Discharge before baseline sampling.
  digitalWrite(OUT_PIN, LOW);
  waitMicros(dischargeTimeFromSeed(seedTauUsForDischarge));

  // Calibrate baseline with OUT low.
  readBaselinePair(cap.baseline1Mv, cap.baseline2Mv, cap.noise1Mv, cap.noise2Mv);

  // Start the actual step.
  digitalWrite(OUT_PIN, HIGH);
  int64_t t0 = nowUs();

  while ((uint32_t)(nowUs() - t0) < windowUs && cap.n < MAX_CAPTURE_POINTS) {
    float mv1 = NAN, mv2 = NAN;
    if (!readContinuousPair(mv1, mv2, 2)) {
      continue;
    }

    cap.tUs[cap.n] = (float)((uint32_t)(nowUs() - t0));
    cap.v1Mv[cap.n] = mv1;
    cap.v2Mv[cap.n] = mv2;
    cap.n++;
  }

  digitalWrite(OUT_PIN, LOW);
  analogContinuousStop();

  return cap.n >= 3;
}

static FitResult fitNodeFromCapture(const CurveCapture &cap, bool node1) {
  const float *v = node1 ? cap.v1Mv : cap.v2Mv;
  float baselineMv = node1 ? cap.baseline1Mv : cap.baseline2Mv;

  float fitV[MAX_CAPTURE_POINTS];
  float vMin = v[0];
  float vMax = v[0];
  for (uint8_t i = 0; i < cap.n; ++i) {
    fitV[i] = v[i];
    if (v[i] < vMin) vMin = v[i];
    if (v[i] > vMax) vMax = v[i];
  }

  FitResult fit = fitExponentialMv(cap.tUs, fitV, cap.n, baselineMv);
  if (fit.valid) {
    fit.vSpanMv = vMax - vMin;
    if (fit.r2 < MIN_R2_VALID || fit.vSpanMv < 20.0f) {
      fit.valid = false;
    }
  }

  return fit;
}

struct PassPair {
  NodeSummary s1;
  NodeSummary s2;
  bool valid = false;
};

static PassPair runPass(uint32_t windowUs, float seedTauUsForDischarge) {
  PassPair out;
  CurveCapture cap;

  if (!captureCurve(windowUs, cap, seedTauUsForDischarge)) {
    return out;
  }

  FitResult f1 = fitNodeFromCapture(cap, true);
  FitResult f2 = fitNodeFromCapture(cap, false);

  out.s1 = summarizeFit(f1);
  out.s2 = summarizeFit(f2);
  out.valid = true;
  return out;
}

static bool medianSummaryFromPasses(const NodeSummary *arr, uint8_t n, NodeSummary &out) {
  float taus[8];
  float sigmas[8];
  float r2s[8];
  uint8_t pts[8];
  uint8_t m = 0;

  for (uint8_t i = 0; i < n && m < 8; ++i) {
    if (!arr[i].valid) continue;
    taus[m] = arr[i].tauUs;
    sigmas[m] = arr[i].sigmaUs;
    r2s[m] = arr[i].r2;
    pts[m] = arr[i].usedPoints;
    m++;
  }

  if (m == 0) return false;

  float tauMed = medianOfArray(taus, m);

  uint8_t bestIdx = 0;
  for (uint8_t i = 1; i < m; ++i) {
    if (r2s[i] > r2s[bestIdx]) bestIdx = i;
  }

  out.valid = true;
  out.tauUs = tauMed;
  out.sigmaUs = sigmas[bestIdx];
  out.r2 = r2s[bestIdx];
  out.usedPoints = pts[bestIdx];
  return true;
}

// ----------------------------
// Estimation cycle
// ----------------------------
static void runResistanceCheck() {
  uint32_t nowMs = millis();
  float dtSec = (lastRunMs == 0) ? 0.0f : (nowMs - lastRunMs) / 1000.0f;
  lastRunMs = nowMs;

  // Seed estimates from previous run or priors.
  float tau1SeedUs = isnan(lastTau1Us)
    ? (R_SOURCE_PRIOR_OHM + R1_OHM) * C1_F * 1e6f
    : lastTau1Us;

  float tau2SeedUs = isnan(lastTau2Us)
    ? 20000.0f
    : lastTau2Us;

  uint32_t shortWindowUs = clampu32((uint32_t)(tau1SeedUs * 10.0f), SHORT_WINDOW_MIN_US,
