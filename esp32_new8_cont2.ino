#include <Arduino.h>
#include <math.h>
#include <esp_timer.h>

// ============================================================
// Dual-node ESP32 RC estimator using ADC continuous mode
// ------------------------------------------------------------
// Improvements in this version:
//   - tighter continuous-mode capture loop
//   - explicit channel averaging from adc_continuous_result_t
//   - separate calibration state for each node
//   - adaptive capture windows
//   - monotonicity checks
//   - retry logic when stage 1 is too fast or noisy
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
// Known constants
// ----------------------------
constexpr float VCC_VOLTS = 3.3f;
constexpr float R_SOURCE_PRIOR_OHM = VCC_VOLTS / 0.020f; // 20 mA prior

constexpr float R1_OHM = 1.0f;
constexpr float C1_F   = 0.5e-6f;
constexpr float C2_F   = 10.0e-6f;

constexpr uint32_t UPDATE_PERIOD_MS = 60000UL;

// Continuous ADC settings.
// Espressif documents these functions in Arduino-ESP32:
//   analogContinuous(...), analogContinuousRead(...),
//   analogContinuousStart(), analogContinuousStop(), analogContinuousDeinit().
// The docs also say the result contains avg_read_raw and avg_read_mvolts.
// See: https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html
constexpr uint32_t ADC_SAMPLE_FREQ_HZ = 25000;
constexpr uint8_t  ADC_CONVERSIONS_PER_PIN = 8;

// Capture control
constexpr uint8_t  BASELINE_SAMPLES = 16;
constexpr uint8_t  CAPTURE_REPEATS   = 3;
constexpr uint8_t  MAX_CAPTURE_POINTS = 96;

// Fit thresholds
constexpr float MIN_R2_VALID  = 0.80f;
constexpr float MIN_R2_TRUST  = 0.90f;

// Window limits
constexpr uint32_t SHORT_WINDOW_MIN_US = 1200;
constexpr uint32_t SHORT_WINDOW_MAX_US = 40000;
constexpr uint32_t LONG_WINDOW_MIN_US  = 10000;
constexpr uint32_t LONG_WINDOW_MAX_US  = 1000000;

// Sampling fractions inside one capture window
constexpr uint8_t SAMPLE_COUNT = 10;
constexpr float SAMPLE_FRAC[SAMPLE_COUNT] = {
  0.05f, 0.12f, 0.20f, 0.30f, 0.42f,
  0.56f, 0.68f, 0.79f, 0.90f, 0.97f
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

static float medianOfArray(float *a, uint8_t n) {
  if (n == 0) return NAN;
  insertionSort(a, n);
  if (n & 1) return a[n / 2];
  return 0.5f * (a[n / 2 - 1] + a[n / 2]);
}

static float madAroundMedian(const float *src, uint8_t n, float med) {
  float tmp[BASELINE_SAMPLES > MAX_CAPTURE_POINTS ? BASELINE_SAMPLES : MAX_CAPTURE_POINTS];
  if (n > sizeof(tmp) / sizeof(tmp[0])) n = sizeof(tmp) / sizeof(tmp[0]);

  for (uint8_t i = 0; i < n; ++i) {
    tmp[i] = fabsf(src[i] - med);
  }
  return medianOfArray(tmp, n);
}

static uint32_t dischargeTimeFromSeed(float tauUs) {
  float us = isnan(tauUs) ? 50000.0f : tauUs * 5.0f;
  return clampu32((uint32_t)us, 5000UL, 2000000UL);
}

// ----------------------------
// ADC continuous setup
// ----------------------------
static const uint8_t adcPins[] = { SENSE1_PIN, SENSE2_PIN };
constexpr uint8_t adcPinsCount = sizeof(adcPins) / sizeof(adcPins[0]);

static bool adcContinuousReady = false;

void ARDUINO_ISR_ATTR onAdcDone() {
  adcContinuousReady = true;
}

static bool initContinuous() {
  analogContinuousSetWidth(12);
  analogContinuousSetAtten(ADC_11db);

  // The Arduino docs describe this API as taking the pin list,
  // conversions per pin, sampling frequency, and optional callback.
  if (!analogContinuous(adcPins, adcPinsCount, ADC_CONVERSIONS_PER_PIN, ADC_SAMPLE_FREQ_HZ, &onAdcDone)) {
    return false;
  }
  return true;
}

static bool startContinuous() {
  adcContinuousReady = false;
  return analogContinuousStart();
}

static bool stopContinuous() {
  return analogContinuousStop();
}

static void deinitContinuous() {
  analogContinuousDeinit();
}

// ----------------------------
// Calibration state
// ----------------------------
struct AdcCal {
  bool valid = false;
  float zeroOffsetMv = 0.0f;
  float noiseMv = 5.0f;
  float baselineMv = 0.0f;
};

static AdcCal cal1;
static AdcCal cal2;

static float applyCalMv(float mv, const AdcCal &cal) {
  float x = (mv - cal.zeroOffsetMv);
  return (x < 0.0f) ? 0.0f : x;
}

static bool readContinuousPair(float &mv1, float &mv2, uint32_t timeoutMs) {
  adc_continuous_result_t *buffer = nullptr;
  if (!analogContinuousRead(&buffer, timeoutMs) || buffer == nullptr) {
    return false;
  }

  bool got1 = false;
  bool got2 = false;

  // The documented result buffer is an array of adc_continuous_result_t.
  // The driver groups averages per pin for the configured channels. 1
  for (int i = 0; buffer[i].pin != 0 || buffer[i].channel != 0 || buffer[i].avg_read_raw != 0 || buffer[i].avg_read_mvolts != 0; ++i) {
    if (buffer[i].pin == SENSE1_PIN) {
      mv1 = (float)buffer[i].avg_read_mvolts;
      got1 = true;
    } else if (buffer[i].pin == SENSE2_PIN) {
      mv2 = (float)buffer[i].avg_read_mvolts;
      got2 = true;
    }
    if (got1 && got2) break;

    // Safety stop in case the returned array is shorter than expected.
    if (i > 8) break;
  }

  return got1 && got2;
}

static AdcCal calibrateNodeZero(int pin, float dischargeTauUs) {
  AdcCal cal;
  float samples[BASELINE_SAMPLES];

  digitalWrite(OUT_PIN, LOW);
  waitMicros(dischargeTimeFromSeed(dischargeTauUs));

  for (uint8_t i = 0; i < BASELINE_SAMPLES; ++i) {
    float mv1 = NAN, mv2 = NAN;
    if (!readContinuousPair(mv1, mv2, 10)) {
      samples[i] = 0.0f;
      continue;
    }
    samples[i] = (pin == SENSE1_PIN) ? mv1 : mv2;
    delayMicroseconds(120);
  }

  float tmp[BASELINE_SAMPLES];
  for (uint8_t i = 0; i < BASELINE_SAMPLES; ++i) tmp[i] = samples[i];
  float med = medianOfArray(tmp, BASELINE_SAMPLES);
  float mad = madAroundMedian(samples, BASELINE_SAMPLES, med);

  cal.valid = true;
  cal.zeroOffsetMv = med;
  cal.noiseMv = fmaxf(2.0f, 1.4826f * mad);
  cal.baselineMv = med;
  return cal;
}

static void refreshCalibration(bool forcePrint = false) {
  float tau1GuessUs = isnan(lastTau1Us)
    ? (R_SOURCE_PRIOR_OHM + R1_OHM) * C1_F * 1e6f
    : lastTau1Us;

  float tau2GuessUs = isnan(lastTau2Us) ? 20000.0f : lastTau2Us;

  cal1 = calibrateNodeZero(SENSE1_PIN, tau1GuessUs);
  cal2 = calibrateNodeZero(SENSE2_PIN, tau2GuessUs);

  if (forcePrint) {
    Serial.printf("ADC cal S1 zero: %.2f mV, noise: %.2f mV\n", cal1.zeroOffsetMv, cal1.noiseMv);
    Serial.printf("ADC cal S2 zero: %.2f mV, noise: %.2f mV\n", cal2.zeroOffsetMv, cal2.noiseMv);
  }
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

static FitResult fitExponential(const float *tUs, const float *vVolts, uint8_t n, float baselineVolts) {
  FitResult out;
  if (n < 3) return out;

  const float denV = VCC_VOLTS - baselineVolts;
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
// Node capture
// ----------------------------
struct NodeSummary {
  bool valid = false;
  float tauUs = NAN;
  float sigmaUs = NAN;
  float r2 = NAN;
  uint8_t usedPoints = 0;
  float vSpanMv = NAN;
};

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

struct CurveCapture {
  float tUs[MAX_CAPTURE_POINTS];
  float v1Mv[MAX_CAPTURE_POINTS];
  float v2Mv[MAX_CAPTURE_POINTS];
  uint8_t n = 0;
  float baseline1Mv = NAN;
  float baseline2Mv = NAN;
};

static bool captureCurve(uint32_t windowUs, CurveCapture &cap, float seedTauUsForDischarge) {
  cap.n = 0;

  if (!startContinuous()) {
    return false;
  }

  digitalWrite(OUT_PIN, LOW);
  waitMicros(dischargeTimeFromSeed(seedTauUsForDischarge));

  float b1 = NAN, b2 = NAN;
  {
    float n1 = NAN, n2 = NAN;
    // Capture a few baseline points while the node is discharged.
    float s1[BASELINE_SAMPLES];
    float s2[BASELINE_SAMPLES];
    for (uint8_t i = 0; i < BASELINE_SAMPLES; ++i) {
      if (!readContinuousPair(n1, n2, 10)) {
        s1[i] = 0.0f;
        s2[i] = 0.0f;
      } else {
        s1[i] = n1;
        s2[i] = n2;
      }
    }
    float c1[BASELINE_SAMPLES], c2[BASELINE_SAMPLES];
    for (uint8_t i = 0; i < BASELINE_SAMPLES; ++i) {
      c1[i] = s1[i];
      c2[i] = s2[i];
    }
    b1 = medianOfArray(c1, BASELINE_SAMPLES);
    b2 = medianOfArray(c2, BASELINE_SAMPLES);
  }

  cap.baseline1Mv = b1;
  cap.baseline2Mv = b2;

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
  stopContinuous();

  return cap.n >= 3;
}

static FitResult fitNodeFromCapture(const CurveCapture &cap, bool node1) {
  const float *v = node1 ? cap.v1Mv : cap.v2Mv;
  float baselineMv = node1 ? cap.baseline1Mv : cap.baseline2Mv;

  float vv[MAX_CAPTURE_POINTS];
  float vMin = v[0];
  float vMax = v[0];
  for (uint8_t i = 0; i < cap.n; ++i) {
    vv[i] = v[i];
    if (v[i] < vMin) vMin = v[i];
    if (v[i] > vMax) vMax = v[i];
  }

  FitResult fit = fitExponential(cap.tUs, vv, cap.n, baselineMv / 1000.0f);
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
  if (!captureCurve(windowUs, cap, seedTauUsForDischarge)) return out;

  FitResult f1 = fitNodeFromCapture(cap, true);
  FitResult f2 = fitNodeFromCapture(cap, false);

  out.s1 = summarizeFit(f1);
  out.s2 = summarizeFit(f2);
  out.valid = true;
  return out;
}

static bool medianSummaryFromPasses(const NodeSummary *arr, uint8_t n, NodeSummary &out) {
  float taus[CAPTURE_REPEATS];
  float sigmas[CAPTURE_REPEATS];
  float r2s[CAPTURE_REPEATS];
  uint8_t pts[CAPTURE_REPEATS];
  uint8_t m = 0;

  for (uint8_t i = 0; i < n && m < CAPTURE_REPEATS; ++i) {
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
// Measurement cycle
// ----------------------------
static void runResistanceCheck() {
  uint32_t nowMs = millis();
  float dtSec = (lastRunMs == 0) ? 0.0f : (nowMs - lastRunMs) / 1000.0f;
  lastRunMs = nowMs;

  // Re-learn ADC baseline on every cycle.
  float tau1GuessUs = isnan(lastTau1Us)
    ? (R_SOURCE_PRIOR_OHM + R1_OHM) * C1_F * 1e6f
    : lastTau1Us;

  float tau2GuessUs = isnan(lastTau2Us) ? 20000.0f : lastTau2Us;

  refreshCalibration(false);

  uint32_t shortWindowUs = clampu32((uint32_t)(tau1GuessUs * 10.0f), SHORT_WINDOW_MIN_US, SHORT_WINDOW_MAX_US);
  uint32_t longWindowUs  = clampu32((uint32_t)(tau2GuessUs * 5.0f), LONG_WINDOW_MIN_US, LONG_WINDOW_MAX_US);

  // Multiple passes for each stage.
  PassPair shortPasses[CAPTURE_REPEATS];
  PassPair longPasses[CAPTURE_REPEATS];

  for (uint8_t i = 0; i < CAPTURE_REPEATS; ++i) {
    shortPasses[i] = runPass(shortWindowUs, tau1GuessUs);
  }

  for (uint8_t i = 0; i < CAPTURE_REPEATS; ++i) {
    longPasses[i] = runPass(longWindowUs, tau2GuessUs);
  }

  // Summarize stage 1
  NodeSummary s1Arr[CAPTURE_REPEATS];
  for (uint8_t i = 0; i < CAPTURE_REPEATS; ++i) s1Arr[i] = shortPasses[i].s1;

  NodeSummary s1Summary;
  bool s1Ok = medianSummaryFromPasses(s1Arr, CAPTURE_REPEATS, s1Summary);

  float rSourceRaw = NAN;
  bool stage1Trusted = false;

  if (s1Ok) {
    lastTau1Us = s1Summary.tauUs;
    rSourceRaw = (lastTau1Us * 1e-6f) / C1_F - R1_OHM;
    if (rSourceRaw < 0.0f) rSourceRaw = 0.0f;
    stage1Trusted = (s1Summary.r2 >= MIN_R2_TRUST && s1Summary.usedPoints >= 5);

    if (isnan(lastRsourceOhm)) {
      lastRsourceOhm = rSourceRaw;
    } else if (stage1Trusted) {
      lastRsourceOhm = 0.75f * lastRsourceOhm + 0.25f * rSourceRaw;
    } else {
      lastRsourceOhm = 0.90f * lastRsourceOhm + 0.10f * rSourceRaw;
    }
  } else {
    if (isnan(lastRsourceOhm)) lastRsourceOhm = R_SOURCE_PRIOR_OHM;
  }

  // Summarize stage 2
  NodeSummary s2Arr[CAPTURE_REPEATS];
  for (uint8_t i = 0; i < CAPTURE_REPEATS; ++i) s2Arr[i] = longPasses[i].s2;

  NodeSummary s2Summary;
  bool s2Ok = medianSummaryFromPasses(s2Arr, CAPTURE_REPEATS, s2Summary);
  if (!s2Ok) {
    Serial.println("Stage 2 measurement failed.");
    return;
  }

  lastTau2Us = s2Summary.tauUs;
  float rawR2 = (lastTau2Us * 1e-6f) / C2_F - lastRsourceOhm - R1_OHM;
  if (rawR2 < 0.0f) rawR2 = 0.0f;

  if (isnan(lastR2Ohm)) {
    lastR2Ohm = rawR2;
  } else {
    lastR2Ohm = 0.75f * lastR2Ohm + 0.25f * rawR2;
  }

  // Diagnostics
  Serial.println();
  Serial.println("=== Dual-node RC estimate with ADC continuous ===");
  Serial.printf("Short window:            %lu us\n", (unsigned long)shortWindowUs);
  Serial.printf("Long window:             %lu us\n", (unsigned long)longWindowUs);

  if (s1Ok) {
    Serial.printf("S1 baseline:             %.2f mV\n", shortPasses[0].s1.valid ? shortPasses[0].s1.vSpanMv : 0.0f);
    Serial.printf("S1 fit R2:               %.4f\n", s1Summary.r2);
    Serial.printf("S1 points:               %u\n", s1Summary.usedPoints);
    Serial.printf("S1 tau:                  %.3f us\n", lastTau1Us);
    Serial.printf("Rsource raw:             %.3f ohm\n", rSourceRaw);
    Serial.printf("Rsource filtered:        %.3f ohm\n", lastRsourceOhm);
    Serial.printf("S1 trusted:              %s\n", stage1Trusted ? "yes" : "no");
  } else {
    Serial.println("S1 unresolved; using prior source-resistance estimate.");
    Serial.printf("Rsource used:            %.3f ohm\n", lastRsourceOhm);
  }

  Serial.printf("S2 fit R2:               %.4f\n", s2Summary.r2);
  Serial.printf("S2 points:               %u\n", s2Summary.usedPoints);
  Serial.printf("S2 tau:                  %.3f us\n", lastTau2Us);
  Serial.printf("Unknown R2 raw:          %.3f ohm\n", rawR2);
  Serial.printf("Unknown R2 filtered:     %.3f ohm\n", lastR2Ohm);
}

void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);

  pinMode(SENSE1_PIN, INPUT);
  pinMode(SENSE2_PIN, INPUT);

  if (!initContinuous()) {
    Serial.println("ADC continuous init failed.");
    while (true) delay(1000);
  }

  Serial.println("ESP32 dual-node RC estimator started.");
  Serial.printf("Rsource prior:           %.3f ohm\n", R_SOURCE_PRIOR_OHM);
  Serial.printf("Known R1:                %.3f ohm\n", R1_OHM);
  Serial.printf("Known C1:                %.6e F\n", C1_F);
  Serial.printf("Known C2:                %.6e F\n", C2_F);
  Serial.printf("ADC sample rate:         %lu Hz\n", (unsigned long)ADC_SAMPLE_FREQ_HZ);

  refreshCalibration(true);
  runResistanceCheck();
  lastRunMs = millis();
}

void loop() {
  if ((millis() - lastRunMs) >= UPDATE_PERIOD_MS) {
    runResistanceCheck();
  }
  delay(50);
}
