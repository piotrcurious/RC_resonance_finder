#include <Arduino.h>
#include <math.h>

// ============================================================
// ESP32 RC estimator
// ------------------------------------------------------------
// This sketch estimates an unknown resistor in a 2-stage RC
// network by driving a pulse on a digital pin and measuring the
// capacitor voltage on an ADC pin.
//
// Important physical note:
// An RC circuit does not have a true resonance frequency.
// The meaningful observable here is the step-response time
// constant tau, found from pulse-width timing.
//
// Model used:
//   Vc(t) = V0 + (VCC - V0) * (1 - exp(-t / tau))
//
// Therefore:
//   tau = -t / ln((VCC - Vc) / (VCC - V0))
//
// Then:
//   tau = R_total * C2
//   R_unknown = tau / C2 - R_source - R1
//
// The code uses binary search to find the pulse width that
// lands near 63.2% of the rise, then refines with repeated
// measurements and a Kalman filter.
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
constexpr float RC_TARGET_FRACTION = 0.6321205588f;

// Run once every 60 seconds
constexpr uint32_t UPDATE_PERIOD_MS = 60000UL;

// Search limits
constexpr float MIN_PULSE_US = 10.0f;
constexpr float MAX_PULSE_US = 2000000.0f;

// ----------------------------
// Persistent state
// ----------------------------
RTC_DATA_ATTR float lastTauUs = NAN;
RTC_DATA_ATTR float lastR2Ohm  = NAN;
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

static void sort3(float &a, float &b, float &c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
}

static void sort5(float v[5]) {
  for (int i = 0; i < 5; ++i) {
    for (int j = i + 1; j < 5; ++j) {
      if (v[j] < v[i]) {
        float t = v[i];
        v[i] = v[j];
        v[j] = t;
      }
    }
  }
}

static float median3(float a, float b, float c) {
  sort3(a, b, c);
  return b;
}

static float median5(float v[5]) {
  sort5(v);
  return v[2];
}

static float safeLogRatio(float num, float den) {
  if (num <= 0.0f || den <= 0.0f) return NAN;
  float r = num / den;
  if (r <= 0.0f || r >= 1.0f) return NAN;
  return logf(r);
}

// ----------------------------
// ADC helpers
// ----------------------------
static float readVoltageAvg(int pin, uint8_t samples = 16) {
  // Throw away the first read because ESP32 ADC mux can be noisy after switching.
  (void)analogReadMilliVolts(pin);

  long mvSum = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    mvSum += analogReadMilliVolts(pin);
    delayMicroseconds(60);
  }
  return (mvSum / (float)samples) / 1000.0f;
}

// ----------------------------
// Pulse measurement
// ----------------------------
static void samplePulse(uint32_t pulseUs, float &v0, float &v1) {
  // Discharge between samples so each measurement starts from a comparable state.
  digitalWrite(OUT_PIN, LOW);
  waitMicros(clampu32((uint32_t)(fmaxf(5.0f * (isnan(lastTauUs) ? 10000.0f : lastTauUs), 5000.0f)), 5000UL, 2000000UL));

  v0 = readVoltageAvg(SENSE_PIN, 12);

  digitalWrite(OUT_PIN, HIGH);
  waitMicros(clampu32(pulseUs, (uint32_t)MIN_PULSE_US, (uint32_t)MAX_PULSE_US));

  v1 = readVoltageAvg(SENSE_PIN, 12);

  digitalWrite(OUT_PIN, LOW);
}

static float tauFromPulseUs(float pulseUs, float v0, float v1) {
  // tau = -t / ln((VCC - Vc) / (VCC - V0))
  float den = VCC_VOLTS - v0;
  float num = VCC_VOLTS - v1;
  float lnArg = safeLogRatio(num, den);
  if (isnan(lnArg) || lnArg >= 0.0f) return NAN;
  return -pulseUs / lnArg;
}

static float targetVoltage(float v0) {
  return v0 + RC_TARGET_FRACTION * (VCC_VOLTS - v0);
}

// Estimate the pulse width needed to reach the 63.2% point.
// This is the direct physical time constant estimate.
static float binarySearchTauUs() {
  float seed = isnan(lastTauUs) ? 20000.0f : lastTauUs;

  float lo = clampf(seed * 0.25f, MIN_PULSE_US, MAX_PULSE_US);
  float hi = clampf(seed * 4.00f, lo * 1.10f, MAX_PULSE_US);

  float v0 = 0.0f, v1 = 0.0f;

  // Expand lower bound until it is below target.
  for (int i = 0; i < 8; ++i) {
    samplePulse((uint32_t)lo, v0, v1);
    if (v1 < targetVoltage(v0)) break;
    lo = fmaxf(MIN_PULSE_US, lo * 0.5f);
  }

  // Expand upper bound until it is above target.
  for (int i = 0; i < 8; ++i) {
    samplePulse((uint32_t)hi, v0, v1);
    if (v1 >= targetVoltage(v0)) break;
    hi = fminf(MAX_PULSE_US, hi * 2.0f);
  }

  // Binary search on pulse width.
  for (int i = 0; i < 16; ++i) {
    float mid = 0.5f * (lo + hi);
    samplePulse((uint32_t)mid, v0, v1);
    if (v1 < targetVoltage(v0)) lo = mid;
    else hi = mid;
  }

  return 0.5f * (lo + hi);
}

static float robustTauEstimateUs(float coarseTauUs, float &estimatedSigmaUs) {
  // Sample around the coarse estimate and convert each sample to tau.
  // This reduces sensitivity to ADC noise and pulse jitter.
  const float factors[5] = {0.85f, 0.95f, 1.00f, 1.05f, 1.15f};
  float taus[5];
  float valid[5];
  int n = 0;

  for (int i = 0; i < 5; ++i) {
    float v0 = 0.0f, v1 = 0.0f;
    uint32_t pulseUs = (uint32_t)clampf(coarseTauUs * factors[i], MIN_PULSE_US, MAX_PULSE_US);

    samplePulse(pulseUs, v0, v1);
    float tauUs = tauFromPulseUs((float)pulseUs, v0, v1);

    if (!isnan(tauUs) && tauUs > 0.0f) {
      valid[n++] = tauUs;
    }
  }

  if (n == 0) {
    estimatedSigmaUs = NAN;
    return NAN;
  }

  if (n == 1) {
    estimatedSigmaUs = 5000.0f;
    return valid[0];
  }

  // Median for robustness.
  float med = NAN;
  if (n == 2) med = 0.5f * (valid[0] + valid[1]);
  else if (n == 3) med = median3(valid[0], valid[1], valid[2]);
  else if (n == 4) {
    float tmp[5] = {valid[0], valid[1], valid[2], valid[3], 0.0f};
    sort5(tmp);
    med = 0.5f * (tmp[1] + tmp[2]);
  } else {
    float tmp[5] = {valid[0], valid[1], valid[2], valid[3], valid[4]};
    med = median5(tmp);
  }

  // Estimate spread using mean absolute deviation around median.
  float mad = 0.0f;
  for (int i = 0; i < n; ++i) mad += fabsf(valid[i] - med);
  mad /= (float)n;

  estimatedSigmaUs = fmaxf(200.0f, 1.4826f * mad);
  return med;
}

// ----------------------------
// Kalman filter
// State:
//   x0 = tau_us
//   x1 = d(tau_us)/dt
//
// Measurement:
//   z = tau_us
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

    // x = F x, with F = [1 dt; 0 1]
    x0 += x1 * dtSec;

    // P = F P F^T + Q
    float p00 = P00 + dtSec * (P10 + P01) + dtSec * dtSec * P11;
    float p01 = P01 + dtSec * P11;
    float p10 = P10 + dtSec * P11;
    float p11 = P11;

    // Process noise:
    // tau can drift slowly; drift rate can vary a bit more.
    float qTau  = 100.0f * dtSec;
    float qRate = 20.0f * dtSec;

    P00 = p00 + qTau;
    P01 = p01;
    P10 = p10;
    P11 = p11 + qRate;
  }

  void update(float zTauUs, float measSigmaUs) {
    if (!initialized) {
      init(zTauUs);
      return;
    }

    // Innovation
    float y = zTauUs - x0;

    // Measurement covariance from observed spread
    float Rm = measSigmaUs * measSigmaUs;
    Rm = fmaxf(Rm, 2500.0f);

    // Innovation covariance
    float S = P00 + Rm;
    if (S <= 0.0f) return;

    // Gating: reject strong outliers
    float gate = fabsf(y) / sqrtf(S);
    if (gate > 5.0f) {
      return;
    }

    // Kalman gain
    float K0 = P00 / S;
    float K1 = P10 / S;

    float oldP00 = P00;
    float oldP01 = P01;
    float oldP10 = P10;
    float oldP11 = P11;

    // State update
    x0 += K0 * y;
    x1 += K1 * y;

    // Covariance update
    P00 = (1.0f - K0) * oldP00;
    P01 = (1.0f - K0) * oldP01;
    P10 = oldP10 - K1 * oldP00;
    P11 = oldP11 - K1 * oldP01;

    // Re-symmetrize
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
// Main estimation routine
// ----------------------------
static void runResistanceCheck() {
  uint32_t nowMs = millis();
  float dtSec = (lastRunMs == 0) ? 0.0f : (nowMs - lastRunMs) / 1000.0f;
  lastRunMs = nowMs;

  // Coarse time constant from binary search
  float coarseTauUs = binarySearchTauUs();

  // Refined estimate around the coarse solution
  float sigmaUs = NAN;
  float refinedTauUs = robustTauEstimateUs(coarseTauUs, sigmaUs);

  if (isnan(refinedTauUs) || refinedTauUs <= 0.0f) {
    Serial.println("Measurement failed: invalid tau estimate.");
    return;
  }

  if (!kf.initialized) {
    kf.init(refinedTauUs);
  } else {
    kf.predict(dtSec);
    kf.update(refinedTauUs, sigmaUs);
  }

  float tauFiltUs = kf.x0;

  // Convert tau to resistance
  // tau = R_total * C2
  float rTotalOhm = (tauFiltUs * 1e-6f) / C2_F;
  float rUnknownOhm = rTotalOhm - R_SOURCE_OHM - R1_OHM;
  if (rUnknownOhm < 0.0f) rUnknownOhm = 0.0f;

  lastTauUs = tauFiltUs;
  lastR2Ohm = rUnknownOhm;

  Serial.println();
  Serial.println("=== RC estimate ===");
  Serial.printf("Stage-1 tau assumption: %.3f us\n", TAU1_US);
  Serial.printf("Coarse tau estimate:     %.3f us\n", coarseTauUs);
  Serial.printf("Refined tau estimate:    %.3f us\n", refinedTauUs);
  Serial.printf("Filtered tau estimate:   %.3f us\n", tauFiltUs);
  Serial.printf("Measured spread sigma:    %.3f us\n", sigmaUs);
  Serial.printf("Total series R estimate:  %.3f ohm\n", rTotalOhm);
  Serial.printf("Source resistance model:  %.3f ohm\n", R_SOURCE_OHM);
  Serial.printf("Known R1:                 %.3f ohm\n", R1_OHM);
  Serial.printf("Unknown R2 estimate:      %.3f ohm\n", rUnknownOhm);
  Serial.printf("Tau drift rate:           %.4f us/s\n", kf.x1);
  Serial.printf("Converged:                %s\n", kf.converged() ? "yes" : "no");
}

// ----------------------------
// Arduino entry points
// ----------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);

  pinMode(SENSE_PIN, INPUT);

  analogReadResolution(12);
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
