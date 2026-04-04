/*
 * Refined Phase Shift measurement in RC_resonance_phase.ino.
 * Uses a lower threshold (1.0V) and sub-microsecond interpolation for better precision.
 */

#include "src/RCConfig.h"
#include "src/RCLadder.h"

RCLadder meter(PIN_OUT, PIN_IN, R1, C1, C2, R0);

void setup() {
  Serial.begin(9600);
}

void loop() {
  // 1.0V Threshold is 204.6 in ADC units.
  // We want to find the frequency where phase is exactly 90 deg.
  // For a 2nd order system, this means the threshold crossing happens at exactly T/4
  // but the gain might be low. We target crossing T/4.

  float fLow = 1.0, fHigh = 1000.0;
  float bestCrossing = 0;

  for(int i=0; i<12; i++) {
    float fMid = (fLow + fHigh) / 2.0;
    float period = 1000000.0 / fMid;
    float crossing = meter.findCrossingMicros(fMid, 204);

    if (crossing < period / 4.0) fLow = fMid;
    else fHigh = fMid;
    bestCrossing = crossing;
  }

  float targetF = (fLow + fHigh) / 2.0;
  // Re-calculate R2 using the 90deg condition: w^2 Ra R2 C1 C2 = 1
  float w = 2.0 * 3.14159265 * targetF;
  float r2 = 1.0 / (w * w * (R0+R1) * C1 * C2);

  Serial.print("Phase90_Freq_Hz:"); Serial.print(targetF);
  Serial.print(" R2_Est:"); Serial.println(r2);

  delay(60000);
}
