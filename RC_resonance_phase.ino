/*
 * Precision Phase Shift measurement using RCLadder library.
 * Leverages sub-microsecond interpolation to find the 90-degree phase point.
 */

#include "RCLadder.h"
#include "RCConfig.h"

RCLadder meter(PIN_OUT, PIN_IN, R1, C1, C2, R0);

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Find frequency where phase is 90 deg (crossing threshold 512 at exactly T/4)
  float fLow = 1.0, fHigh = 1000.0;
  float targetT = 0;

  for(int i=0; i<12; i++) {
    float fMid = (fLow + fHigh) / 2.0;
    float period = 1000000.0 / fMid;
    float crossing = meter.findCrossingMicros(fMid, 204);

    // In 2nd order, 90 deg phase means crossing happens at T/4
    if (crossing < period / 4.0) fLow = fMid;
    else fHigh = fMid;
    targetT = crossing;
  }

  float targetF = (fLow + fHigh) / 2.0;
  float w = 2.0 * 3.14159265 * targetF;
  float r2 = 1.0 / (w * w * (R0+R1) * C1 * C2);

  Serial.print("Phase_90_Freq:"); Serial.print(targetF);
  Serial.print(" Crossing_us:"); Serial.print(targetT);
  Serial.print(" R2_Est:"); Serial.println(r2);

  delay(60000);
}
