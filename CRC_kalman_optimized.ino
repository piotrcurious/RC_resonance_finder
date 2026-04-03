float measureVpp(float freq);
#include "RCConfig.h"
/*
 * Refactored Optimized RC Measurement using RCLadder library.
 */

#include "RCLadder.h"

RCLadder meter(3, A0, 1.0, 0.5e-6, 10.0e-6);

unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (millis() - lastTime >= 60000 || lastTime == 0) {
    lastTime = millis();

    // Find target frequency
    float fL = 0.1, fH = 1000.0;
    for (int i=0; i<10; i++) {
      float fM = (fL + fH) / 2.0;
      if (meter.measureVpp(fM) > 3.6) fL = fM; else fH = fM;
    }
    float f = (fL + fH) / 2.0;
    float vpp = meter.measureVpp(f);
    float rawR = meter.solveR2(f, vpp);

    meter.updateKalman(rawR);

    Serial.print("R2_Raw:"); Serial.print(rawR);
    Serial.print(" R2_Est:"); Serial.print(meter.getEstimate());
    Serial.print(" Confidence:"); Serial.println(meter.getConfidence());
  }
}
