#include "src/RCConfig.h"
#include "src/RCLadder.h"

RCLadder meter(PIN_OUT, PIN_IN, R1, C1, C2, R0);

void setup() {
  Serial.begin(9600);
}

void loop() {
  float f = 20.0;
  float vpp = meter.measureVpp(f);
  float r2 = meter.solveR2(f, vpp);
  Serial.print("R2_Est:"); Serial.println(r2);
  delay(60000);
}
