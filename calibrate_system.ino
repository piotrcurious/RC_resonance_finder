float measureVpp(float freq);
#include "RCConfig.h"
/*
 * calibrate_system.ino - Helps calibrate Arduino pin resistance R0.
 * Instructions: Connect a known reference resistor (e.g., 1k) as R2.
 */

#include "RCLadder.h"

// Define with a default R0 = 0 to see the total path resistance
RCLadder meter(3, A0, 1.0, 0.5e-6, 10.0e-6, 0.0);
float referenceR2 = 1000.0;

void setup() {
  Serial.begin(9600);
  Serial.println("System Calibration Starting...");
  Serial.println("Please connect a known 1000 Ohm resistor as R2.");
  delay(2000);
}

void loop() {
  float f = 20.0;
  float vpp = meter.measureVpp(f);
  float ratio = vpp / 5.0;

  if (ratio < 0.99) {
    float artanh_val = 0.5 * log((1.0 + ratio) / (1.0 - ratio));
    float tau = 1.0 / (4.0 * f * artanh_val);

    // Solve for total series resistance (Ra + R2)
    // tau = (Ra + R2)*C2 + Ra*C1
    // tau = Ra*(C1 + C2) + R2*C2
    // Ra = (tau - R2*C2) / (C1 + C2)
    float C1 = 0.5e-6, C2 = 10.0e-6;
    float Ra_calc = (tau - referenceR2 * C2) / (C1 + C2);

    // R0 = Ra - R1
    float R0_calc = Ra_calc - 1.0;

    Serial.print("Vpp:"); Serial.print(vpp);
    Serial.print(" Calculated_R0:"); Serial.println(R0_calc);
  }

  delay(5000);
}
