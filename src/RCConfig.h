#ifndef RCCONFIG_H
#define RCCONFIG_H

// Physical Constants for 2nd-Order RC Ladder
const float R0 = 250.0;    // Arduino digital pin internal resistance (Approx 20mA limit)
const float R1 = 1.0;      // Series resistor stage 1
const float C1 = 0.5e-6;   // Shunt capacitor stage 1
const float C2 = 10.0e-6;  // Shunt capacitor stage 2 (Measurement node)
const float VCC = 5.0;     // Logic supply voltage

// Pin Mapping
const int PIN_OUT = 3;     // PWM/Digital output for excitation
const int PIN_IN = A0;     // Analog measurement node (Voltage across C2)

#ifndef PI
#define PI 3.14159265358979323846
#endif

#endif
