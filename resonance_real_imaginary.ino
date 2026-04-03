/*
 * Advanced Measurement in resonance_real_imaginary.ino.
 * Uses a correlation-based approach to extract magnitude and phase,
 * providing the complex impedance response of the 2nd-order RC ladder.
 */

#include <math.h>

const int PIN_OUT = 10;
const int PIN_IN = A0;
const float R0 = 250.0, R1 = 1.0, C1 = 0.5e-6, C2 = 10.0e-6, VCC = 5.0;

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  Serial.begin(9600);
}

void loop() {
  float freq = 20.0; // Test frequency

  float real_corr = 0, imag_corr = 0;
  int samples = 200;
  unsigned long period = 1000000.0 / freq;
  unsigned long dt = period / 20; // 20 samples per period

  for (int i = 0; i < samples; i++) {
    unsigned long start = micros();
    // Square wave oscillation
    if ((i % 20) < 10) digitalWrite(PIN_OUT, HIGH);
    else digitalWrite(PIN_OUT, LOW);

    float v = (analogRead(PIN_IN) * VCC) / 1023.0;
    float phase = (2.0 * PI * (i % 20)) / 20.0;

    real_corr += v * cos(phase);
    imag_corr += v * sin(phase);

    while(micros() - start < dt);
  }

  float mag = sqrt(real_corr*real_corr + imag_corr*imag_corr) * (2.0 / samples);
  float phase_deg = atan2(imag_corr, real_corr) * 180.0 / PI;

  // Solve for R2 from magnitude
  float gain = mag / (VCC * 0.5); // 0.5 factor for fundamental of square wave approx
  if (gain > 0.99) gain = 0.99;
  float w = 2.0 * PI * freq;
  float tau = sqrt(1.0/(gain*gain) - 1.0) / w;
  float r2 = (tau - (R0+R1)*(C1+C2)) / C2;

  Serial.print("Mag_V:"); Serial.print(mag);
  Serial.print(" Phase_Deg:"); Serial.print(phase_deg);
  Serial.print(" R2_Ohm:"); Serial.println(r2);

  delay(60000);
}
