/*
 * Resistance measurement using 2nd-order RC ladder theory (Kirchhoff approach).
 * Pin (R0=250) -- R1(1) --+-- R2(Unknown) --+-- A0
 *                         |                 |
 *                        C1(0.5uF)         C2(10uF)
 *                         |                 |
 *                        GND               GND
 */

const int PIN_OUT = 9;
const int PIN_IN = A0;

// Known Constants
const float R0 = 250.0;
const float R1 = 1.0;
const float C1 = 0.5e-6;
const float C2 = 10.0e-6;
const float VCC = 5.0;

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  Serial.begin(9600);
}

// Measure Vpp at a given frequency
float measureVpp(float freq) {
  unsigned long period = 1000000.0 / freq;
  unsigned long halfPeriod = period / 2;
  float vMax = 0, vMin = 5.0;
  unsigned long start = millis();
  unsigned long window = 200; if (halfPeriod > 133333) window = (halfPeriod * 1.5) / 1000; while(millis() - start < window) {
    digitalWrite(PIN_OUT, HIGH);
    if (halfPeriod > 16000) delay(halfPeriod/1000); else delayMicroseconds(halfPeriod);
    float v = analogRead(PIN_IN) * (VCC / 1023.0);
    if (v > vMax) vMax = v;
    digitalWrite(PIN_OUT, LOW);
    if (halfPeriod > 16000) delay(halfPeriod/1000); else delayMicroseconds(halfPeriod);
    v = analogRead(PIN_IN) * (VCC / 1023.0);
    if (v < vMin) vMin = v;
  }
  return vMax - vMin;
}

void loop() {
  // Simple sweep to find "corner" frequency (Gain ~ 0.707)
  float bestFreq = 1.0;
  float minError = 5.0;
  
  for (float f = 1.0; f < 500.0; f *= 1.2) {
    float vpp = measureVpp(f);
    float err = abs(vpp - 3.535); // 5.0 * 0.707
    if (err < minError) {
      minError = err;
      bestFreq = f;
    }
  }

  // Calculate R2 using Kirchhoff/Transfer Function magnitude:
  // gain = vpp / Vcc
  // |H(jw)| = 1 / sqrt((1 - w^2 Ra R2 C1 C2)^2 + (w (R2 C2 + Ra C1 + Ra C2))^2)
  float vppFinal = measureVpp(bestFreq);
  float g = vppFinal / VCC;
  float w = 2.0 * PI * bestFreq;
  float Ra = R0 + R1;
  
  // R2 = (tau - Ra(C1+C2))/C2 where tau approx 1/(w*sqrt(1/g^2 - 1))
  float ratio = 1.0/g;
  if (ratio > 1.0) {
    float tau = sqrt(ratio*ratio - 1.0) / w;
    float r2 = (tau - Ra*(C1+C2)) / C2;
    Serial.print("Freq: "); Serial.print(bestFreq);
    Serial.print(" Hz, Measured R2: "); Serial.println(r2);
  }

  delay(60000);
}
