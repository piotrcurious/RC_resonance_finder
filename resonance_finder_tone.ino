/*
 * Frequency-Sweep measurement using 'tone()' for 2nd-order RC circuit.
 * Corrected to find the corner frequency where gain is 0.707.
 */

#define PIN_OUT 9
#define PIN_IN A0
#define C1 0.5e-6
#define C2 10e-6
#define RA (250.0 + 1.0)
#define V_REF 5.0

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  Serial.begin(9600);
}

void loop() {
  float maxVpp = 0;
  float cornerFreq = 1.0;
  float targetVpp = 3.535; // 5.0 * 0.707

  // Find corner frequency (Gain = 0.707)
  float bestFreq = 1.0;
  float minDiff = 5.0;

  for (float f = 1.0; f < 1000.0; f *= 1.1) {
    tone(PIN_OUT, (int)f);
    delay(100);
    float vpp = measureVpp(f);
    float diff = abs(vpp - targetVpp);
    if (diff < minDiff) {
      minDiff = diff;
      bestFreq = f;
    }
  }
  noTone(PIN_OUT);

  // w = 1 / tau_eff for corner freq
  float w = 2.0 * PI * bestFreq;
  float tau = 1.0 / w;
  float r2 = (tau - RA*(C1+C2)) / C2;

  Serial.print("Corner Freq: "); Serial.print(bestFreq);
  Serial.print(" Hz, Calc R2: "); Serial.println(r2);
  
  delay(60000);
}

float measureVpp(float f) {
  float vMax = 0, vMin = 5.0;
  unsigned long start = millis();
  while(millis() - start < 50) {
    float v = analogRead(PIN_IN) * (V_REF/1023.0);
    if (v > vMax) vMax = v;
    if (v < vMin) vMin = v;
  }
  return vMax - vMin;
}
