float measureVpp(float freq);
#include "RCConfig.h"
/*
 * Advanced PID Autotuning in CRC_kalman_pid_autotune.ino.
 * Uses a relay-feedback method to identify the ultimate gain (Ku)
 * and ultimate period (Tu) for the RC frequency control loop.
 */

#include <math.h>

const int PIN_OUT_VAL = 3;
const int PIN_IN_VAL = A0;

float targetVpp = 3.6;
float currentFreq = 22.0;

void setup() {
  pinMode(PIN_OUT_VAL, OUTPUT);
  pinMode(PIN_IN_VAL, INPUT);
  Serial.begin(9600);
  Serial.println("Starting PID Autotune (Relay Method)...");

  autoTune();
}

float measureVpp(float freq) {
  if (freq < 0.1) freq = 0.1;
  unsigned long half = 500000.0 / freq;
  float vSumMax = 0, vSumMin = 0;
  int count = 0;
  unsigned long start = millis();
  while(millis() - start < 200) {
    digitalWrite(PIN_OUT_VAL, HIGH);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    vSumMax += (analogRead(PIN_IN_VAL) * VCC) / 1023.0;
    digitalWrite(PIN_OUT_VAL, LOW);
    if (half > 16000) delay(half/1000); else delayMicroseconds(half);
    vSumMin += (analogRead(PIN_IN_VAL) * VCC) / 1023.0;
    count++;
  }
  return (vSumMax - vSumMin) / (count + 1e-6);
}

void autoTune() {
  float d_relay = 10.0; // Frequency step for relay
  float f_base = 22.0;
  float v_high = 0, v_low = 0;
  unsigned long t1, t2;

  Serial.println("Identifying Tu and Ku...");

  // High phase
  f_base = 15.0;
  v_high = measureVpp(f_base);
  t1 = millis();

  // Low phase
  f_base = 30.0;
  v_low = measureVpp(f_base);
  t2 = millis();

  float amplitude = abs(v_high - v_low) / 2.0;
  float Tu_ms = (t2 - t1) * 2.0;

  // Ku = 4d / (3.1415926535 * a)
  float Ku = (4.0 * d_relay) / (3.1415926535 * amplitude + 1e-6);
  float Tu = Tu_ms / 1000.0;

  float Kp = 0.6 * Ku;
  float Ki = 2.0 * Kp / (Tu + 1e-6);
  float Kd = Kp * Tu / 8.0;

  Serial.print("Tuned_Kp:"); Serial.print(Kp);
  Serial.print(" Tuned_Ki:"); Serial.print(Ki);
  Serial.print(" Tuned_Kd:"); Serial.println(Kd);
}

void loop() {
  float vpp = measureVpp(currentFreq);
  Serial.print("Freq_Hz:"); Serial.print(currentFreq);
  Serial.print(" Vpp_V:"); Serial.println(vpp);
  delay(5000);
}
