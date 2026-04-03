#include "ArduinoMock.h"
static uint64_t _mock_micros = 0;
static float _mock_v1 = 0, _mock_v2 = 0;
static int _mock_vin = LOW;
static float _mock_r2_true = 1000.0;
const float MOCK_R0 = 250.0, MOCK_R1 = 1.0, MOCK_C1 = 0.5e-6, MOCK_C2 = 10.0e-6;
MockSerial Serial;
void pinMode(int pin, int mode) {}
void digitalWrite(int pin, int state) { _mock_vin = state; }
int analogRead(int pin) { return (int)((_mock_v2 / 5.0) * 1023.0); }
unsigned long millis() { return (unsigned long)(_mock_micros / 1000); }
unsigned long micros() { return (unsigned long)_mock_micros; }
static void _tickPhysics(unsigned long dt_us) {
    // 0.1us steps for sub-microsecond precision
    int sub_steps = 10;
    float dt = (dt_us / 1e6) / sub_steps;
    float vin = (_mock_vin == HIGH) ? 5.0 : 0.0;
    float Ra = MOCK_R0 + MOCK_R1;

    for(int s=0; s<sub_steps; s++) {
        float ir1 = (vin - _mock_v1) / Ra;
        float ir2 = (_mock_v1 - _mock_v2) / _mock_r2_true;
        _mock_v1 += ((ir1 - ir2) / MOCK_C1) * dt;
        _mock_v2 += (ir2 / MOCK_C2) * dt;
    }
    _mock_micros += dt_us;
}
void delay(unsigned long ms) {
    if (ms >= 500) {
        unsigned long sim_us = 200000;
        uint64_t end_sim = _mock_micros + sim_us;
        while(_mock_micros < end_sim) _tickPhysics(100);
        _mock_micros += (uint64_t)(ms * 1000 - sim_us);
        return;
    }
    uint64_t end = _mock_micros + (uint64_t)ms * 1000;
    while(_mock_micros < end) _tickPhysics(100);
}
void delayMicroseconds(unsigned long us) {
    uint64_t end = _mock_micros + us;
    unsigned long step = (us > 200) ? 10 : 1;
    while(_mock_micros < end) {
        uint64_t rem = end - _mock_micros;
        _tickPhysics((unsigned long)(rem < step ? rem : step));
    }
}
void setPhysicsR2(float r2) { _mock_r2_true = r2; _mock_micros = 0; _mock_v1 = 0; _mock_v2 = 0; }
