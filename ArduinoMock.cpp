#include "ArduinoMock.h"
#include <math.h>

static uint64_t _mock_micros = 0;
static float _mock_v1 = 0, _mock_v2 = 0;
static int _mock_vin = LOW;
static float _mock_r2_true = 1000.0;

const float MOCK_R0 = 250.0, MOCK_R1 = 1.0, MOCK_C1 = 0.5e-6, MOCK_C2 = 10.0e-6;

MockSerial Serial;

void pinMode(int pin, int mode) {}
void digitalWrite(int pin, int state) { _mock_vin = state; }

int analogRead(int pin) {
    int raw = (int)((_mock_v2 / 5.0) * 1023.0);
    if (raw < 0) raw = 0; if (raw > 1023) raw = 1023;
    return raw;
}

unsigned long millis() { return (unsigned long)(_mock_micros / 1000); }
unsigned long micros() { return (unsigned long)_mock_micros; }

static void _tickPhysics(unsigned long dt_us) {
    float dt = dt_us / 1e6;
    float vin = (_mock_vin == HIGH) ? 5.0 : 0.0;
    float Ra = MOCK_R0 + MOCK_R1;
    // Semi-implicit Euler for better stability at large dt
    float v1_next = (_mock_v1 + (vin/Ra + _mock_v2/_mock_r2_true)*(dt/MOCK_C1)) / (1.0 + (1.0/Ra + 1.0/_mock_r2_true)*(dt/MOCK_C1));
    float v2_next = (_mock_v2 + (v1_next/_mock_r2_true)*(dt/MOCK_C2)) / (1.0 + (dt/(_mock_r2_true*MOCK_C2)));
    _mock_v1 = v1_next;
    _mock_v2 = v2_next;
    _mock_micros += dt_us;
}

void delay(unsigned long ms) {
    uint64_t end = _mock_micros + (uint64_t)ms * 1000;
    while(_mock_micros < end) _tickPhysics(100);
}

void delayMicroseconds(unsigned long us) {
    uint64_t end = _mock_micros + us;
    while(_mock_micros < end) {
        uint64_t rem = end - _mock_micros;
        _tickPhysics((unsigned long)(rem < 10 ? rem : 10));
    }
}

void setPhysicsR2(float r2) {
    _mock_r2_true = r2;
    _mock_micros = 0; _mock_v1 = 0; _mock_v2 = 0;
}
