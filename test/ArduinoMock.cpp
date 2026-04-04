#include "ArduinoMock.h"
#include <cmath>
#include <algorithm>

static uint64_t _mock_micros = 0;
static float _v1 = 0, _v2 = 0;
static int _vin_state = LOW;
static float _r2_true = 1000.0;

const float MOCK_R0 = 250.0, MOCK_R1 = 1.0, MOCK_C1 = 0.5e-6, MOCK_C2 = 10.0e-6;
const float Ra = MOCK_R0 + MOCK_R1;

MockSerial Serial;

void pinMode(int pin, int mode) {}
void digitalWrite(int pin, int state) { _vin_state = state; }

static void _tickPhysics(unsigned long dt_us) {
    if (dt_us == 0) return;
    float dt = dt_us / 1e6;
    float vin = (_vin_state == HIGH) ? 5.0 : 0.0;
    float r2 = _r2_true + 1e-9;

    // Matrix A elements
    float a11 = -(1.0/Ra + 1.0/r2) / MOCK_C1;
    float a12 = (1.0/r2) / MOCK_C1;
    float a21 = (1.0/r2) / MOCK_C2;
    float a22 = -(1.0/r2) / MOCK_C2;

    // Eigenvalues of A: lambda^2 - Tr(A)*lambda + Det(A) = 0
    float trA = a11 + a22;
    float detA = a11*a22 - a12*a21;
    float disc = trA*trA - 4.0*detA;
    if (disc < 0) disc = 0; // Should not happen for RC
    float sD = sqrt(disc);
    float l1 = (trA + sD) / 2.0;
    float l2 = (trA - sD) / 2.0;

    // Matrix exponential exp(At) = m1*I + m2*A
    float m1, m2;
    if (abs(l1 - l2) < 1e-9) {
        m1 = exp(l1 * dt) * (1.0 - l1 * dt);
        m2 = exp(l1 * dt) * dt;
    } else {
        float e1 = exp(l1 * dt);
        float e2 = exp(l2 * dt);
        m1 = (l1*e2 - l2*e1) / (l1 - l2);
        m2 = (e1 - e2) / (l1 - l2);
    }

    // exp(At) components
    float e11 = m1 + m2*a11;
    float e12 = m2*a12;
    float e21 = m2*a21;
    float e22 = m1 + m2*a22;

    // V(t+dt) = Vss + exp(At)*(V(t) - Vss)
    float dv1 = _v1 - vin;
    float dv2 = _v2 - vin;

    _v1 = vin + e11*dv1 + e12*dv2;
    _v2 = vin + e21*dv1 + e22*dv2;

    _mock_micros += dt_us;
}

int analogRead(int pin) {
    // real analogRead takes time
    _tickPhysics(100);
    int raw = (int)((_v2 / 5.0) * 1023.0);
    return std::max(0, std::min(1023, raw));
}

unsigned long millis() { return (unsigned long)(_mock_micros / 1000); }
unsigned long micros() { return (unsigned long)_mock_micros; }

void delay(unsigned long ms) {
    _tickPhysics(ms * 1000);
}

void delayMicroseconds(unsigned long us) {
    _tickPhysics(us);
}

void setPhysicsR2(float r2) {
    _r2_true = r2;
    _mock_micros = 0; _v1 = 0; _v2 = 0; _vin_state = LOW;
}

void advanceMillis(unsigned long ms) {
    // Advance time without physics? No, typically we want to settle.
    // If we advance without physics, we jump in time but voltages stay the same.
    // This is useful for bypassing millis() checks in loops.
    _mock_micros += (uint64_t)ms * 1000;
}
