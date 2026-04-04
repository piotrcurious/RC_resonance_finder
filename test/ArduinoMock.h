#ifndef ArduinoMock_h
#define ArduinoMock_h
#include <iostream>
#include <cmath>
#include <cstdint>
#define HIGH 0x1
#define LOW  0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define A0 0
#ifndef PI
#define PI 3.14159265358979323846
#endif
typedef uint8_t byte;
void pinMode(int pin, int mode);
void digitalWrite(int pin, int state);
int analogRead(int pin);
unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned long us);
class MockSerial {
public:
    void begin(unsigned long baud) {}
    template<typename T> void print(T val) { std::cout << val; }
    template<typename T> void println(T val) { std::cout << val << std::endl; }
    void println() { std::cout << "\n"; }
};
extern MockSerial Serial;
void setPhysicsR2(float r2);
void advanceMillis(unsigned long ms);
#endif
