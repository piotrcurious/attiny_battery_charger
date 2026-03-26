#ifndef ARDUINO_H
#define ARDUINO_H

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

typedef uint8_t byte;
typedef bool boolean;

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define bit(b) (1UL << (b))

extern uint8_t MCUSR;
extern uint8_t WDTCSR;
extern uint8_t ACSR;
extern uint8_t ADCSRA;
extern uint8_t ADMUX;

#define WDP0 0
#define WDP1 1
#define WDP2 2
#define WDP3 5
#define WDE  3
#define WDCE 4
#define WDRF 3
#define WDTIE 6

#define ACBG 6
#define ACIE 3
#define ACI  4

#define ADEN 7

#define ANALOG_COMP_vect
#define ISR(vector) void vector##_impl()

#define SLEEP_MODE_PWR_DOWN 0
void set_sleep_mode(int mode);
void sleep_enable();
void sleep_mode();
void sleep_disable();

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int val);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
unsigned long micros();
unsigned long millis();

long map(long x, long in_min, long in_max, long out_min, long out_max);

class MockSerial {
public:
    void begin(long speed) {}
    void print(const char* s) { printf("%s", s); }
    void print(float f) { printf("%f", f); }
    void print(int i) { printf("%d", i); }
    void println(const char* s) { printf("%s\n", s); }
    void println(float f) { printf("%f\n", f); }
    void println(int i) { printf("%d\n", i); }
};
extern MockSerial Serial;

// Mock control and simulation
void setAnalogValue(uint8_t pin, int value);
void setDigitalValue(uint8_t pin, int value);
int getDigitalValue(uint8_t pin);
int getPWMValue(uint8_t pin);
void resetMock();
void setDischargeTime(uint8_t pin, int time_us);

// Sleep tracking
extern int sleepCallCount;
void resetSleepCount();

// Battery and Environment simulation
struct BatterySim {
    float ocv_mv;
    float ir_ohms;
    float capacity_mah;
    float current_ma;
    float temp_c;
};

extern BatterySim globalBattery;
void updateBatterySim(float dt_s, float charge_efficiency = 1.0);

#endif
