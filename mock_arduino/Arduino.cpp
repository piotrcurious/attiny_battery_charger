#include "Arduino.h"
#include <chrono>
#include <map>

uint8_t MCUSR = 0;
uint8_t WDTCSR = 0;
uint8_t ACSR = 0;
uint8_t ADCSRA = 0;
uint8_t ADMUX = 0;

static int analogValues[8] = {0};
static int digitalValues[10] = {0};
static int pwmValues[10] = {0};
static unsigned long _millis = 0;
static unsigned long _micros = 0;

static std::map<uint8_t, int> pin_discharge_times;
static std::map<uint8_t, unsigned long> pin_start_times;

BatterySim globalBattery = {3700.0, 0.1, 2000.0, 0.0, 25.0};
int sleepCallCount = 0;

void resetMock() {
    for (int i=0; i<8; i++) analogValues[i] = 0;
    for (int i=0; i<10; i++) {
        digitalValues[i] = 0;
        pwmValues[i] = 0;
    }
    _millis = 0;
    _micros = 0;
    MCUSR = 0;
    WDTCSR = 0;
    ACSR = 0;
    ADCSRA = 0x80; // ADEN set by default
    ADMUX = 0;
    pin_discharge_times.clear();
    pin_start_times.clear();
    globalBattery = {3700.0, 0.1, 2000.0, 0.0, 25.0};
    sleepCallCount = 0;
}

void resetSleepCount() { sleepCallCount = 0; }

void setAnalogValue(uint8_t pin, int value) {
    if (pin < 8) analogValues[pin] = value;
}

void setDigitalValue(uint8_t pin, int value) {
    if (pin < 10) digitalValues[pin] = value;
}

int getDigitalValue(uint8_t pin) {
    if (pin < 10) return digitalValues[pin];
    return 0;
}

int getPWMValue(uint8_t pin) {
    if (pin < 10) return pwmValues[pin];
    return 0;
}

void pinMode(uint8_t pin, uint8_t mode) {
    if (mode == INPUT) {
        pin_start_times[pin] = _micros;
    }
}

void digitalWrite(uint8_t pin, uint8_t val) {
    if (pin < 10) digitalValues[pin] = val;
}

int digitalRead(uint8_t pin) {
    if (pin_discharge_times.count(pin)) {
        if (_micros - pin_start_times[pin] >= (unsigned long)pin_discharge_times[pin]) {
            return LOW;
        }
        _micros += 1;
        return HIGH;
    }
    if (pin < 10) return digitalValues[pin];
    return 0;
}

int analogRead(uint8_t pin) {
    if (pin < 8) return analogValues[pin];
    return 0;
}

void analogWrite(uint8_t pin, int val) {
    if (pin < 10) pwmValues[pin] = val;
}

void delay(unsigned long ms) { _micros += ms * 1000; _millis += ms; }
void delayMicroseconds(unsigned int us) { _micros += us; _millis += us / 1000; }
unsigned long micros() { return _micros; }
unsigned long millis() { return _millis; }

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setDischargeTime(uint8_t pin, int time_us) {
    pin_discharge_times[pin] = time_us;
}

void updateBatterySim(float dt_s, float charge_efficiency) {
    float max_ma = 500.0;
    globalBattery.current_ma = (float)pwmValues[0] / 255.0 * max_ma;

    float delta_mah = globalBattery.current_ma * dt_s / 3600.0 * charge_efficiency;
    globalBattery.ocv_mv += delta_mah / globalBattery.capacity_mah * (4200.0 - 3000.0);

    float terminal_voltage_mv = globalBattery.ocv_mv + globalBattery.ir_ohms * globalBattery.current_ma;

    analogValues[1] = (int)(terminal_voltage_mv / 5000.0 * 1023.0);
    analogValues[2] = (int)(globalBattery.current_ma / 1000.0 * 1023.0);
    analogValues[3] = (int)((globalBattery.temp_c + 40.0) / 165.0 * 1023.0);

    float T = globalBattery.temp_c + 273.15;
    float R_ntc = 10000.0 * exp(3950.0 * (1.0/T - 1.0/298.15));
    int t_ntc = (int)(1000.0 * R_ntc / 10000.0);
    pin_discharge_times[5] = 1000;
    pin_discharge_times[3] = t_ntc;
}

void set_sleep_mode(int mode) {}
void sleep_enable() {}
void sleep_mode() { sleepCallCount++; _millis += 8000; _micros += 8000000; } // Assuming 8s WDT sleep
void sleep_disable() {}

MockSerial Serial;
