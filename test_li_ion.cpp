#include <iostream>
#include <cassert>
#include <vector>
#include <numeric>
#include "mock_arduino/Arduino.h"

void setup();
void loop();

#include "li-ion.ino"

int main() {
    std::cout << "Testing li-ion.ino with EMA and Hysteresis..." << std::endl;

    resetMock();
    setup();

    // Simulate battery at 3500mV (should start charging)
    setAnalogValue(VOLTAGE_PIN, map(3500, 0, 5000, 0, 1023));
    setAnalogValue(TEMPERATURE_PIN, map(25, -40, 125, 0, 1023));
    for (int i=0; i<10; i++) loop();
    std::cout << "Smoothed Voltage: " << (int)smoothedVoltage << "mV, Charging: " << charging << std::endl;
    assert(charging == true);

    // Test recharge voltage (hysteresis)
    // First, reach full charge
    setAnalogValue(VOLTAGE_PIN, map(4300, 0, 5000, 0, 1023));
    for (int i=0; i<20; i++) loop(); // Stabilize EMA
    std::cout << "Voltage reached: " << (int)smoothedVoltage << "mV, Charging: " << charging << std::endl;
    assert(charging == false);

    // Drop to RECHARGE_VOLTAGE - 10
    setAnalogValue(VOLTAGE_PIN, map(RECHARGE_VOLTAGE - 10, 0, 5000, 0, 1023));
    for (int i=0; i<20; i++) loop(); // Stabilize EMA
    std::cout << "Voltage dropped to: " << (int)smoothedVoltage << "mV, Charging: " << charging << std::endl;
    assert(charging == true);

    std::cout << "li-ion.ino tests passed!" << std::endl;
    return 0;
}
