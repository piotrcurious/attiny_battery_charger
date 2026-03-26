#include <iostream>
#include <cassert>
#include <vector>
#include "mock_arduino/Arduino.h"

void setup();
void loop();

#include "li-ion_with_load.ino"

int main() {
    std::cout << "Testing li-ion_with_load.ino with EMA and Load Scaling..." << std::endl;

    resetMock();
    setup();

    // Low battery
    setAnalogValue(VOLTAGE_PIN, map(3100, 0, 5000, 0, 1023));
    setAnalogValue(TEMPERATURE_PIN, map(25, -40, 125, 0, 1023));
    for (int i=0; i<20; i++) loop(); // Stabilize
    std::cout << "Voltage: " << (int)smoothedVoltage << "mV, Charging: " << charging << ", Load PWM: " << getPWMValue(LOAD_PIN) << std::endl;
    assert(charging == true);
    assert(getPWMValue(LOAD_PIN) == MIN_LOAD_PWM);

    // Medium battery
    setAnalogValue(VOLTAGE_PIN, map(3600, 0, 5000, 0, 1023));
    for (int i=0; i<20; i++) loop();
    std::cout << "Voltage: " << (int)smoothedVoltage << "mV, Load PWM: " << getPWMValue(LOAD_PIN) << std::endl;
    assert(getPWMValue(LOAD_PIN) > MIN_LOAD_PWM && getPWMValue(LOAD_PIN) < MAX_LOAD_PWM);

    // High battery
    setAnalogValue(VOLTAGE_PIN, map(4100, 0, 5000, 0, 1023));
    for (int i=0; i<20; i++) loop();
    std::cout << "Voltage: " << (int)smoothedVoltage << "mV, Load PWM: " << getPWMValue(LOAD_PIN) << std::endl;
    assert(getPWMValue(LOAD_PIN) == MAX_LOAD_PWM);

    std::cout << "li-ion_with_load.ino tests passed!" << std::endl;
    return 0;
}
