#include <iostream>
#include <cassert>
#include "mock_arduino/Arduino.h"

// Forward declare setup and loop from the ino file
void setup();
void loop();

#include "li-ion_with_load.ino"

int main() {
    std::cout << "Testing li-ion_with_load.ino..." << std::endl;

    // Test setup
    resetMock();
    setup();

    // Test battery voltage reading - too low, should start charging and disable load
    setAnalogValue(VOLTAGE_PIN, 500); // ~2443mV
    setAnalogValue(TEMPERATURE_PIN, 512); // ~42.5C
    loop();

    std::cout << "Voltage: " << voltage << "mV, Charging: " << charging << ", Load PWM: " << getPWMValue(LOAD_PIN) << std::endl;
    assert(charging == true);
    assert(getPWMValue(LOAD_PIN) == MIN_LOAD_PWM);

    // Test battery voltage reading - half-charged, should scale load
    setAnalogValue(VOLTAGE_PIN, 737); // map(737, 0, 1023, 0, 5000) = 3602mV
    loop();
    std::cout << "Voltage: " << voltage << "mV, Load PWM: " << getPWMValue(LOAD_PIN) << std::endl;
    assert(getPWMValue(LOAD_PIN) > MIN_LOAD_PWM && getPWMValue(LOAD_PIN) < MAX_LOAD_PWM);

    // Test battery voltage reading - high voltage, should allow full load
    setAnalogValue(VOLTAGE_PIN, 819); // map(819, 0, 1023, 0, 5000) = 4002mV
    loop();
    std::cout << "Voltage: " << voltage << "mV, Load PWM: " << getPWMValue(LOAD_PIN) << std::endl;
    assert(getPWMValue(LOAD_PIN) == MAX_LOAD_PWM);

    std::cout << "li-ion_with_load.ino tests passed!" << std::endl;
    return 0;
}
