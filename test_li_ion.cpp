#include <iostream>
#include <cassert>
#include "mock_arduino/Arduino.h"

// Forward declare setup and loop from the ino file
void setup();
void loop();

// Define charging and other variables if they are not already in scope from the ino
// But since we include the ino, they will be there.

#include "li-ion.ino"

int main() {
    std::cout << "Testing li-ion.ino..." << std::endl;

    // Test setup
    resetMock();
    setup();

    // Test battery voltage reading - too low, should start charging
    setAnalogValue(VOLTAGE_PIN, 500); // map(500, 0, 1023, 0, 5000) = 2443mV (< 3000mV)
    setAnalogValue(TEMPERATURE_PIN, 512); // map(512, 0, 1023, -40, 125) = 42.5C
    loop();

    std::cout << "Voltage: " << voltage << "mV, Charging: " << charging << ", PWM: " << getPWMValue(CHARGE_PIN) << std::endl;
    assert(charging == true);
    assert(getPWMValue(CHARGE_PIN) == 255);

    // Test battery voltage reading - high enough to stop charging
    setAnalogValue(VOLTAGE_PIN, 900); // map(900, 0, 1023, 0, 5000) = 4398mV (> 4200mV)
    loop();
    std::cout << "Voltage: " << voltage << "mV, Charging: " << charging << ", PWM: " << getPWMValue(CHARGE_PIN) << std::endl;
    assert(charging == false);
    assert(getPWMValue(CHARGE_PIN) == 0);

    // Test temperature safety - too hot
    setAnalogValue(VOLTAGE_PIN, 600); // map(600, 0, 1023, 0, 5000) = 2932mV (< 3000mV)
    setAnalogValue(TEMPERATURE_PIN, 900); // map(900, 0, 1023, -40, 125) = 105C
    loop();
    std::cout << "Temperature: " << temperature << "C, Charging: " << charging << std::endl;
    assert(charging == false);

    std::cout << "li-ion.ino tests passed!" << std::endl;
    return 0;
}
