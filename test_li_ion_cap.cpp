#include <iostream>
#include <cassert>
#include "mock_arduino/Arduino.h"

// Forward declare setup and loop from the ino file
void setup();
void loop();

#include "li-ion_cap.ino"

int main() {
    std::cout << "Testing li-ion_cap.ino..." << std::endl;

    resetMock();
    setup();

    // Simulate R_ref = 10k, R_ntc = 10k (25C)
    setDischargeTime(REFERENCE_PIN, 1000);
    setDischargeTime(TEMPERATURE_PIN, 1000);

    loop();
    std::cout << "Measured temperature (expected ~25C): " << temperature << "C" << std::endl;
    assert(temperature >= 24 && temperature <= 26);

    // Simulate R_ntc = 4.35k (~45C)
    // t_ntc / t_ref = R_ntc / R_ref
    // t_ntc = 1000 * 4350 / 10000 = 435
    setDischargeTime(TEMPERATURE_PIN, 435);
    loop();
    std::cout << "Measured temperature (expected ~45C): " << temperature << "C" << std::endl;
    assert(temperature >= 43 && temperature <= 47);

    // Test charging logic with cap-based temp
    setAnalogValue(VOLTAGE_PIN, 500); // map(500, 0, 1023, 0, 5000) = 2443mV
    loop();
    assert(charging == true);

    // Test over temperature
    setDischargeTime(TEMPERATURE_PIN, 300); // Even lower R_ntc -> even higher temp
    loop();
    std::cout << "Measured temperature (hot): " << temperature << "C, Charging: " << charging << std::endl;
    assert(charging == false);

    std::cout << "li-ion_cap.ino tests passed!" << std::endl;
    return 0;
}
