#include <iostream>
#include <cassert>
#include "mock_arduino/Arduino.h"

void setup();
void loop();

#include "li-ion_cap.ino"

int main() {
    std::cout << "Testing li-ion_cap.ino with EMA and Discharge logic..." << std::endl;

    resetMock();
    setup();

    // Simulate R_ref = 10k, R_ntc = 10k (25C)
    setDischargeTime(REFERENCE_PIN, 1000);
    setDischargeTime(TEMPERATURE_PIN, 1000);

    for (int i=0; i<30; i++) loop(); // Stabilize EMA
    std::cout << "Measured temperature (expected ~25C): " << (int)smoothedTemperature << "C" << std::endl;
    assert(smoothedTemperature >= 24 && smoothedTemperature <= 26);

    // Simulate R_ntc = 4.35k (~45C)
    setDischargeTime(TEMPERATURE_PIN, 435);
    for (int i=0; i<30; i++) loop();
    std::cout << "Measured temperature (expected ~45C): " << (int)smoothedTemperature << "C" << std::endl;
    assert(smoothedTemperature >= 43 && smoothedTemperature <= 47);

    // Test charging logic with cap-based temp
    setAnalogValue(VOLTAGE_PIN, map(3500, 0, 5000, 0, 1023));
    for (int i=0; i<30; i++) loop();
    assert(charging == true);

    // Test over temperature
    setDischargeTime(TEMPERATURE_PIN, 300); // Higher temp
    for (int i=0; i<30; i++) loop();
    std::cout << "Measured temperature (hot): " << (int)smoothedTemperature << "C, Charging: " << charging << std::endl;
    assert(charging == false);

    std::cout << "li-ion_cap.ino tests passed!" << std::endl;
    return 0;
}
