#include <iostream>
#include <cassert>
#include "mock_arduino/Arduino.h"

void setup();
void loop();

#include "li-ion_cap.ino"

void resetFirmwareVars() {
    charging = false;
    smoothedVoltageQ8 = 0;
    smoothedCurrentQ8 = 0;
    smoothedTemperatureQ8 = 25 << 8;
    temp_fault = false;
    sensor_fault = false;
    chargeValue = 0;
    first_run = true;
}

int main() {
    std::cout << "Testing li-ion_cap.ino Production Quality..." << std::endl;

    resetMock();
    resetFirmwareVars();
    setup();

    // Test 1: 25C logic
    std::cout << "Testing 25C..." << std::endl;
    setDischargeTime(REFERENCE_PIN, 1000);
    setDischargeTime(TEMPERATURE_PIN, 1000); // Ratio = 1.0 (1024 q10)

    for (int i=0; i<100; i++) {
        setAnalogValue(VOLTAGE_PIN, map(3500, 0, 5000, 0, 1023));
        loop();
    }
    std::cout << "Measured temperature: " << (int)smoothedTemperature << "C" << std::endl;
    assert(smoothedTemperature >= 24 && smoothedTemperature <= 26);

    // Test 2: 45C logic
    std::cout << "Testing 45C..." << std::endl;
    setDischargeTime(TEMPERATURE_PIN, 439);
    for (int i=0; i<500; i++) {
        setAnalogValue(VOLTAGE_PIN, map(3500, 0, 5000, 0, 1023));
        loop();
    }
    std::cout << "Measured temperature: " << (int)smoothedTemperature << "C" << std::endl;
    assert(smoothedTemperature >= 43 && smoothedTemperature <= 47);

    // Test 3: 0C logic
    std::cout << "Testing 0C..." << std::endl;
    setDischargeTime(TEMPERATURE_PIN, 3200);
    for (int i=0; i<500; i++) {
        setAnalogValue(VOLTAGE_PIN, map(3500, 0, 5000, 0, 1023));
        loop();
    }
    std::cout << "Measured temperature: " << (int)smoothedTemperature << "C" << std::endl;
    assert(smoothedTemperature >= -2 && smoothedTemperature <= 2);

    std::cout << "li-ion_cap.ino production tests passed!" << std::endl;
    return 0;
}
