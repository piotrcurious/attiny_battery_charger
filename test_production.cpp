#include <iostream>
#include <cassert>
#include "mock_arduino/Arduino.h"

void setup();
void loop();

#include "li-ion.ino"

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
    std::cout << "Production Quality Tests: li-ion.ino" << std::endl;

    // 1. Fixed-point precision test
    std::cout << "Testing Fixed-point precision..." << std::endl;
    resetMock();
    resetFirmwareVars();
    globalBattery.ocv_mv = 3700.0;
    globalBattery.ir_ohms = 0.0; // Remove IR for simple precision check
    setup();
    for(int i=0; i<300; i++) {
        updateBatterySim(0.1);
        loop();
    }
    std::cout << "Smoothed Voltage: " << (int)smoothedVoltage << "mV, Current: " << (int)smoothedCurrent << "mA" << std::endl;
    assert(smoothedVoltage >= 3690 && smoothedVoltage <= 3750);
    assert(smoothedCurrent > 450);

    // 2. Sensor Fault Test (Open Circuit)
    std::cout << "Testing Sensor Fault (Open Circuit Voltage Pin)..." << std::endl;
    resetMock();
    resetFirmwareVars();
    setup();
    setAnalogValue(VOLTAGE_PIN, 1023); // Open circuit
    loop();
    std::cout << "sensor_fault=" << sensor_fault << ", charging=" << charging << std::endl;
    assert(sensor_fault == true);
    assert(charging == false);

    // 3. Sensor Fault Test (Short to GND)
    std::cout << "Testing Sensor Fault (Short GND Temperature Pin)..." << std::endl;
    resetMock();
    resetFirmwareVars();
    setup();
    setAnalogValue(TEMPERATURE_PIN, 0); // Short to GND
    loop();
    std::cout << "sensor_fault=" << sensor_fault << ", charging=" << charging << std::endl;
    assert(sensor_fault == true);
    assert(charging == false);

    std::cout << "Production tests passed!" << std::endl;
    return 0;
}
