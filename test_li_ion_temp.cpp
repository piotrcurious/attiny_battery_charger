#include <iostream>
#include <cassert>
#include "mock_arduino/Arduino.h"

void setup();
void loop();

#include "li-ion.ino"

int main() {
    std::cout << "Temperature Hysteresis Test: li-ion.ino" << std::endl;
    resetMock();
    globalBattery.ocv_mv = 3700.0;
    globalBattery.temp_c = 25.0;
    setup();

    // Normal charging
    for (int i=0; i<50; i++) {
        updateBatterySim(0.1);
        loop();
    }
    assert(charging == true);

    // Over-temperature
    std::cout << "Simulating over-temperature (50C)..." << std::endl;
    globalBattery.temp_c = 50.0;
    for (int i=0; i<50; i++) {
        updateBatterySim(0.1);
        loop();
    }
    std::cout << "Temp=" << (int)smoothedTemperature << "C, Charging=" << charging << ", temp_fault=" << temp_fault << std::endl;
    assert(charging == false);
    assert(temp_fault == true);

    // Temperature recovery - entering safe range but not recovery range yet
    std::cout << "Temperature recovery to 42C (above 40C recovery limit)..." << std::endl;
    globalBattery.temp_c = 42.0;
    for (int i=0; i<100; i++) {
        updateBatterySim(0.1);
        loop();
    }
    std::cout << "Temp=" << (int)smoothedTemperature << "C, Charging=" << charging << ", temp_fault=" << temp_fault << std::endl;
    assert(charging == false); // Should still be in fault

    // Temperature recovery - entering recovery range
    std::cout << "Temperature recovery to 38C (below 40C recovery limit)..." << std::endl;
    globalBattery.temp_c = 38.0;
    for (int i=0; i<100; i++) {
        updateBatterySim(0.1);
        loop();
    }
    std::cout << "Temp=" << (int)smoothedTemperature << "C, Charging=" << charging << ", temp_fault=" << temp_fault << std::endl;
    assert(charging == true); // Fault cleared

    std::cout << "Temperature hysteresis tests passed!" << std::endl;
    return 0;
}
