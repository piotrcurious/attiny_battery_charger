#include <iostream>
#include <cassert>
#include "mock_arduino/Arduino.h"

void setup();
void loop();

#include "li-ion.ino"

int main() {
    std::cout << "Multi-Stage Thermal Constraint Test: li-ion.ino" << std::endl;
    resetMock();
    globalBattery.ocv_mv = 3700.0; // Ready for fast charge
    globalBattery.temp_c = 25.0;  // Standard zone
    setup();

    // 1. Standard Zone (10-35C) -> Target 500mA
    std::cout << "Testing Standard Zone (25C)..." << std::endl;
    for (int i=0; i<300; i++) {
        updateBatterySim(0.1);
        loop();
    }
    std::cout << "Temp=" << (int)smoothedTemperature << "C, Current=" << (int)smoothedCurrent << "mA" << std::endl;
    assert(smoothedCurrent > 450 && smoothedCurrent <= 550);

    // 2. Warm Zone (35-45C) -> Target 250mA
    std::cout << "Testing Warm Zone (40C)..." << std::endl;
    globalBattery.temp_c = 40.0;
    for (int i=0; i<300; i++) {
        updateBatterySim(0.1);
        loop();
    }
    std::cout << "Temp=" << (int)smoothedTemperature << "C, Current=" << (int)smoothedCurrent << "mA" << std::endl;
    assert(smoothedCurrent > 200 && smoothedCurrent <= 300);

    // 3. Cold Zone (0-10C) -> Target 100mA
    std::cout << "Testing Cold Zone (5C)..." << std::endl;
    globalBattery.temp_c = 5.0;
    for (int i=0; i<500; i++) { // Even more loops to overcome EMA
        updateBatterySim(0.1);
        loop();
    }
    std::cout << "Temp=" << (int)smoothedTemperature << "C, Current=" << (int)smoothedCurrent << "mA" << std::endl;
    assert(smoothedCurrent > 50 && smoothedCurrent <= 150);

    // 4. Over-temp Stop (>45C)
    std::cout << "Testing Over-temp (50C)..." << std::endl;
    globalBattery.temp_c = 50.0;
    for (int i=0; i<50; i++) {
        updateBatterySim(0.1);
        loop();
    }
    assert(charging == false);

    std::cout << "Multi-stage thermal constraint tests passed!" << std::endl;
    return 0;
}
