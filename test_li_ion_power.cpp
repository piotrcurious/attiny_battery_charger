#include <iostream>
#include <cassert>
#include "mock_arduino/Arduino.h"

void setup();
void loop();

#include "li-ion.ino"

void resetFirmwareVars() {
    charging = false;
    smoothedVoltage = 0;
    smoothedCurrent = 0;
    smoothedTemperature = 25;
    temp_fault = false;
    chargeValue = 0;
}

int main() {
    std::cout << "Power Management Tests: li-ion.ino" << std::endl;

    // 1. Standby Sleep Test
    std::cout << "Testing Standby Sleep (Battery Full)..." << std::endl;
    resetMock();
    resetFirmwareVars();
    globalBattery.ocv_mv = 4200.0;
    setup();

    for(int i=0; i<100; i++) {
        updateBatterySim(0.1);
        loop();
    }

    resetSleepCount();
    updateBatterySim(0.1);
    loop();
    std::cout << "Sleep count: " << sleepCallCount << ", charging: " << charging << ", smoothedVoltage: " << (int)smoothedVoltage << "mV" << std::endl;
    assert(sleepCallCount == 1);
    assert(charging == false);

    // 2. Eco Mode Sleep Test
    std::cout << "Testing Eco Mode (Battery Critically Low < 2000mV)..." << std::endl;
    resetMock();
    resetFirmwareVars();
    globalBattery.ocv_mv = 1500.0;
    setup();

    for(int i=0; i<100; i++) {
        updateBatterySim(0.1);
        loop();
    }

    resetSleepCount();
    updateBatterySim(0.1);
    loop();
    std::cout << "Sleep count: " << sleepCallCount << ", charging: " << charging << ", smoothedVoltage: " << (int)smoothedVoltage << "mV" << std::endl;
    assert(sleepCallCount == 8);
    assert(charging == false);

    // 3. Charging (No Sleep) Test
    std::cout << "Testing Charging (No Sleep)..." << std::endl;
    resetMock();
    resetFirmwareVars();
    globalBattery.ocv_mv = 3700.0;
    setup();

    for(int i=0; i<100; i++) {
        updateBatterySim(0.1);
        loop();
    }

    resetSleepCount();
    updateBatterySim(0.1);
    loop();
    std::cout << "Sleep count: " << sleepCallCount << ", charging: " << charging << std::endl;
    assert(sleepCallCount == 0);
    assert(charging == true);

    std::cout << "Power management tests passed!" << std::endl;
    return 0;
}
