#include <iostream>
#include <cassert>
#include "mock_arduino/Arduino.h"

void setup();
void loop();

#include "li-ion.ino"

int main() {
    std::cout << "Advanced Simulation: Empty Battery (Pre-charging?)" << std::endl;
    resetMock();
    globalBattery.ocv_mv = 2800.0; // Deeply discharged
    globalBattery.ir_ohms = 0.2;
    setup();

    // Need to initialize sensors for EMA
    setAnalogValue(VOLTAGE_PIN, map(2800, 0, 5000, 0, 1023));
    setAnalogValue(TEMPERATURE_PIN, map(25, -40, 125, 0, 1023));

    for (int i=0; i<100; i++) {
        updateBatterySim(0.1);
        loop();
        if (i % 20 == 0) {
            std::cout << "t=" << i*0.1 << "s: V_term=" << smoothedVoltage << "mV, I=" << smoothedCurrent << "mA, PWM=" << (int)chargeValue << ", charging=" << charging << std::endl;
        }
    }

    std::cout << "After 10s: OCV=" << globalBattery.ocv_mv << "mV, Charging=" << charging << ", PWM=" << (int)chargeValue << std::endl;

    std::cout << "\nAdvanced Simulation: CC/CV Transition" << std::endl;
    resetMock();
    globalBattery.ocv_mv = 4150.0;
    setup();
    setAnalogValue(VOLTAGE_PIN, map(4150, 0, 5000, 0, 1023));
    setAnalogValue(TEMPERATURE_PIN, map(25, -40, 125, 0, 1023));

    for (int i=0; i<300; i++) {
        updateBatterySim(0.1);
        loop();
        if (i % 50 == 0) {
           std::cout << "t=" << i*0.1 << "s: OCV=" << globalBattery.ocv_mv << "mV, V_term=" << (int)smoothedVoltage << "mV, I=" << (int)smoothedCurrent << "mA, PWM=" << (int)chargeValue << std::endl;
        }
    }

    std::cout << "\nAdvanced Simulation: Full Battery (Termination)" << std::endl;
    resetMock();
    globalBattery.ocv_mv = 4250.0;
    setup();
    setAnalogValue(VOLTAGE_PIN, map(4250, 0, 5000, 0, 1023));
    setAnalogValue(TEMPERATURE_PIN, map(25, -40, 125, 0, 1023));
    for (int i=0; i<50; i++) {
        updateBatterySim(0.1);
        loop();
    }
    std::cout << "After 5s: OCV=" << globalBattery.ocv_mv << "mV, Charging=" << charging << ", PWM=" << (int)chargeValue << std::endl;
    assert(charging == false);

    std::cout << "Advanced simulation tests completed!" << std::endl;
    return 0;
}
