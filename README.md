# ATtiny13 Li-Ion Battery Charger

A robust, production-quality Li-Ion battery charger firmware designed for the ATtiny13 microcontroller.

## Features
- **CC/CV Charging**: Managed via hardware PWM.
- **Longevity-Optimized Thermal Control**: Adjusts charging current based on temperature zones (Cold, Standard, Warm).
- **Advanced Power Management**: Uses `SLEEP_MODE_PWR_DOWN` and Watchdog Timer interrupts. Features an 'Eco Mode' for deeply discharged batteries.
- **Production-Ready Logic**: Implemented entirely with fixed-point integer math to fit within 1KB Flash. No `float` or heavy `math.h` dependencies.
- **Robustness**:
    - Exponential Moving Average (EMA) filtering for all sensors.
    - Hysteresis on voltage and temperature thresholds.
    - Sensor sanity checks (detects open/short sensor faults).

## Technical Specifications
- **Target MCU**: ATtiny13/ATtiny13A
- **Clock**: 1.2MHz or 9.6MHz (Internal Oscillator)
- **Memory Usage**: Designed to fit < 1024 bytes Flash.

### Pinout (ATtiny13)
- **PB0 (Pin 5)**: `CHARGE_PIN` (Hardware PWM OC0A)
- **PB1 (Pin 6)**: `LOAD_PIN` (Hardware PWM OC0B)
- **PB2 (Pin 7)**: `VOLTAGE_PIN` (ADC1 - Battery Voltage)
- **PB3 (Pin 2)**: `TEMPERATURE_PIN` (ADC3 or Cap-based discharge)
- **PB4 (Pin 3)**: `CURRENT_PIN` (ADC2 - Charging Current)
- **PB5 (Pin 1)**: `REFERENCE_PIN` (Cap-based reference - Optional)

## Testing
A comprehensive mock Arduino environment and test suite are included.

### Run Tests
```bash
# Basic functional tests
g++ -Imock_arduino mock_arduino/Arduino.cpp test_li_ion.cpp -o test_li_ion && ./test_li_ion

# Production quality and sensor fault tests
g++ -Imock_arduino mock_arduino/Arduino.cpp test_production.cpp -o test_production && ./test_production

# Advanced battery simulation
g++ -Imock_arduino mock_arduino/Arduino.cpp test_li_ion_adv.cpp -o test_li_ion_adv && ./test_li_ion_adv
```

## Files
- `li-ion.ino`: Core charger logic.
- `li-ion_with_load.ino`: Charger with load PWM control.
- `li-ion_cap.ino`: Charger with capacitor-based ratiometric temperature sensing.
