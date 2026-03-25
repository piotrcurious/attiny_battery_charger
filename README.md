# attiny_battery_charger
Battery charger based on ATtiny13.

## Features
- Li-Ion battery charging with CC/CV (simplified)
- Watchdog safety
- Deep sleep (in real hardware)
- Temperature safety protection
- Optional load PWM control

## Files
- `li-ion.ino`: Basic Li-Ion charger.
- `li-ion_with_load.ino`: Li-Ion charger with load control.
- `li-ion_cap.ino`: Li-Ion charger with load control and capacitor-based temperature measurement.

## Testing
A mock Arduino environment is provided in the `mock_arduino/` directory.
Tests can be run using the provided C++ test runners:
- `test_li_ion.cpp`
- `test_li_ion_load.cpp`
- `test_li_ion_cap.cpp`

Example:
```bash
g++ -Imock_arduino mock_arduino/Arduino.cpp test_li_ion.cpp -o test_li_ion && ./test_li_ion
```
