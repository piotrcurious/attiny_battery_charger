// Arduino code for attiny13 li-ion battery charger with load control and thermistor temperature measurement
// Features: maximum charge voltage, wdt safety, deep sleep, over current protection, thermal safety protection, load PWM control, thermistor temperature measurement
// Disclaimer: This code is for educational purposes only and not intended to be used in real applications.
// Always verify the code and test it with proper equipment before using it with your battery, load, and thermistor.
// Make sure you have a battery protection board or circuit to prevent overcharging, overdischarging, short-circuiting, or overheating of your battery.

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

// Define the pins
#define CHARGE_PIN 0 // PWM pin to control the charging MOSFET
#define VOLTAGE_PIN 1 // Analog pin to measure the battery voltage
#define CURRENT_PIN 2 // Analog pin to measure the charging current
#define TEMPERATURE_PIN 3 // Analog pin to measure the thermistor voltage
#define LOAD_PIN 4 // PWM pin to control the load MOSFET
#define REFERENCE_PIN 5 // Digital pin to charge and discharge the capacitor through the reference resistor

// Define the constants
#define MAX_VOLTAGE 4200 // Maximum charge voltage in millivolts
#define MIN_VOLTAGE 3000 // Minimum voltage to start charging in millivolts
#define MAX_CURRENT 500 // Maximum charge current in milliamps
#define MIN_CURRENT 50 // Minimum current to terminate charging in milliamps
#define MAX_TEMPERATURE 45 // Maximum temperature to charge in degrees Celsius
#define MIN_TEMPERATURE 0 // Minimum temperature to charge in degrees Celsius
#define WDT_TIMEOUT 8 // Watchdog timer timeout in seconds
#define SLEEP_TIME 60 // Sleep time between measurements in seconds
#define MAX_LOAD_VOLTAGE 4000 // Maximum battery voltage to allow maximum load PWM in millivolts
#define MIN_LOAD_VOLTAGE 3200 // Minimum battery voltage to allow minimum load PWM in millivolts
#define MAX_LOAD_PWM 255 // Maximum load PWM value
#define MIN_LOAD_PWM 0 // Minimum load PWM value
#define REFERENCE_RESISTOR 10000 // Reference resistor value in ohms
#define THERMISTOR_NOMINAL_RESISTOR 10000 // Thermistor resistor value at 25 degrees Celsius in ohms
#define THERMISTOR_BETA 3950 // Thermistor beta coefficient in Kelvin
#define INTERNAL_REFERENCE 1100 // Internal reference voltage in millivolts

// Define the variables
int chargeValue = 0; // PWM value to control the charging MOSFET
int voltageValue = 0; // Analog value to measure the battery voltage
int currentValue = 0; // Analog value to measure the charging current
int temperatureValue = 0; // Analog value to measure the thermistor voltage
int loadValue = 0; // PWM value to control the load MOSFET
int voltage = 0; // Battery voltage in millivolts
int current = 0; // Charging current in milliamps
int temperature = 0; // Battery temperature in degrees Celsius
bool charging = false; // Charging status flag
float supplyVoltage = 5000; // Default supply voltage in millivolts

// Forward declarations for functions
void reset_watchdog();
void stop_charging();

// Initialize the watchdog timer
void setup_watchdog(int timeout) {
  byte wdtcsr = 0;
  if (timeout > 0) {
    // Enable the watchdog timer
    wdtcsr = bit(WDCE) | bit(WDE);
    // Set the timeout
    switch (timeout) {
      case 1: wdtcsr |= bit(WDP0); break; // 16 ms
      case 2: wdtcsr |= bit(WDP1); break; // 32 ms
      case 4: wdtcsr |= bit(WDP1) | bit(WDP0); break; // 64 ms
      case 8: wdtcsr |= bit(WDP2); break; // 0.125 s
      case 16: wdtcsr |= bit(WDP2) | bit(WDP0); break; // 0.25 s
      case 32: wdtcsr |= bit(WDP2) | bit(WDP1); break; // 0.5 s
      case 64: wdtcsr |= bit(WDP2) | bit(WDP1) | bit(WDP0); break; // 1 s
      case 128: wdtcsr |= bit(WDP3); break; // 2 s
      case 256: wdtcsr |= bit(WDP3) | bit(WDP0); break; // 4 s
      default: wdtcsr |= bit(WDP3) | bit(WDP1); break; // 8 s
    }
    // Reset the watchdog timer
    reset_watchdog();
    // Start the watchdog timer
    MCUSR &= ~bit(WDRF);
    WDTCR |= bit(WDCE) | bit(WDE);
    WDTCR = wdtcsr;
  }
  else {
    // Disable the watchdog timer
    MCUSR &= ~bit(WDRF);
    WDTCR |= bit(WDCE) | bit(WDE);
    WDTCR = 0x00;
  }
}

// Reset the watchdog timer
void reset_watchdog() {
  wdt_reset();
}

// Enter sleep mode
void enter_sleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set the sleep mode
  sleep_enable(); // Enable sleep mode
  sleep_mode(); // Enter sleep mode
  sleep_disable(); // Disable sleep mode after waking up
}

// Read the battery voltage
void read_voltage() {
  voltageValue = analogRead(VOLTAGE_PIN); // Read the analog value
  voltage = map(voltageValue, 0, 1023, 0, (int)supplyVoltage); // Map the value to millivolts using current supply voltage
}

// Read the charging current
void read_current() {
  currentValue = analogRead(CURRENT_PIN); // Read the analog value
  current = map(currentValue, 0, 1023, 0, 1000); // Map the value to milliamps
}

// Measure discharge time of a capacitor on a pin
unsigned long measure_discharge_time(uint8_t pin) {
  // Charge the capacitor to Vcc
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(10);

  // Start discharging and measure time to reach logic LOW
  pinMode(pin, INPUT);
  unsigned long startTime = micros();
  while (digitalRead(pin) == HIGH) {
      if (micros() - startTime > 100000) break; // Timeout 100ms
  }
  unsigned long endTime = micros();
  return endTime - startTime;
}

// Read the thermistor and supply voltage
void read_temperature_and_supply() {
  unsigned long t_ref = measure_discharge_time(REFERENCE_PIN);
  unsigned long t_ntc = measure_discharge_time(TEMPERATURE_PIN);

  if (t_ref > 0 && t_ntc > 0) {
      // t = R * C * ln(Vcc / Vth)
      // Since C and ln(Vcc / Vth) are approximately constant:
      // t_ntc / t_ref = R_ntc / R_ref
      float thermistorResistance = (float)REFERENCE_RESISTOR * (float)t_ntc / (float)t_ref;

      // Calculate temperature using Steinhart-Hart (Beta model)
      float steinhart;
      steinhart = thermistorResistance / THERMISTOR_NOMINAL_RESISTOR; // (R/Ro)
      steinhart = log(steinhart);                  // ln(R/Ro)
      steinhart /= THERMISTOR_BETA;                // 1/B * ln(R/Ro)
      steinhart += 1.0 / (25.0 + 273.15);          // + (1/To)
      steinhart = 1.0 / steinhart;                 // Invert
      steinhart -= 273.15;                         // convert to C

      temperature = (int)steinhart;
  }
}

// Start charging
void start_charging() {
  charging = true; // Set the charging flag
  chargeValue = 255; // Set the PWM value to maximum
  analogWrite(CHARGE_PIN, chargeValue); // Write the PWM value to the pin
}

// Stop charging
void stop_charging() {
  charging = false; // Clear the charging flag
  chargeValue = 0; // Set the PWM value to zero
  analogWrite(CHARGE_PIN, chargeValue); // Write the PWM value to the pin
}

// Adjust charging
void adjust_charging() {
  if (voltage >= MAX_VOLTAGE) {
    // Battery is fully charged, stop charging
    stop_charging();
  }
  else if (current >= MAX_CURRENT) {
    // Charging current is too high, reduce the PWM value
    if (chargeValue > 0) chargeValue--;
    analogWrite(CHARGE_PIN, chargeValue);
  }
  else if (current <= MIN_CURRENT && voltage >= MIN_VOLTAGE) {
    // Charging current is too low, increase the PWM value
    if (chargeValue < 255) chargeValue++;
    analogWrite(CHARGE_PIN, chargeValue);
  }
}

// Check the battery temperature
void check_temperature() {
  if (temperature >= MAX_TEMPERATURE || temperature <= MIN_TEMPERATURE) {
    // Temperature is out of range, stop charging
    stop_charging();
  }
}

// Control the load
void control_load() {
  if (voltage >= MAX_LOAD_VOLTAGE) {
    loadValue = MAX_LOAD_PWM;
  }
  else if (voltage <= MIN_LOAD_VOLTAGE) {
    loadValue = MIN_LOAD_PWM;
  }
  else {
    loadValue = map(voltage, MIN_LOAD_VOLTAGE, MAX_LOAD_VOLTAGE, MIN_LOAD_PWM, MAX_LOAD_PWM);
  }
  analogWrite(LOAD_PIN, loadValue);
}

// Setup the pins and the watchdog timer
void setup() {
  pinMode(CHARGE_PIN, OUTPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);
  pinMode(TEMPERATURE_PIN, INPUT);
  pinMode(LOAD_PIN, OUTPUT);
  pinMode(REFERENCE_PIN, INPUT);
  setup_watchdog(WDT_TIMEOUT);
}

// Loop the measurements and the charging logic
void loop() {
  read_temperature_and_supply();
  read_voltage();
  read_current();
  if (voltage <= MIN_VOLTAGE) {
    start_charging();
  }
  else if (charging) {
    adjust_charging();
  }
  check_temperature();
  control_load();
  reset_watchdog();
  delay(10);
}
