// Arduino code for attiny13 li-ion battery charger
// Features: CC/CV, Pre-charging, Hysteresis, Smoothing, WDT Safety, Multi-stage Thermal Control
// Disclaimer: This code is for educational purposes only and not intended to be used in real applications.

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

// Define the pins
#define CHARGE_PIN 0 // PWM pin to control the charging MOSFET
#define VOLTAGE_PIN 1 // Analog pin to measure the battery voltage
#define CURRENT_PIN 2 // Analog pin to measure the charging current
#define TEMPERATURE_PIN 3 // Analog pin to measure the battery temperature

// Define the constants
#define MAX_VOLTAGE 4200 // Maximum charge voltage in millivolts
#define RECHARGE_VOLTAGE 4050 // Voltage to restart charging in millivolts
#define PRECHARGE_THRESHOLD 3000 // Voltage to switch from pre-charge to fast-charge in millivolts
#define PRECHARGE_CURRENT 50 // Current limit for pre-charging in milliamps

// Longevity-optimized Thermal Zones
#define FASTCHARGE_CURRENT 500 // Max current at ideal temp (10-35°C)
#define COLD_TEMP 10 // Below this, reduce current
#define COLD_CURRENT 100 // Max current at cold temp (0-10°C)
#define WARM_TEMP 35 // Above this, reduce current
#define WARM_CURRENT 250 // Max current at warm temp (35-45°C)

#define TERMINATION_CURRENT 50 // Current to terminate charging in CV phase in milliamps
#define MAX_TEMPERATURE 45 // Maximum temperature to charge in degrees Celsius
#define MAX_TEMP_RECOVERY 40 // Temperature to restart charging after over-temp
#define MIN_TEMPERATURE 0 // Minimum temperature to charge in degrees Celsius
#define MIN_TEMP_RECOVERY 5 // Temperature to restart charging after under-temp
#define WDT_TIMEOUT 8 // Watchdog timer timeout in seconds
#define ALPHA 0.2 // Smoothing factor for EMA (0.0 to 1.0)

// Define the variables
int chargeValue = 0; // PWM value to control the charging MOSFET
float smoothedVoltage = 0; // Smoothed battery voltage in millivolts
float smoothedCurrent = 0; // Smoothed charging current in milliamps
float smoothedTemperature = 25; // Smoothed battery temperature in degrees Celsius
bool charging = false; // Charging status flag
bool temp_fault = false; // Temperature fault flag

// Forward declarations
void reset_watchdog();
void stop_charging();

// Initialize the watchdog timer
void setup_watchdog(int timeout) {
  byte wdtcsr = 0;
  if (timeout > 0) {
    wdtcsr = bit(WDCE) | bit(WDE);
    switch (timeout) {
      case 1: wdtcsr |= bit(WDP0); break;
      case 2: wdtcsr |= bit(WDP1); break;
      case 4: wdtcsr |= bit(WDP1) | bit(WDP0); break;
      case 8: wdtcsr |= bit(WDP2); break;
      case 16: wdtcsr |= bit(WDP2) | bit(WDP0); break;
      case 32: wdtcsr |= bit(WDP2) | bit(WDP1); break;
      case 64: wdtcsr |= bit(WDP2) | bit(WDP1) | bit(WDP0); break;
      case 128: wdtcsr |= bit(WDP3); break;
      case 256: wdtcsr |= bit(WDP3) | bit(WDP0); break;
      default: wdtcsr |= bit(WDP3) | bit(WDP1); break;
    }
    reset_watchdog();
    MCUSR &= ~bit(WDRF);
    WDTCR |= bit(WDCE) | bit(WDE);
    WDTCR = wdtcsr;
  }
  else {
    MCUSR &= ~bit(WDRF);
    WDTCR |= bit(WDCE) | bit(WDE);
    WDTCR = 0x00;
  }
}

void reset_watchdog() { wdt_reset(); }

void update_readings() {
  int rawVoltage = map(analogRead(VOLTAGE_PIN), 0, 1023, 0, 5000);
  int rawCurrent = map(analogRead(CURRENT_PIN), 0, 1023, 0, 1000);
  int rawTemperature = map(analogRead(TEMPERATURE_PIN), 0, 1023, -40, 125);

  if (smoothedVoltage == 0) smoothedVoltage = rawVoltage;
  else smoothedVoltage = (ALPHA * (float)rawVoltage) + ((1.0 - ALPHA) * smoothedVoltage);

  smoothedCurrent = (ALPHA * (float)rawCurrent) + ((1.0 - ALPHA) * smoothedCurrent);
  smoothedTemperature = (ALPHA * (float)rawTemperature) + ((1.0 - ALPHA) * smoothedTemperature);
}

void start_charging() {
  if (temp_fault) return;
  charging = true;
  chargeValue = 10;
  analogWrite(CHARGE_PIN, chargeValue);
}

void stop_charging() {
  charging = false;
  chargeValue = 0;
  analogWrite(CHARGE_PIN, chargeValue);
}

void adjust_charging() {
  if (temp_fault) {
      stop_charging();
      return;
  }

  // Termination condition (CV phase complete)
  if (smoothedVoltage >= MAX_VOLTAGE - 10 && smoothedCurrent <= TERMINATION_CURRENT && chargeValue < 100) {
    stop_charging();
    return;
  }

  // Safety over-voltage protection
  if (smoothedVoltage >= MAX_VOLTAGE + 50) {
    stop_charging();
    return;
  }

  int current_target;
  if (smoothedVoltage < PRECHARGE_THRESHOLD) {
    current_target = PRECHARGE_CURRENT;
  } else {
    // Determine target current based on temperature for longevity
    if (smoothedTemperature < COLD_TEMP) {
      current_target = COLD_CURRENT;
    } else if (smoothedTemperature > WARM_TEMP) {
      current_target = WARM_CURRENT;
    } else {
      current_target = FASTCHARGE_CURRENT;
    }
  }

  // CC/CV Regulation Logic
  if (smoothedVoltage >= MAX_VOLTAGE) {
    if (chargeValue > 0) chargeValue--;
  }
  else if (smoothedCurrent > current_target) {
    if (chargeValue > 0) chargeValue--;
  }
  else if (chargeValue < 255) {
    chargeValue++;
  }

  analogWrite(CHARGE_PIN, chargeValue);
}

void check_temperature() {
  if (smoothedTemperature >= MAX_TEMPERATURE || smoothedTemperature <= MIN_TEMPERATURE) {
    temp_fault = true;
    stop_charging();
  } else if (temp_fault) {
    if (smoothedTemperature <= MAX_TEMP_RECOVERY && smoothedTemperature >= MIN_TEMP_RECOVERY) {
      temp_fault = false;
    }
  }
}

void setup() {
  pinMode(CHARGE_PIN, OUTPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);
  pinMode(TEMPERATURE_PIN, INPUT);
  setup_watchdog(WDT_TIMEOUT);
}

void loop() {
  update_readings();
  check_temperature();

  if (smoothedVoltage >= MAX_VOLTAGE + 50) {
      stop_charging();
  } else if (!charging && smoothedVoltage <= RECHARGE_VOLTAGE && !temp_fault) {
      start_charging();
  } else if (charging) {
      adjust_charging();
  }

  reset_watchdog();
  delay(10);
}
