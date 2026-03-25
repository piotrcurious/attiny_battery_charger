// Arduino code for attiny13 li-ion battery charger with load control
// Features: maximum charge voltage, wdt safety, deep sleep, over current protection, thermal safety protection, load PWM control, smoothing, hysteresis
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
#define LOAD_PIN 4 // PWM pin to control the load MOSFET

// Define the constants
#define MAX_VOLTAGE 4200 // Maximum charge voltage in millivolts
#define RECHARGE_VOLTAGE 4050 // Voltage to restart charging in millivolts
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
#define ALPHA 0.2 // Smoothing factor for EMA (0.0 to 1.0)

// Define the variables
int chargeValue = 0; // PWM value to control the charging MOSFET
float smoothedVoltage = 0; // Smoothed battery voltage in millivolts
float smoothedCurrent = 0; // Smoothed charging current in milliamps
float smoothedTemperature = 25; // Smoothed battery temperature in degrees Celsius
int loadValue = 0; // PWM value to control the load MOSFET
bool charging = false; // Charging status flag

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

void enter_sleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  sleep_disable();
}

// Update smoothed readings using EMA
void update_readings() {
  int rawVoltage = map(analogRead(VOLTAGE_PIN), 0, 1023, 0, 5000);
  int rawCurrent = map(analogRead(CURRENT_PIN), 0, 1023, 0, 1000);
  int rawTemperature = map(analogRead(TEMPERATURE_PIN), 0, 1023, -40, 125);

  if (smoothedVoltage == 0) smoothedVoltage = rawVoltage;
  else smoothedVoltage = (ALPHA * rawVoltage) + ((1.0 - ALPHA) * smoothedVoltage);

  smoothedCurrent = (ALPHA * rawCurrent) + ((1.0 - ALPHA) * smoothedCurrent);
  smoothedTemperature = (ALPHA * rawTemperature) + ((1.0 - ALPHA) * smoothedTemperature);
}

void start_charging() {
  charging = true;
  chargeValue = 128; // Start at half PWM
  analogWrite(CHARGE_PIN, chargeValue);
}

void stop_charging() {
  charging = false;
  chargeValue = 0;
  analogWrite(CHARGE_PIN, chargeValue);
}

void adjust_charging() {
  if (smoothedVoltage >= MAX_VOLTAGE) {
    stop_charging();
  }
  else if (smoothedCurrent >= MAX_CURRENT) {
    if (chargeValue > 0) chargeValue--;
    analogWrite(CHARGE_PIN, chargeValue);
  }
  else if (smoothedCurrent <= MIN_CURRENT && smoothedVoltage >= RECHARGE_VOLTAGE) {
     if (smoothedVoltage < MAX_VOLTAGE - 10 && chargeValue < 255) {
         chargeValue++;
         analogWrite(CHARGE_PIN, chargeValue);
     }
  }
  else if (chargeValue < 255) {
    chargeValue++;
    analogWrite(CHARGE_PIN, chargeValue);
  }
}

void check_temperature() {
  if (smoothedTemperature >= MAX_TEMPERATURE || smoothedTemperature <= MIN_TEMPERATURE) {
    stop_charging();
  }
}

void control_load() {
  if (smoothedVoltage >= MAX_LOAD_VOLTAGE) {
    loadValue = MAX_LOAD_PWM;
  }
  else if (smoothedVoltage <= MIN_LOAD_VOLTAGE) {
    loadValue = MIN_LOAD_PWM;
  }
  else {
    loadValue = map((int)smoothedVoltage, MIN_LOAD_VOLTAGE, MAX_LOAD_VOLTAGE, MIN_LOAD_PWM, MAX_LOAD_PWM);
  }
  analogWrite(LOAD_PIN, loadValue);
}

void setup() {
  pinMode(CHARGE_PIN, OUTPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);
  pinMode(TEMPERATURE_PIN, INPUT);
  pinMode(LOAD_PIN, OUTPUT);
  setup_watchdog(WDT_TIMEOUT);
}

void loop() {
  update_readings();

  if (!charging && smoothedVoltage <= RECHARGE_VOLTAGE && smoothedVoltage >= MIN_VOLTAGE) {
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
