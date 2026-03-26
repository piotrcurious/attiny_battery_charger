// Arduino code for attiny13 li-ion battery charger
// Features: CC/CV, Pre-charging, Hysteresis, Smoothing, WDT Safety, Thermal Control, Sleep/Power Management
// Disclaimer: This code is for educational purposes only and not intended to be used in real applications.

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

// Define the pins
#define CHARGE_PIN 0
#define VOLTAGE_PIN 1
#define CURRENT_PIN 2
#define TEMPERATURE_PIN 3

// Define the constants
#define MAX_VOLTAGE 4200
#define RECHARGE_VOLTAGE 4050
#define PRECHARGE_THRESHOLD 3000
#define ECO_MODE_THRESHOLD 2500
#define PRECHARGE_CURRENT 50
#define FASTCHARGE_CURRENT 500
#define COLD_TEMP 10
#define COLD_CURRENT 100
#define WARM_TEMP 35
#define WARM_CURRENT 250
#define TERMINATION_CURRENT 50
#define MAX_TEMPERATURE 45
#define MAX_TEMP_RECOVERY 40
#define MIN_TEMPERATURE 0
#define MIN_TEMP_RECOVERY 5
#define MIN_CHARGE_VOLTAGE 2000
#define WDT_TIMEOUT 8
#define ALPHA 0.2

// Define the variables
int chargeValue = 0;
float smoothedVoltage = 0;
float smoothedCurrent = 0;
float smoothedTemperature = 25;
bool charging = false;
bool temp_fault = false;

// Forward declarations
void reset_watchdog();
void stop_charging();

// Initialize the watchdog timer for interrupt mode
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
    WDTCR = wdtcsr | bit(WDTIE);
  }
}

void reset_watchdog() { wdt_reset(); }

void enter_sleep(int cycles) {
  ADCSRA &= ~bit(ADEN);
  for (int i=0; i<cycles; i++) {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
    sleep_disable();
    reset_watchdog();
  }
  ADCSRA |= bit(ADEN);
}

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
  if (smoothedVoltage < MIN_CHARGE_VOLTAGE) return;
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
  if (smoothedVoltage >= MAX_VOLTAGE - 10 && smoothedCurrent <= TERMINATION_CURRENT && chargeValue < 100) {
    stop_charging();
    return;
  }
  if (smoothedVoltage >= MAX_VOLTAGE + 50) {
    stop_charging();
    return;
  }

  int current_target;
  if (smoothedVoltage < PRECHARGE_THRESHOLD) {
    current_target = PRECHARGE_CURRENT;
  } else {
    if (smoothedTemperature < COLD_TEMP) current_target = COLD_CURRENT;
    else if (smoothedTemperature > WARM_TEMP) current_target = WARM_CURRENT;
    else current_target = FASTCHARGE_CURRENT;
  }

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
  sei();
}

void loop() {
  update_readings();
  check_temperature();

  // Update charging state
  if (smoothedVoltage >= MAX_VOLTAGE + 50) {
      stop_charging();
  } else if (!charging && smoothedVoltage <= RECHARGE_VOLTAGE && !temp_fault && smoothedVoltage >= MIN_CHARGE_VOLTAGE) {
      start_charging();
  } else if (charging) {
      adjust_charging();
  }

  reset_watchdog();

  // Power management
  if (charging) {
      delay(10);
  } else {
      if (smoothedVoltage < ECO_MODE_THRESHOLD) {
          enter_sleep(8);
      } else {
          enter_sleep(1);
      }
  }
}

EMPTY_INTERRUPT(WDT_vect);
