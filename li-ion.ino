// Arduino code for attiny13 li-ion battery charger
// Features: CC/CV, Pre-charging, Hysteresis, Smoothing, WDT Safety, Thermal Control, Sleep/Power Management
// Production Quality: Fixed-point math, Integer EMA with precision, Robust Sensor Sanity Checks

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

// ATtiny13 Pin Mapping
#define CHARGE_PIN 0 // PB0 (OC0A)
#define VOLTAGE_PIN 1 // ADC1 (PB2)
#define CURRENT_PIN 2 // ADC2 (PB4)
#define TEMPERATURE_PIN 3 // ADC3 (PB3)

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
#define EMA_SHIFT 3

#define SENSOR_MIN_RAW 10
#define SENSOR_MAX_RAW 1010

int chargeValue = 0;
int32_t smoothedVoltageQ8 = 0;
int32_t smoothedCurrentQ8 = 0;
int32_t smoothedTemperatureQ8 = 25 << 8;
bool charging = false;
bool temp_fault = false;
bool sensor_fault = false;
bool first_run = true;

// Convenience macros for accessing smoothed values
#define smoothedVoltage (smoothedVoltageQ8 >> 8)
#define smoothedCurrent (smoothedCurrentQ8 >> 8)
#define smoothedTemperature (smoothedTemperatureQ8 >> 8)

// Forward declarations
void reset_watchdog();
void stop_charging();
void start_charging();
void adjust_charging();
void check_temperature();
void update_readings();
void enter_sleep(int cycles);

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
  int rawV = analogRead(VOLTAGE_PIN);
  int rawI = analogRead(CURRENT_PIN);
  int rawT = analogRead(TEMPERATURE_PIN);

  if (rawV < SENSOR_MIN_RAW || rawV > SENSOR_MAX_RAW ||
      rawT < SENSOR_MIN_RAW || rawT > SENSOR_MAX_RAW) {
      sensor_fault = true;
      stop_charging();
  } else {
      sensor_fault = false;
  }

  int32_t rawVoltage = ((int32_t)rawV * 5000) >> 10;
  int32_t rawCurrent = ((int32_t)rawI * 1000) >> 10;
  int16_t rawTemperature = (int16_t)((((int32_t)rawT * 165) >> 10) - 40);

  if (first_run) {
      smoothedVoltageQ8 = rawVoltage << 8;
      smoothedCurrentQ8 = rawCurrent << 8;
      smoothedTemperatureQ8 = (int32_t)rawTemperature << 8;
      first_run = false;
  } else {
      smoothedVoltageQ8 = smoothedVoltageQ8 + (((rawVoltage << 8) - smoothedVoltageQ8) >> EMA_SHIFT);
      smoothedCurrentQ8 = smoothedCurrentQ8 + (((rawCurrent << 8) - smoothedCurrentQ8) >> EMA_SHIFT);
      smoothedTemperatureQ8 = smoothedTemperatureQ8 + ((((int32_t)rawTemperature << 8) - smoothedTemperatureQ8) >> EMA_SHIFT);
  }
}

void start_charging() {
  if (temp_fault || sensor_fault) return;
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
  if (temp_fault || sensor_fault) {
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
  setup_watchdog(WDT_TIMEOUT);
  sei();
}

void loop() {
  update_readings();
  check_temperature();

  if (smoothedVoltage >= MAX_VOLTAGE + 50 || sensor_fault) {
      stop_charging();
  } else if (!charging && smoothedVoltage <= RECHARGE_VOLTAGE && !temp_fault && !sensor_fault && smoothedVoltage >= MIN_CHARGE_VOLTAGE) {
      start_charging();
  } else if (charging) {
      adjust_charging();
  }

  reset_watchdog();

  if (charging) {
      delay(10);
  } else {
      if (smoothedVoltage < ECO_MODE_THRESHOLD) enter_sleep(8);
      else enter_sleep(1);
  }
}

EMPTY_INTERRUPT(WDT_vect);
