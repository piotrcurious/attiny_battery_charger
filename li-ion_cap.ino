// Arduino code for attiny13 li-ion battery charger with load control and thermistor temperature measurement
// Features: CC/CV, Pre-charging, Cap-based NTC, Load PWM, Smoothing, Temperature Hysteresis
// Disclaimer: This code is for educational purposes only and not intended to be used in real applications.

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
#define RECHARGE_VOLTAGE 4050 // Voltage to restart charging in millivolts
#define PRECHARGE_THRESHOLD 3000 // Voltage to switch from pre-charge to fast-charge in millivolts
#define PRECHARGE_CURRENT 50 // Current limit for pre-charging in milliamps
#define FASTCHARGE_CURRENT 500 // Current limit for fast-charging in milliamps
#define TERMINATION_CURRENT 50 // Current to terminate charging in CV phase in milliamps
#define MAX_TEMPERATURE 45 // Maximum temperature to charge in degrees Celsius
#define MAX_TEMP_RECOVERY 40 // Temperature to restart charging after over-temp
#define MIN_TEMPERATURE 0 // Minimum temperature to charge in degrees Celsius
#define MIN_TEMP_RECOVERY 5 // Temperature to restart charging after under-temp
#define MAX_LOAD_VOLTAGE 4000 // Maximum battery voltage to allow maximum load PWM in millivolts
#define MIN_LOAD_VOLTAGE 3200 // Minimum battery voltage to allow minimum load PWM in millivolts
#define MAX_LOAD_PWM 255 // Maximum load PWM value
#define MIN_LOAD_PWM 0 // Minimum load PWM value
#define REFERENCE_RESISTOR 10000 // Reference resistor value in ohms
#define THERMISTOR_NOMINAL_RESISTOR 10000 // Thermistor resistor value at 25 degrees Celsius in ohms
#define THERMISTOR_BETA 3950 // Thermistor beta coefficient in Kelvin
#define WDT_TIMEOUT 8 // Watchdog timer timeout in seconds
#define ALPHA 0.2 // Smoothing factor for EMA (0.0 to 1.0)

// Define the variables
int chargeValue = 0; // PWM value to control the charging MOSFET
float smoothedVoltage = 0; // Smoothed battery voltage in millivolts
float smoothedCurrent = 0; // Smoothed charging current in milliamps
float smoothedTemperature = 25; // Smoothed battery temperature in degrees Celsius
int loadValue = 0; // PWM value to control the load MOSFET
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

unsigned long measure_discharge_time(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(10);
  pinMode(pin, INPUT);
  unsigned long startTime = micros();
  while (digitalRead(pin) == HIGH) {
      if (micros() - startTime > 100000) break;
  }
  unsigned long endTime = micros();
  return endTime - startTime;
}

void update_readings() {
  int rawVoltage = map(analogRead(VOLTAGE_PIN), 0, 1023, 0, 5000);
  int rawCurrent = map(analogRead(CURRENT_PIN), 0, 1023, 0, 1000);

  if (smoothedVoltage == 0) smoothedVoltage = rawVoltage;
  else smoothedVoltage = (ALPHA * (float)rawVoltage) + ((1.0 - ALPHA) * smoothedVoltage);

  smoothedCurrent = (ALPHA * (float)rawCurrent) + ((1.0 - ALPHA) * smoothedCurrent);

  unsigned long t_ref = measure_discharge_time(REFERENCE_PIN);
  unsigned long t_ntc = measure_discharge_time(TEMPERATURE_PIN);

  if (t_ref > 0 && t_ntc > 0) {
      float thermistorResistance = (float)REFERENCE_RESISTOR * (float)t_ntc / (float)t_ref;
      float steinhart;
      steinhart = thermistorResistance / THERMISTOR_NOMINAL_RESISTOR;
      steinhart = log(steinhart);
      steinhart /= THERMISTOR_BETA;
      steinhart += 1.0 / (25.0 + 273.15);
      steinhart = 1.0 / steinhart;
      steinhart -= 273.15;

      smoothedTemperature = (ALPHA * steinhart) + ((1.0 - ALPHA) * smoothedTemperature);
  }
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
    current_target = FASTCHARGE_CURRENT;
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
  pinMode(REFERENCE_PIN, INPUT);
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

  control_load();
  reset_watchdog();
  delay(10);
}
