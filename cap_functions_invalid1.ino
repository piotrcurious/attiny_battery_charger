// Dreamed by Gemini. 

#define THERMISTORPIN A0
#define REFERENCEPIN 5
#define NTCPIN 6
#define CAPACITOR 1000  // Capacitor value in nanofarads
#define REFERENCE_RESISTOR 10000  // Reference resistor value in ohms
#define NTC_NOMINAL_RESISTANCE 10000  // NTC nominal resistance at 25°C
#define NTC_BETA 3950  // NTC beta coefficient

int pulseCount = 0;
float supplyVoltage = 0.0;
float thermistorResistance = 0.0;
float temperatureCelsius = 0.0;

void setup() {
  Serial.begin(9600);
  pinMode(REFERENCEPIN, OUTPUT);
  pinMode(NTCPIN, OUTPUT);
  pinMode(THERMISTORPIN, INPUT);

  // Initial calibration cycle
  digitalWrite(REFERENCEPIN, HIGH);
  delay(10);  // Charge capacitor
  digitalWrite(REFERENCEPIN, LOW);
  calibrateSupplyVoltage();
}

void loop() {
  readTemperature();
  Serial.print("Temperature: ");
  Serial.print(temperatureCelsius);
  Serial.println(" C");
  delay(1000);
}

void readTemperature() {
  // Charge capacitor with NTC sensor
  digitalWrite(NTCPIN, HIGH);
  delay(10);
  digitalWrite(NTCPIN, LOW);

  // Measure discharge time
  pulseCount = 0;
  while (analogRead(THERMISTORPIN) > 10) {
    pulseCount++;
    digitalWrite(NTCPIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(NTCPIN, LOW);
  }

  // Calculate thermistor resistance
  thermistorResistance = REFERENCE_RESISTOR * (float)pulseCount * supplyVoltage / (CAP_VALUE * supplyVoltage - REFERENCE_RESISTOR * pulseCount);

  // Calculate temperature using Steinhart-Hart equation
  temperatureCelsius = 1 / (NTC_BETA * log(thermistorResistance / NTC_NOMINAL_RESISTANCE) + (1 / 298.15)) - 273.15;
}

void calibrateSupplyVoltage() {
  pulseCount = 0;
  while (analogRead(REFERENCEPIN) > 10) {
    pulseCount++;
    digitalWrite(REFERENCEPIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(REFERENCEPIN, LOW);
  }
  supplyVoltage = REFERENCE_RESISTOR * (float)pulseCount / CAP_VALUE;
}
