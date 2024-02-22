//Dreamed by Gemini 

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

  // Set unused pins as input
  pinMode(REFERENCEPIN, INPUT);
  pinMode(NTCPIN, INPUT);
}

void loop() {
  readTemperature();

  // Set unused pins as input again after measurement
  pinMode(REFERENCEPIN, INPUT);
  pinMode(NTCPIN, INPUT);

  Serial.print("Temperature: ");
  Serial.print(temperatureCelsius);
  Serial.println(" C");
  delay(1000);
}

void readTemperature() {
  // Set reference and NTC pins as output for charging
  pinMode(REFERENCEPIN, OUTPUT);
  pinMode(NTCPIN, OUTPUT);

  // Charge capacitor with NTC sensor
  digitalWrite(NTCPIN, HIGH);
  delay(10);
  digitalWrite(NTCPIN, LOW);

  // Set NTC pin as input for measuring discharge
  pinMode(NTCPIN, INPUT);

  // Measure discharge time with pulse counting
  pulseCount = 0;
  while (analogRead(THERMISTORPIN) > 10) {
    pulseCount++;
    // Set NTC pin as output for pulse generation
    pinMode(NTCPIN, OUTPUT);
    delayMicroseconds(1);
    digitalWrite(NTCPIN, LOW);
    // Set NTC pin as input again after pulse
    pinMode(NTCPIN, INPUT);
  }

  // Calculate thermistor resistance
  thermistorResistance = REFERENCE_RESISTOR * (float)pulseCount * supplyVoltage / (CAP_VALUE * supplyVoltage - REFERENCE_RESISTOR * pulseCount);

  // Calculate temperature using Steinhart-Hart equation
  temperatureCelsius = 1 / (NTC_BETA * log(thermistorResistance / NTC_NOMINAL_RESISTANCE) + (1 / 298.15)) - 273.15;
}

void calibrateSupplyVoltage() {
  // Set reference pin as output for charging
  pinMode(REFERENCEPIN, OUTPUT);

  pulseCount = 0;
  while (analogRead(REFERENCEPIN) > 10) {
    pulseCount++;
    // Set reference pin as output for pulse generation
    pinMode(REFERENCEPIN, OUTPUT);
    delayMicroseconds(1);
    digitalWrite(REFERENCEPIN, LOW);
    // Set reference pin as input again after pulse
    pinMode(REFERENCEPIN, INPUT);
  }
  supplyVoltage = REFERENCE_RESISTOR * (float)pulseCount / CAP_VALUE;

  // Set reference pin as input again
  pinMode(REFERENCEPIN, INPUT);
}
