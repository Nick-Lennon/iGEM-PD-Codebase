#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*
 * MAIN QUESTIONS FOR PD PEOPLE:
 * 1) what pin does the perchlorate sensor connect to?
 * 2) is my math right for calculating logarithmic concentrations of perchlorate
 * 3) is 0.01M a good reference concentration for perchlorate, or do you want it to be different?
 */

///////////////////////////////////////////////////////////
//  STATIC VARIABLES (immutable)
///////////////////////////////////////////////////////////
#define ISE_PIN        A1        // perchlorate ISE analog pin
#define ONE_WIRE_BUS   2         // DS18B20 data pin
#define REF_VOLTAGE    5000.0f   // ADC reference (mV)
#define NUM_SAMPLES    10        // samples for averaging


///////////////////////////////////////////////////////////
//  VARIABLES (mutable)
///////////////////////////////////////////////////////////

/*
 * EEPROM stands for Electrically Erasable Programmable Read-Only Memory. This is how we send our instructions to the
 * sensor so it knows how to interpret the data it recieves. ADDR_V01M_MV and ADDR_V0001M_MV are memory addresses
 * (places in memory) where we store our calibration data for voltage at a perchlorate concentration of 0.01 and 0.0001 respectively.
 */
const int ADDR_V01M_MV   = 0;                              // voltage at 0.01 M
const int ADDR_V0001M_MV = ADDR_V01M_MV + sizeof(float);   // voltage at 0.0001 M
const int ADDR_SLOPE25   = ADDR_V0001M_MV + sizeof(float); // slope at 25 °C

/*
 * These are the mutable variables that we load our existing calibration data into and do math with 
 * (prevent overwriting of calibration data)
 */
float v01_mV   = 0.0f;   // measured mV at 0.01 M
float v0001_mV = 0.0f;   // measured mV at 0.0001 M
float slope25  = 0.0f;   // slope at 25 °C (decade/mV)

/*
 * oneWire is the object that knows how to talk to any OneWire devices connected to pin 2.
 * Sensors is a higher-level object representing the temperature sensor(s), connected to the oneWire bus.
 * Like saying, "Here’s my OneWire bus object. Please manage all DS18B20 sensors that are connected to it." 
 * (can call all or reference specific sensor ID)
  */
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


///////////////////////////////////////////////////////////
//  MAIN SETUP
///////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);     // USB console
  Serial1.begin(9600);      // Bluetooth module
  sensors.begin();

  loadCalibration();

  Serial.println(F("Perchlorate ISE system ready"));
  Serial1.println(F("Perchlorate ISE system ready"));
}


///////////////////////////////////////////////////////////
//  MAIN LOOP
///////////////////////////////////////////////////////////

/*
 * our loop is 5 steps and occurs once every second (1000 ms)
 * 
 * 1) read perchlorate sensor voltage
 * 2) read current temperature
 * 3) compute temperature-adjusted slope
 * 4/5) compute concentration of perchlorate relative to a 0.01M reference
 * 5) report to both USB and Bluetooth
 * 
 * These values are placed into an array of maximum 64 characters called "buf"
 * that holds the formatted string in the form "xxx mV, xx.x °C, pH: x.xx".
 * That array of data is then sent to both the USB console and the bluetooth module
 */
void loop() {
  processSerialCommands();

  // 1) read electrode voltage
  float ise_mV = readAverageVoltage(ISE_PIN);

  // 2) read current temperature
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  // 3) temperature-adjust slope
  float slopeT = slope25 * ((tempC + 273.15f) / 298.15f);

  // 4) compute log concentration relative to 0.01 M reference
  //     Ref concentration = 0.01 M → logC = -2
  float logC_val = -2.0f + slopeT * (ise_mV - v01_mV);

  // 5) convert log concentration → molarity
  float conc = pow(10, logC_val);

  // 6) report
  char buf[64];
  snprintf(buf, sizeof(buf),
    "%.1f mV, %.1f °C, [ClO4-]=%.5f M",
    ise_mV, tempC, conc
  );

  Serial.println(buf);
  Serial1.println(buf);

  delay(1000);
}


///////////////////////////////////////////////////////////
//  HELPER METHODS
///////////////////////////////////////////////////////////

/*
 * Input: Arduino analog input pin that we are reading from
 * Output: Float representing averaged voltage in mV
 * Goal: Read+average raw ADC and convert to mV
 */
float readAverageVoltage(int pin) {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(pin);
    delay(5);
  }
  float avgRaw = sum / (float)NUM_SAMPLES;
  return avgRaw * (REF_VOLTAGE / 1024.0f);
}

/*
 * This method is responsible for loading calibration data (voltages at concentration of 0.01M, 0.0001M, and slope at 25 °C) 
 * from the Arduino’s EEPROM memory when the system starts up
 * if slope25 = 0.0, a warning is sent because that means no calibration data has been saved yet
 */
void loadCalibration() {
  EEPROM.get(ADDR_V01M_MV, v01_mV);
  EEPROM.get(ADDR_V0001M_MV, v0001_mV);
  EEPROM.get(ADDR_SLOPE25, slope25);

  if (slope25 != 0.0f) {
    char buf[80];
    snprintf(buf, sizeof(buf),
      "Cal loaded: V0.01M=%.1f mV, V0.0001M=%.1f mV, slope25=%.6f",
      v01_mV, v0001_mV, slope25
    );
    Serial.println(buf);
    Serial1.println(buf);
  } else {
    Serial.println("No calibration found. Use 'cal01', 'cal0001', then 'calise'.");
    Serial1.println("No calibration found. Use 'cal01', 'cal0001', then 'calise'.");
  }
}

/*
 * Handle calibration commands
 */
void processSerialCommands() {
  auto tryRead = [&](Stream &stream) -> String {
    if (!stream.available()) return String();
    String s = stream.readStringUntil('\n');
    s.trim();
    return s;
  };

  String cmd = tryRead(Serial);
  if (cmd.length() == 0) cmd = tryRead(Serial1);

  if (cmd.length() > 0) {
    Serial.print(F("CMD: "));  Serial.println(cmd);
    Serial1.print(F("CMD: ")); Serial1.println(cmd);

    if (cmd == "cal01") {
      v01_mV = readAverageVoltage(ISE_PIN);
      EEPROM.put(ADDR_V01M_MV, v01_mV);
      Serial.print("Stored 0.01M voltage: ");  Serial.println(v01_mV);
      Serial1.print("Stored 0.01M voltage: "); Serial1.println(v01_mV);
    }
    else if (cmd == "cal0001") {
      v0001_mV = readAverageVoltage(ISE_PIN);
      EEPROM.put(ADDR_V0001M_MV, v0001_mV);
      Serial.print("Stored 0.0001M voltage: ");  Serial.println(v0001_mV);
      Serial1.print("Stored 0.0001M voltage: "); Serial1.println(v0001_mV);
    }
    else if (cmd == "calise") {
      if (v01_mV > 0 && v0001_mV > 0 && v01_mV != v0001_mV) {
        sensors.requestTemperatures();
        float tempC_cal = sensors.getTempCByIndex(0);

        // ΔlogC = -2 – (-4) = 2
        float slope_cal = ( -2.0f - (-4.0f) ) / (v01_mV - v0001_mV);
        slope25 = slope_cal * (298.15f / (tempC_cal + 273.15f));
        EEPROM.put(ADDR_SLOPE25, slope25);

        Serial.print("Saved slope25 = ");  Serial.println(slope25, 6);
        Serial1.print("Saved slope25 = "); Serial1.println(slope25, 6);
      } else {
        Serial.println("Error: first set both 0.01M & 0.0001M with 'cal01' and 'cal0001'.");
        Serial1.println("Error: first set both 0.01M & 0.0001M with 'cal01' and 'cal0001'.");
      }
    }
    else {
      Serial.println("Unknown cmd.");
      Serial1.println("Unknown cmd.");
    }
  }
}
