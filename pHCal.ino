#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

///////////////////////////////////////////////////////////
//  STATIC VARIABLES (immutable)
///////////////////////////////////////////////////////////
#define PH_PIN         A1        // pH sensor analog pin
#define ONE_WIRE_BUS   2         // DS18B20 data pin
#define REF_VOLTAGE    5000.0f   // ADC reference (mV)
#define NUM_SAMPLES    10        // samples for averaging


///////////////////////////////////////////////////////////
//  VARIABLES (mutable)
///////////////////////////////////////////////////////////

/*
 * EEPROM stands for Electrically Erasable Programmable Read-Only Memory. This is how we send our 
 * instructions to the sensor so it knows how to interpret the data it recieves. ADDR_V7_MV and ADDR_V4_MV are memory 
 * addresses (places in memory) where we store our calibration data for voltage at a pH of 7 and 4 respectively.
 */
const int ADDR_V7_MV   = 0;
const int ADDR_V4_MV   = ADDR_V7_MV + sizeof(float);
const int ADDR_SLOPE25 = ADDR_V4_MV + sizeof(float);

// Calibration vars
/*
 * These are the mutable variables that we load our calibration data into and do math with
 */
float v7_mV    = 0.0f;   // measured mV at pH 7.0
float v4_mV    = 0.0f;   // measured mV at pH 4.0
float slope25  = 0.0f;   // true Nernst slope at 25 °C (pH/mV)

/*
 * oneWire is the object that knows how to talk to any OneWire devices connected to pin 2.
 * Sensors is a higher-level object representing the temperature sensor(s), connected to the oneWire bus.
 * Like saying, "Here’s my OneWire bus object. Please manage all DS18B20 sensors that are connected to it." 
 * (can call all or reference specific sensor ID)
  */
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


///////////////////////////////////////////////////////////
//  MAIN LOOP BODY
///////////////////////////////////////////////////////////

/*
 * Setup initializes variables using our stored calibration data
 * Serial is our USB console
 * Serial1 is our Bluetooth module
 */
void setup() {
  // USB console
  Serial.begin(115200);
  // hardware UART → Bluetooth module
  Serial1.begin(9600);

  sensors.begin();
  loadCalibration();

  Serial.println(F("pH/temp system ready"));
  Serial1.println(F("pH/temp system ready"));
}


/*
 * our loop is 5 steps and occurs once every second (1000 ms)
 * 
 * 1) read pH sensor voltage
 * 2) read current temperature
 * 3) compute temperature-adjusted slope
 * 4) compute pH anchored at pH=7 voltage
 * 5) report to both USB and Bluetooth
 * 
 * These values are placed into an array of maximum 64 characters called "buf"
 * that holds the formatted string in the form "xxx mV, xx.x °C, pH: x.xx".
 * That array of data is then sent to both the USB console and the bluetooth module
 */
void loop() {
  processSerialCommands();

  // 1) read pH sensor voltage
  float ph_mV = readAverageVoltage(PH_PIN);

  // 2) read current temperature
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  // 3) compute temperature-adjusted slope
  float slopeT = slope25 * ((tempC + 273.15f) / 298.15f);

  // 4) compute pH anchored at pH=7 voltage
  float pH_val = 7.0f + slopeT * (ph_mV - v7_mV);

  // 5) report to both USB and Bluetooth
  char buf[64];
  int n = snprintf(buf, sizeof(buf),
    "%.1f mV, %.1f °C, pH: %.2f",
    ph_mV, tempC, pH_val
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
    delay(5); //5 ms spacing inbetween readings to avoid noise disruptions
  }
  float avgRaw = sum / (float)NUM_SAMPLES; //NUM_SAMPLES cast to float to avoid int division (loses data)
  return avgRaw * (REF_VOLTAGE / 1024.0f); //converts ADC to mV
}


/*
 * This method is responsible for loading calibration data (voltages at pH 7, pH 4, and slope at 25 °C) 
 * from the Arduino’s EEPROM memory when the system starts up
 * if slope25 = 0.0, a warning is sent because that means no calibration data has been saved yet
 */
void loadCalibration() {
  EEPROM.get(ADDR_V7_MV, v7_mV);
  EEPROM.get(ADDR_V4_MV, v4_mV);
  EEPROM.get(ADDR_SLOPE25, slope25);

  if (slope25 != 0.0f) {
    char buf[80];
    int n = snprintf(buf, sizeof(buf),
      "Cal loaded: V7=%.1f mV, V4=%.1f mV, slope25=%.6f",
      v7_mV, v4_mV, slope25
    );
    Serial.println(buf);
    Serial1.println(buf);
  } else {
    Serial.println("No calibration found. Use 'cal7', 'cal4', then 'calph'.");
    Serial1.println("No calibration found. Use 'cal7', 'cal4', then 'calph'.");
  }
}


/*
 * Handles serial commands (sequential bits of data) being sent over BOTH USB ("Serial") and Bluetooth ("Serial1")
 * 
 */
void processSerialCommands() {
  // Helper lambda to read a line from a Stream
  auto tryRead = [&](Stream &stream) -> String {
    if (!stream.available()) return String(); //If there's nothing to read, return an empty string
    String s = stream.readStringUntil('\n'); //reads characters until newline or timeout
    s.trim(); //remove whitespace
    return s;
  };
  
  // Check USB, if nothing try Bluetooth next
  String cmd = tryRead(Serial);
  // If nothing on USB, check BT
  if (cmd.length() == 0) {
    cmd = tryRead(Serial1);
  }

  if (cmd.length() > 0) { // if cmd.length > 0, a USB/Bluetooth command was recieved
    // Echo back where it came from
    Serial.print(F("CMD: "));  Serial.println(cmd);
    Serial1.print(F("CMD: ")); Serial1.println(cmd);

    // Each of these conditionals will rewrite either the 7.0 pH voltage, 4.0 pH coltage,
    // or recalibrate the slope of the pH via the Nernst equation

    if (cmd == "cal7") {
      v7_mV = readAverageVoltage(PH_PIN);
      EEPROM.put(ADDR_V7_MV, v7_mV);
      Serial.print("Stored pH7 voltage: ");  Serial.print(v7_mV); Serial.println(" mV");
      Serial1.print("Stored pH7 voltage: "); Serial1.print(v7_mV); Serial1.println(" mV");
    }
    else if (cmd == "cal4") {
      v4_mV = readAverageVoltage(PH_PIN);
      EEPROM.put(ADDR_V4_MV, v4_mV);
      Serial.print("Stored pH4 voltage: ");  Serial.print(v4_mV); Serial.println(" mV");
      Serial1.print("Stored pH4 voltage: "); Serial1.print(v4_mV); Serial1.println(" mV");
    }
    else if (cmd == "calph") {
      if (v7_mV > 0 && v4_mV > 0 && v7_mV != v4_mV) {
        sensors.requestTemperatures();
        float tempC_cal = sensors.getTempCByIndex(0);
        float slope_cal = (7.0f - 4.0f) / (v7_mV - v4_mV);
        slope25 = slope_cal * (298.15f / (tempC_cal + 273.15f));
        EEPROM.put(ADDR_SLOPE25, slope25);

        Serial.print("Saved slope25 = ");  Serial.println(slope25, 6);
        Serial1.print("Saved slope25 = "); Serial1.println(slope25, 6);
      } else {
        Serial.println("Error: first set both pH7 & pH4 with 'cal7' and 'cal4'.");
        Serial1.println("Error: first set both pH7 & pH4 with 'cal7' and 'cal4'.");
      }
    }
    else {
      Serial.println("Unknown cmd.");
      Serial1.println("Unknown cmd.");
    }
  }
}
