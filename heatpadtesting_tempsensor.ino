
// -----------------------------------------------------------------------------
// Single-Serial refactor: USB only
// - All Serial1/Serial2 uses routed to Serial (USB CDC).
// - BLE UART removed. Commands and telemetry go over the Serial Monitor.
// - DS18B20 support integrated (DallasTemperature & OneWire).
// -----------------------------------------------------------------------------

#include <Adafruit_NeoPixel.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#ifndef ONE_WIRE_BUS
#define ONE_WIRE_BUS 2
#endif
// --- Temperature sensor bus & driver (single DS18B20 on D2) ---
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// --- Temperature publishing cadence ---
static unsigned long TEMP_LAST_MS = 0;
const unsigned long TEMP_PERIOD_MS = 1000;  // 1 Hz


// ---------------- LED / BLE Pins ----------------
#define LED_PIN        19
#define LED_COUNT      19
#define HEAT_PAD_PIN   44    // Heat pad pin
#define BLE_STATE_PIN  18

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Heat pad power control
int heatLevel = 0;     // Start at 0%
int heatPWM   = 0;

// Breathing control for status LED
int brightness = 0;
int fadeAmount = 6;
unsigned long lastBreathUpdate = 0;
unsigned long pauseStartTime = 0;
const int pauseDuration = 1000; // pause at min/max brightness (ms)

enum BreathState { BRIGHTENING, DARKENING, PAUSED };
BreathState breathState = BRIGHTENING;

String rxBuf = "";
bool lastBleConnected = false;

// ---------------- Motor Pins ----------------
#define PUMP1_PWM 2
#define PUMP1_LOW 3
#define PUMP2_PWM 5
#define PUMP2_LOW 4
#define STIR_PWM  6
#define STIR_LOW  7

// ---------------- Motor Ramp Parameters ----------------
const int PUMP_TARGET_PWM  = 180;
const int STIR_TARGET_PWM  = 200;
const int BOTH_PUMPS_PWM   = 150;
const int STEP      = 5;
const int STEP_DT   = 15;

int pump1CurrentPWM = 0;
int pump2CurrentPWM = 0;
int stirCurrentPWM  = 0;

// ---------------- Setup ----------------
void setup() {
  sensors.begin();  // init DS18B20
  Serial.begin(9600);  // USB serial only

  strip.begin();
  strip.setBrightness(100);
  strip.show();
Serial.begin(9600);

  pinMode(HEAT_PAD_PIN, OUTPUT);
  setHeatPadPercent(0);   // Start heat pad OFF

  pinMode(BLE_STATE_PIN, INPUT);

  pinMode(PUMP1_PWM, OUTPUT);
  pinMode(PUMP1_LOW, OUTPUT);
  pinMode(PUMP2_PWM, OUTPUT);
  pinMode(PUMP2_LOW, OUTPUT);
  pinMode(STIR_PWM,  OUTPUT);
  pinMode(STIR_LOW,  OUTPUT);

  stopPumps();
  runStartupAnimation();
}

// ---------------- Loop ----------------
void loop() {
  publishTemperature();  // periodic USB print of TempC
  handleBluetooth();
  updateStatusLED();
}

// ---------------- BLE Command Handler ----------------
void handleBluetooth() {
  while (Serial.available()) {
    char c = Serial.read();
    rxBuf += c;

    // Heat pad power command: "NNN;"
    if (c == ';') {
      int val = rxBuf.toInt();
      setHeatPadPercent(val);
      Serial.print("\nHeat Pad -> "); Serial.print(val); Serial.println("%");
      rxBuf = "";
    }

    // ---- STOP commands ----
    if (rxBuf.endsWith("AS"))  { Serial.println("Agitator STOP"); rampDownMotor(STIR_PWM, STIR_LOW, stirCurrentPWM); rxBuf=""; }
    if (rxBuf.endsWith("PS1")) { Serial.println("Pump1 STOP");    rampDownMotor(PUMP1_PWM, PUMP1_LOW, pump1CurrentPWM); rxBuf=""; }
    if (rxBuf.endsWith("PS2")) { Serial.println("Pump2 STOP");    rampDownMotor(PUMP2_PWM, PUMP2_LOW, pump2CurrentPWM); rxBuf=""; }
    if (rxBuf.endsWith("S"))   { Serial.println("STOP both pumps"); rampDownMotor(PUMP1_PWM, PUMP1_LOW, pump1CurrentPWM); rampDownMotor(PUMP2_PWM, PUMP2_LOW, pump2CurrentPWM); rxBuf=""; }

    // ---- START commands ----
    if (rxBuf.endsWith("A"))   { Serial.println("Agitator START"); rampUpMotor(STIR_PWM, STIR_LOW, STIR_TARGET_PWM, stirCurrentPWM); rxBuf=""; }
    if (rxBuf.endsWith("P1"))  { Serial.println("Pump1 START");    rampUpMotor(PUMP1_PWM, PUMP1_LOW, PUMP_TARGET_PWM, pump1CurrentPWM); rxBuf=""; }
    if (rxBuf.endsWith("P2"))  { Serial.println("Pump2 START");    rampUpMotor(PUMP2_PWM, PUMP2_LOW, PUMP_TARGET_PWM, pump2CurrentPWM); rxBuf=""; }
    if (rxBuf.endsWith("P"))   { Serial.println("Both pumps START"); rampUpMotor(PUMP1_PWM, PUMP1_LOW, BOTH_PUMPS_PWM, pump1CurrentPWM); rampUpMotor(PUMP2_PWM, PUMP2_LOW, BOTH_PUMPS_PWM, pump2CurrentPWM); rxBuf=""; }

    if (rxBuf.length() > 16) rxBuf.remove(0, rxBuf.length() - 8);
  }
}

// ---------------- Heat Pad ----------------
void setHeatPadPercent(int percent) {
  percent = constrain(percent, 0, 100);
  heatLevel = percent;
  heatPWM   = map(percent, 0, 100, 0, 255);
  analogWrite(HEAT_PAD_PIN, heatPWM);
}

// ---------------- Motor Helpers ----------------
void rampUpMotor(int pinPWM, int pinLow, int targetPWM, int &currentPWM) {
  digitalWrite(pinLow, LOW);
  for (int v = currentPWM; v <= targetPWM; v += STEP) {
    analogWrite(pinPWM, v);
    currentPWM = v;
    delay(STEP_DT);
  }
}

void rampDownMotor(int pinPWM, int pinLow, int &currentPWM) {
  digitalWrite(pinLow, LOW);
  for (int v = currentPWM; v >= 0; v -= STEP) {
    analogWrite(pinPWM, v);
    currentPWM = v;
    delay(STEP_DT);
  }
  analogWrite(pinPWM, 0);
  currentPWM = 0;
}

void stopPumps() {
  analogWrite(PUMP1_PWM, 0); digitalWrite(PUMP1_LOW, LOW);
  analogWrite(PUMP2_PWM, 0); digitalWrite(PUMP2_LOW, LOW);
  pump1CurrentPWM = pump2CurrentPWM = 0;
}

// ---------------- WS2812 Startup Animation ----------------
void runStartupAnimation() {
  strip.clear();
  for (int pos = 2; pos <= 18; pos++) {
    strip.clear();
    strip.setPixelColor(pos, strip.Color(128, 0, 0));
    if (pos - 1 >= 2) strip.setPixelColor(pos - 1, strip.Color(75, 0, 0));
    if (pos - 2 >= 2) strip.setPixelColor(pos - 2, strip.Color(30, 0, 0));
    if (pos + 1 <= 18) strip.setPixelColor(pos + 1, strip.Color(75, 0, 0));
    if (pos + 2 <= 18) strip.setPixelColor(pos + 2, strip.Color(30, 0, 0));
    strip.show();
    delay(100);
  }
  strip.clear();
  strip.show();
}

// ---------------- BLE Connect Animation ----------------
void runBleConnectAnimation() {
  int leftStart = 2, leftEnd = 7;
  int rightStart = 18, rightEnd = 11;

  for (int step = 0; step < 6; step++) {
    strip.clear();
    for (int i = leftStart; i <= leftStart + step && i <= leftEnd; i++)
      strip.setPixelColor(i, strip.Color(40, 75, 128));
    for (int i = rightStart; i >= rightStart - step && i >= rightEnd; i--)
      strip.setPixelColor(i, strip.Color(40, 75, 128));
    strip.show();
    delay(150);
  }
  for (int i = 2; i <= 7; i++) strip.setPixelColor(i, strip.Color(40, 75, 128));
  for (int i = 11; i <= 18; i++) strip.setPixelColor(i, strip.Color(40, 75, 128));
  strip.show();
}

// ---------------- WS2812 Status LED ----------------
void updateStatusLED() {
  bool bleConnected = digitalRead(BLE_STATE_PIN);
  unsigned long now = millis();

  if (bleConnected && !lastBleConnected) runBleConnectAnimation();
  lastBleConnected = bleConnected;

  if (!bleConnected) {
    if (breathState == BRIGHTENING && now - lastBreathUpdate > 40) {
      brightness += fadeAmount;
      if (brightness >= 128) { brightness = 128; breathState = DARKENING; }
      lastBreathUpdate = now;
    } else if (breathState == DARKENING && now - lastBreathUpdate > 50) {
      brightness -= fadeAmount;
      if (brightness <= 0) { brightness = 0; breathState = PAUSED; pauseStartTime = now; }
      lastBreathUpdate = now;
    } else if (breathState == PAUSED && now - pauseStartTime > pauseDuration) {
      breathState = BRIGHTENING;
    }

    strip.clear();
    for (int i = 2; i <= 7; i++) strip.setPixelColor(i, strip.Color(brightness, 0, 0));
    for (int i = 11; i <= 18; i++) strip.setPixelColor(i, strip.Color(brightness, 0, 0));
    strip.show();
  } else {
    strip.clear();
    if (heatLevel > 0) {
      for (int i = 2; i <= 7; i++) strip.setPixelColor(i, strip.Color(255, 255, 0));
      for (int i = 11; i <= 18; i++) strip.setPixelColor(i, strip.Color(255, 255, 0));
    } else {
      for (int i = 2; i <= 7; i++) strip.setPixelColor(i, strip.Color(40, 75, 128));
      for (int i = 11; i <= 18; i++) strip.setPixelColor(i, strip.Color(40, 75, 128));
    }
    strip.show();
  }
}

// Read current temperature (°C) from the first DS18B20 on the OneWire bus
float readTemperatureC() {
  sensors.requestTemperatures();
  float tC = sensors.getTempCByIndex(0);
  if (tC == DEVICE_DISCONNECTED_C) {
    return NAN;
  }
  return tC;
}

// Print temperature to USB Serial at 1 Hz
void publishTemperature() {
  unsigned long now = millis();
  if (now - TEMP_LAST_MS < TEMP_PERIOD_MS) return;
  TEMP_LAST_MS = now;

  float tC = readTemperatureC();
  if (isnan(tC)) {
    Serial.println(F("Temp: sensor not found (check wiring & 4.7k pull-up)"));
    return;
  }
  Serial.print(F("TempC: ")); Serial.println(tC, 2);
}
