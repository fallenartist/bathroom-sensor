#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_AHTX0.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <math.h>
#include <string.h>

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "REPLACE_WITH_WIFI_SSID"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "REPLACE_WITH_WIFI_PASSWORD"
#endif

#ifndef HOMEBRIDGE_URL
#define HOMEBRIDGE_URL "http://homebridge.local:51828/"
#endif

// Pin Definitions
constexpr uint8_t LED_PWM_PIN = 5;
constexpr uint8_t SWITCH_PIN = 2; // Reserved for future logic
constexpr int RX_PIN = 20;        // mmWave Sensor RX (Connect to Sensor TX)
constexpr int TX_PIN = 21;        // mmWave Sensor TX (Connect to Sensor RX)
constexpr int I2C_SDA_PIN = SDA;
constexpr int I2C_SCL_PIN = SCL;

// PWM defaults (safe baseline for LED strip via MOSFET module)
constexpr uint8_t PWM_CHANNEL = 0;
constexpr uint8_t PWM_BITS = 8;
constexpr uint32_t PWM_FREQ_HZ = 5000;
constexpr uint16_t PWM_MAX = (1 << PWM_BITS) - 1;

// Timings
constexpr unsigned long WIFI_CONNECT_TIMEOUT_MS = 10000;
constexpr unsigned long WIFI_RETRY_MS = 10000;
constexpr unsigned long SENSOR_TIMEOUT_MS = 10000;
constexpr unsigned long MMWAVE_REINIT_COOLDOWN_MS = 5000;
constexpr unsigned long ENV_SENSOR_READ_MS = 1000;
constexpr unsigned long LOOP_DELAY_MS = 200;

// Lighting policy for auto mode (occupancy + ambient light).
// Targets:
// - very dark/night: dim output
// - medium ambient: brightest output
// - very bright room: off
constexpr float LUX_NIGHT = 5.0f;
constexpr float LUX_MEDIUM = 40.0f;
constexpr float LUX_VERY_BRIGHT = 180.0f;
constexpr uint8_t DUTY_NIGHT = static_cast<uint8_t>(PWM_MAX * 0.25f);

// Telemetry change thresholds
constexpr float TEMP_DELTA_C = 0.2f;
constexpr float HUMIDITY_DELTA_PCT = 1.0f;
constexpr float LUX_DELTA = 5.0f;
constexpr uint16_t CONTROL_SERVER_PORT = 80;

// HMMD mmWave protocol (Waveshare wiki)
constexpr uint8_t MMWAVE_OUTPUT_MODE_NORMAL = 0x64;
constexpr uint8_t MMWAVE_OUTPUT_MODE_REPORT = 0x04;
constexpr uint8_t MMWAVE_REPORT_CMD_PRESENCE = 0x01;
constexpr uint8_t MMWAVE_FRAME_HEADER[] = {0xF4, 0xF3, 0xF2, 0xF1};
constexpr uint8_t MMWAVE_FRAME_TAIL[] = {0xF8, 0xF7, 0xF6, 0xF5};
constexpr size_t MMWAVE_RX_BUFFER_SIZE = 96;
constexpr bool MMWAVE_RAW_HEX_DUMP = false; // Temporary debug mode
constexpr size_t MMWAVE_RAW_DUMP_LINE_BYTES = 32;
constexpr unsigned long MMWAVE_RAW_DUMP_FLUSH_MS = 250;

// Accessory IDs for Homebridge HTTP Webhooks plugin
constexpr const char* ACCESSORY_OCCUPANCY = "occupancySensor";
constexpr const char* ACCESSORY_TEMPERATURE = "tempSensor";
constexpr const char* ACCESSORY_HUMIDITY = "humiditySensor";
constexpr const char* ACCESSORY_LUX = "luxSensor";
constexpr const char* ACCESSORY_CABINET_LIGHT = "cabinetLight";

// Sensor objects
Adafruit_VEML7700 veml;
Adafruit_AHTX0 aht20;
HardwareSerial mmWaveSerial(1);
WebServer controlServer(CONTROL_SERVER_PORT);

// Variables
bool motionDetected = false;
float lux = 0.0;
float temperature = 0.0;
float humidity = 0.0;
int detectedDistance = 0;
bool lightSwitchState = false;
bool cabinetLightEnabled = false; // HomeKit ON => force full brightness override

// Flags
bool vemlInitialized = false;
bool aht20Initialized = false;
unsigned long lastMmWaveResponseTime = 0;
unsigned long lastMmWaveInitTime = 0;
unsigned long lastEnvReadTime = 0;
unsigned long lastWiFiRetryTime = 0;

// Last sent values (for change-based reporting)
bool occupancySent = false;
bool occupancySentValid = false;
float tempSent = 0.0f;
bool tempSentValid = false;
float humiditySent = 0.0f;
bool humiditySentValid = false;
float luxSent = 0.0f;
bool luxSentValid = false;
bool cabinetLightSent = false;
bool cabinetLightSentValid = false;
uint8_t mmWaveRawDumpLine[MMWAVE_RAW_DUMP_LINE_BYTES];
size_t mmWaveRawDumpLen = 0;
unsigned long mmWaveRawLastByteMs = 0;

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* serverUrl = HOMEBRIDGE_URL;

bool hasNetworkConfig() {
  return strcmp(ssid, "REPLACE_WITH_WIFI_SSID") != 0 && strcmp(password, "REPLACE_WITH_WIFI_PASSWORD") != 0;
}

bool isFinite(float value) {
  return !isnan(value) && !isinf(value);
}

void setCabinetLightEnabled(bool enabled) {
  if (cabinetLightEnabled == enabled) {
    return;
  }
  cabinetLightEnabled = enabled;
  Serial.print("cabinetLight override set to ");
  Serial.println(cabinetLightEnabled ? "ON (forced full)" : "OFF (auto mode)");
}

bool parseBoolValue(const String& raw, bool& parsed) {
  if (raw.equalsIgnoreCase("true") || raw == "1" || raw.equalsIgnoreCase("on")) {
    parsed = true;
    return true;
  }
  if (raw.equalsIgnoreCase("false") || raw == "0" || raw.equalsIgnoreCase("off")) {
    parsed = false;
    return true;
  }
  return false;
}

void sendControlResponse(bool success, const String& message) {
  String body = String("{\"success\":") + (success ? "true" : "false") + ",\"state\":" + (cabinetLightEnabled ? "true" : "false") + ",\"message\":\"" + message + "\"}";
  controlServer.send(success ? 200 : 400, "application/json", body);
}

void handleCabinetLightOn() {
  setCabinetLightEnabled(true);
  sendControlResponse(true, "cabinetLight on");
}

void handleCabinetLightOff() {
  setCabinetLightEnabled(false);
  sendControlResponse(true, "cabinetLight off");
}

void handleCabinetLightState() {
  if (controlServer.hasArg("state")) {
    bool parsedState = false;
    if (!parseBoolValue(controlServer.arg("state"), parsedState)) {
      sendControlResponse(false, "invalid state");
      return;
    }
    setCabinetLightEnabled(parsedState);
  }
  sendControlResponse(true, "ok");
}

void setupControlServer() {
  controlServer.on("/cabinetLight/on", HTTP_GET, handleCabinetLightOn);
  controlServer.on("/cabinetLight/off", HTTP_GET, handleCabinetLightOff);
  controlServer.on("/cabinetLight", HTTP_GET, handleCabinetLightState);
  controlServer.on("/healthz", HTTP_GET, []() { controlServer.send(200, "text/plain", "ok"); });
  controlServer.begin();
  Serial.printf("Control server started on port %u\n", CONTROL_SERVER_PORT);
}

bool sendHttpRequest(const char* accessoryId, const char* param, const String& value) {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }
  HTTPClient http;
  String url = String(serverUrl) + "?accessoryId=" + accessoryId + "&" + param + "=" + value;
  http.begin(url);
  int httpResponseCode = http.GET();
  Serial.printf("HTTP Request [%s]: %d\n", accessoryId, httpResponseCode);
  http.end();
  return httpResponseCode > 0 && httpResponseCode < 400;
}

// WiFi Setup
void setupWiFi() {
  if (!hasNetworkConfig()) {
    Serial.println("WiFi credentials missing. Create secrets.h from secrets.example.h.");
    return;
  }

  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  lastWiFiRetryTime = millis();

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_CONNECT_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? "WiFi connected" : "WiFi failed!");
}

void ensureWiFiConnected() {
  if (!hasNetworkConfig()) {
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  if (millis() - lastWiFiRetryTime < WIFI_RETRY_MS) {
    return;
  }

  Serial.println("WiFi disconnected, retrying...");
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  lastWiFiRetryTime = millis();
}

// Sensor Setup
void setupSensors() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  vemlInitialized = veml.begin();
  aht20Initialized = aht20.begin();

  if (!vemlInitialized) {
    Serial.println("VEML7700 initialization failed.");
  }
  if (!aht20Initialized) {
    Serial.println("AHT20 initialization failed.");
  }
}

void sendMmWaveOutputModeCommand(uint8_t mode) {
  byte command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, mode, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};
  mmWaveSerial.write(command, sizeof(command));
  mmWaveSerial.flush();
}

// Initialize mmWave Sensor in report mode using Waveshare HMMD command format.
void sendMmWaveInitCommand() {
  sendMmWaveOutputModeCommand(MMWAVE_OUTPUT_MODE_REPORT);
  delay(500); // Allow sensor time to process initialization
  while (mmWaveSerial.available()) {
    mmWaveSerial.read();
  }
  lastMmWaveInitTime = millis();
  Serial.println("Initialization command sent to mmWave sensor (report mode).");
}

void flushMmWaveRawDump() {
  if (!MMWAVE_RAW_HEX_DUMP || mmWaveRawDumpLen == 0) {
    return;
  }

  Serial.print("mmWave raw: ");
  for (size_t i = 0; i < mmWaveRawDumpLen; ++i) {
    if (mmWaveRawDumpLine[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(mmWaveRawDumpLine[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
  mmWaveRawDumpLen = 0;
}

void appendMmWaveRawByte(uint8_t byteValue) {
  if (!MMWAVE_RAW_HEX_DUMP) {
    return;
  }

  mmWaveRawLastByteMs = millis();
  mmWaveRawDumpLine[mmWaveRawDumpLen++] = byteValue;
  if (mmWaveRawDumpLen >= MMWAVE_RAW_DUMP_LINE_BYTES || byteValue == '\n') {
    flushMmWaveRawDump();
  }
}

void flushMmWaveRawDumpIfIdle() {
  if (!MMWAVE_RAW_HEX_DUMP || mmWaveRawDumpLen == 0) {
    return;
  }
  if (millis() - mmWaveRawLastByteMs >= MMWAVE_RAW_DUMP_FLUSH_MS) {
    flushMmWaveRawDump();
  }
}

void processMmWaveReportFrame(const uint8_t* payload, size_t payloadLen) {
  if (payloadLen < 3) {
    return;
  }

  // Official Waveshare report frame:
  // [presence:1][distance_cm:2][energy_gates:32...]
  if (payload[0] <= 0x01) {
    motionDetected = payload[0] > 0;
    detectedDistance = static_cast<int>(static_cast<uint16_t>(payload[1]) | (static_cast<uint16_t>(payload[2]) << 8));
    lastMmWaveResponseTime = millis();

    Serial.print("mmWave frame: present=");
    Serial.print(motionDetected ? "1" : "0");
    Serial.print(" distance=");
    Serial.println(detectedDistance);
    return;
  }

  // Compatibility fallback for short frame variants:
  // [cmd:1][presence/state:1][distance_cm:2][...]
  if (payloadLen < 4 || payload[0] != MMWAVE_REPORT_CMD_PRESENCE) {
    return;
  }

  uint8_t presenceState = payload[1];
  uint16_t distance = static_cast<uint16_t>(payload[2]) | (static_cast<uint16_t>(payload[3]) << 8);
  motionDetected = presenceState > 0;
  detectedDistance = static_cast<int>(distance);
  lastMmWaveResponseTime = millis();

  Serial.print("mmWave frame(alt): state=");
  Serial.print(presenceState, HEX);
  Serial.print(" distance=");
  Serial.println(detectedDistance);
}

void consumeMmWaveFrames(uint8_t* buffer, size_t& bufferLen) {
  while (bufferLen >= 10) { // 4-byte header + 2-byte len + 0 payload + 4-byte tail
    size_t headerIndex = 0;
    bool foundHeader = false;
    for (; headerIndex + 4 <= bufferLen; ++headerIndex) {
      if (memcmp(buffer + headerIndex, MMWAVE_FRAME_HEADER, sizeof(MMWAVE_FRAME_HEADER)) == 0) {
        foundHeader = true;
        break;
      }
    }

    if (!foundHeader) {
      if (bufferLen > 3) {
        memmove(buffer, buffer + (bufferLen - 3), 3);
        bufferLen = 3;
      }
      return;
    }

    if (headerIndex > 0) {
      memmove(buffer, buffer + headerIndex, bufferLen - headerIndex);
      bufferLen -= headerIndex;
    }

    if (bufferLen < 6) {
      return;
    }

    uint16_t payloadLen = static_cast<uint16_t>(buffer[4]) | (static_cast<uint16_t>(buffer[5]) << 8);
    size_t totalFrameLen = 4 + 2 + payloadLen + 4;
    if (payloadLen > MMWAVE_RX_BUFFER_SIZE || totalFrameLen > MMWAVE_RX_BUFFER_SIZE) {
      memmove(buffer, buffer + 1, bufferLen - 1);
      bufferLen -= 1;
      continue;
    }

    if (bufferLen < totalFrameLen) {
      return;
    }

    if (memcmp(buffer + totalFrameLen - 4, MMWAVE_FRAME_TAIL, sizeof(MMWAVE_FRAME_TAIL)) != 0) {
      memmove(buffer, buffer + 1, bufferLen - 1);
      bufferLen -= 1;
      continue;
    }

    processMmWaveReportFrame(buffer + 6, payloadLen);

    if (bufferLen > totalFrameLen) {
      memmove(buffer, buffer + totalFrameLen, bufferLen - totalFrameLen);
    }
    bufferLen -= totalFrameLen;
  }
}

void processMmWaveLine(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (line.startsWith("ON")) {
    motionDetected = true;
  } else if (line.startsWith("OFF")) {
    motionDetected = false;
  } else if (line.startsWith("Range")) {
    int separator = line.indexOf(':');
    if (separator < 0) {
      separator = line.indexOf(' ');
    }
    if (separator >= 0 && separator + 1 < line.length()) {
      detectedDistance = line.substring(separator + 1).toInt();
    }
  }

  lastMmWaveResponseTime = millis();
  Serial.print("mmWave: ");
  Serial.println(line);
}

// Read mmWave Data (report-frame parsing with ASCII fallback).
void readMmWaveData() {
  static uint8_t frameBuffer[MMWAVE_RX_BUFFER_SIZE];
  static size_t frameBufferLen = 0;
  static String lineBuffer;

  while (mmWaveSerial.available()) {
    uint8_t byteValue = static_cast<uint8_t>(mmWaveSerial.read());
    appendMmWaveRawByte(byteValue);

    if (frameBufferLen < sizeof(frameBuffer)) {
      frameBuffer[frameBufferLen++] = byteValue;
      consumeMmWaveFrames(frameBuffer, frameBufferLen);
    } else {
      frameBufferLen = 0;
    }

    if (byteValue == '\r') {
      continue;
    }
    if (byteValue == '\n') {
      processMmWaveLine(lineBuffer);
      lineBuffer = "";
      continue;
    }

    if (byteValue >= 32 && byteValue <= 126 && lineBuffer.length() < 96) {
      lineBuffer += static_cast<char>(byteValue);
    } else {
      if (byteValue == 0 || lineBuffer.length() >= 96) {
        lineBuffer = "";
      }
    }
  }

  flushMmWaveRawDumpIfIdle();
}

// Read Environmental Sensors
void readEnvironmentalSensors() {
  if (millis() - lastEnvReadTime < ENV_SENSOR_READ_MS) {
    return;
  }
  lastEnvReadTime = millis();

  if (vemlInitialized) {
    lux = veml.readLux();
    Serial.print("Lux: ");
    Serial.println(lux);
  }

  if (aht20Initialized) {
    sensors_event_t humidityEvent, tempEvent;
    aht20.getEvent(&humidityEvent, &tempEvent);
    temperature = tempEvent.temperature;
    humidity = humidityEvent.relative_humidity;
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C, Humidity: ");
    Serial.println(humidity);
  }
}

uint8_t computeDutyFromLux(float luxValue) {
  if (luxValue <= LUX_NIGHT) {
    return DUTY_NIGHT;
  }
  if (luxValue >= LUX_VERY_BRIGHT) {
    return 0;
  }

  if (luxValue < LUX_MEDIUM) {
    const float upRatio = (luxValue - LUX_NIGHT) / (LUX_MEDIUM - LUX_NIGHT);
    return static_cast<uint8_t>(DUTY_NIGHT + upRatio * (PWM_MAX - DUTY_NIGHT));
  }

  const float downRatio = (LUX_VERY_BRIGHT - luxValue) / (LUX_VERY_BRIGHT - LUX_MEDIUM);
  return static_cast<uint8_t>(PWM_MAX * downRatio);
}

void updateLedOutput() {
  // HomeKit cabinetLight acts as a manual override switch.
  lightSwitchState = cabinetLightEnabled;

  uint8_t duty = 0;
  if (cabinetLightEnabled) {
    duty = PWM_MAX;
  } else if (motionDetected) {
    float ambientLux = vemlInitialized && isFinite(lux) ? lux : 0.0f;
    duty = computeDutyFromLux(ambientLux);
  }

  ledcWrite(PWM_CHANNEL, duty);
}

bool shouldSendFloat(float newValue, float oldValue, bool oldValid, float minDelta) {
  if (!isFinite(newValue)) {
    return false;
  }
  return !oldValid || fabsf(newValue - oldValue) >= minDelta;
}

// Send updates to Homebridge only when values meaningfully change
void sendHomebridgeUpdates() {
  if (!hasNetworkConfig() || WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (!occupancySentValid || occupancySent != motionDetected) {
    if (sendHttpRequest(ACCESSORY_OCCUPANCY, "state", motionDetected ? "true" : "false")) {
      occupancySent = motionDetected;
      occupancySentValid = true;
    }
  }

  if (!cabinetLightSentValid || cabinetLightSent != cabinetLightEnabled) {
    if (sendHttpRequest(ACCESSORY_CABINET_LIGHT, "state", cabinetLightEnabled ? "true" : "false")) {
      cabinetLightSent = cabinetLightEnabled;
      cabinetLightSentValid = true;
    }
  }

  if (shouldSendFloat(temperature, tempSent, tempSentValid, TEMP_DELTA_C)) {
    if (sendHttpRequest(ACCESSORY_TEMPERATURE, "value", String(temperature, 2))) {
      tempSent = temperature;
      tempSentValid = true;
    }
  }

  if (shouldSendFloat(humidity, humiditySent, humiditySentValid, HUMIDITY_DELTA_PCT)) {
    if (sendHttpRequest(ACCESSORY_HUMIDITY, "value", String(humidity, 1))) {
      humiditySent = humidity;
      humiditySentValid = true;
    }
  }

  if (shouldSendFloat(lux, luxSent, luxSentValid, LUX_DELTA)) {
    if (sendHttpRequest(ACCESSORY_LUX, "value", String(lux, 1))) {
      luxSent = lux;
      luxSentValid = true;
    }
  }
}

// Watchdog Timer Reset
void checkSensorTimeout() {
  if (millis() - lastMmWaveResponseTime > SENSOR_TIMEOUT_MS) {
    if (motionDetected) {
      Serial.println("mmWave stale -> occupancy forced OFF.");
    }
    motionDetected = false;

    if (millis() - lastMmWaveInitTime > MMWAVE_REINIT_COOLDOWN_MS) {
      Serial.println("Sensor unresponsive, reinitializing...");
      sendMmWaveInitCommand();
    }
  }
}

// Setup
void setup() {
  Serial.begin(115200);
  unsigned long serialStartMs = millis();
  while (!Serial && millis() - serialStartMs < 3000) {
    delay(10);
  }
  Serial.println("Booting bathroom sensor firmware...");
  if (MMWAVE_RAW_HEX_DUMP) {
    Serial.println("mmWave raw hex dump is ENABLED (temporary debug mode).");
  }

  pinMode(LED_PWM_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);
  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(LED_PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);

  setupWiFi();
  setupSensors();
  setupControlServer();
  mmWaveSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  sendMmWaveInitCommand();
  lastMmWaveResponseTime = millis();
}

// Loop
void loop() {
  ensureWiFiConnected();
  controlServer.handleClient();
  readMmWaveData();
  checkSensorTimeout();
  readEnvironmentalSensors();
  updateLedOutput();
  sendHomebridgeUpdates();
  delay(LOOP_DELAY_MS);
}
