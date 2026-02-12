#include <Wire.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_AHTX0.h>
#include <WiFi.h>
#include <HTTPClient.h>

// Pin Definitions
#define LED_PWM_PIN 5
#define SWITCH_PIN 2
#define RX_PIN 20 // mmWave Sensor RX (Connect to Sensor TX)
#define TX_PIN 21 // mmWave Sensor TX (Connect to Sensor RX)

// WiFi Credentials
const char* ssid = "x";
const char* password = "x";
const char* serverUrl = "http://homebridge.local:51828/";

// Sensor objects
Adafruit_VEML7700 veml;
Adafruit_AHTX0 aht20;
HardwareSerial mmWaveSerial(1);

// Variables
bool motionDetected = false;
float lux = 0.0;
float temperature = 0.0;
float humidity = 0.0;
int detectedDistance = 0;
bool lightSwitchState = false;

// Flags
bool vemlInitialized = false;
bool aht20Initialized = false;
unsigned long lastResponseTime = 0; // Watchdog timer

void sendHttpRequest(const char* accessoryId, const char* param, String value) {
  HTTPClient http;
  String url = String(serverUrl) + "?accessoryId=" + accessoryId + "&" + param + "=" + value;
  http.begin(url);
  int httpResponseCode = http.GET();
  Serial.printf("HTTP Request sent: %d\n", httpResponseCode);
  http.end();
}

// WiFi Setup
void setupWiFi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? "WiFi connected" : "WiFi failed!");
}

// Sensor Setup
void setupSensors() {
  Wire.begin(D4, D5);
  vemlInitialized = veml.begin();
  aht20Initialized = aht20.begin();
}

// Initialize mmWave Sensor
void sendMmWaveInitCommand() {
  byte command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};
  mmWaveSerial.write(command, sizeof(command));
  delay(500); // Allow sensor time to process initialization
  mmWaveSerial.flush(); // Clear serial buffer
  Serial.println("Initialization command sent to mmWave sensor.");
}

// Read mmWave Data
void readMmWaveData() {
  String rawData = "";
  int presenceFlag = 0;
  detectedDistance = 0;

  while (mmWaveSerial.available()) {
    String line = mmWaveSerial.readStringUntil('\n');
    rawData += line;

    if (line.startsWith("ON")) {
      presenceFlag = 1;
    } else if (line.startsWith("OFF")) {
      presenceFlag = 0;
    } else if (line.startsWith("Range")) {
      detectedDistance = line.substring(6).toInt();
    }
  }

  // Update status
  motionDetected = (presenceFlag > 0);
  lastResponseTime = millis(); // Reset watchdog timer

  // Debug output
  Serial.print("Raw Data: ");
  Serial.println(rawData);
  Serial.print("Presence: ");
  Serial.println(presenceFlag);
  Serial.print("Distance: ");
  Serial.println(detectedDistance);
}

// Read Environmental Sensors
void readEnvironmentalSensors() {
  if (vemlInitialized) {
    lux = veml.readLux();
    Serial.print("Lux: ");
    Serial.println(lux);
  } else {
    Serial.println("VEML7700 failed.");
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
  } else {
    Serial.println("AHT20 failed.");
  }
}

// Send updates to Homebridge
void sendHomebridgeUpdates() {
  sendHttpRequest("occupancySensor", "state", motionDetected ? "true" : "false");
  sendHttpRequest("tempSensor", "value", String(temperature));
  sendHttpRequest("humiditySensor", "value", String(humidity));
  sendHttpRequest("luxSensor", "value", String(lux));
}

// Watchdog Timer Reset
void checkSensorTimeout() {
  if (millis() - lastResponseTime > 5000) { // 5-second timeout
    Serial.println("Sensor unresponsive, reinitializing...");
    sendMmWaveInitCommand();
  }
}

// Setup
void setup() {
  Serial.begin(115200);
  pinMode(LED_PWM_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);
  setupWiFi();
  setupSensors();
  mmWaveSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  sendMmWaveInitCommand();
}

// Loop
void loop() {
  readMmWaveData();
  readEnvironmentalSensors();
  sendHomebridgeUpdates();
  checkSensorTimeout();
  delay(1000);
}