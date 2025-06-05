#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>

MPU6050 imu;

// Low-pass filter
float prevFilteredMag = 0;
float alpha = 0.9; // Low-pass filter smoothing factor

// Buffer
const int bufferSize = 100;
float magBuffer[bufferSize];
unsigned long timeBuffer[bufferSize];
int bufferIndex = 0;

// Batching
const int BATCH_SIZE = 30;
Reading buffer[BATCH_SIZE];
int bufferIndex2 = 0; 

unsigned long lastBreathTime = 0;

// Add Wi-Fi & server config
const char* ssid = "";
const char* password = "";
const char* serverUrl = "";
const char* apiKey = "my-secret-key"; 

// Add time 
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 15000; // 15 seconds

void setup() {
  Serial.begin(115200);
  Wire.begin();
  imu.initialize();

  if (!imu.testConnection()) {
    Serial.println("IMU connection failed");
    while (1);
  }

  Serial.println("IMU initialized");
}

void loop() {
  int16_t ax, ay, az;
  imu.getAcceleration(&ax, &ay, &az);

  // Convert to g
  float axg = ax / 16384.0;
  float ayg = ay / 16384.0;
  float azg = az / 16384.0;

  // Compute magnitude of acceleration vector
  float accMag = sqrt(axg * axg + ayg * ayg + azg * azg);

  // Apply low-pass filter
  float filteredMag = alpha * prevFilteredMag + (1 - alpha) * accMag;
  prevFilteredMag = filteredMag;

  // Store in circular buffer
  magBuffer[bufferIndex] = filteredMag;
  timeBuffer[bufferIndex] = millis();

  // Detect local minima (breath cycle point)
  if (bufferIndex > 2) {
    int prev = (bufferIndex - 1 + bufferSize) % bufferSize;
    int prev2 = (bufferIndex - 2 + bufferSize) % bufferSize;

    if (magBuffer[prev2] > magBuffer[prev] && magBuffer[prev] < filteredMag) {
      unsigned long now = millis();
      if (now - lastBreathTime > 2000) { // only count if at least 2s apart (max 30 BPM)
        float bpm = 60000.0 / (now - lastBreathTime);
        lastBreathTime = now;

        Serial.print("Breath detected - BPM: ");
        Serial.println(bpm);
      }
    }
  }

  bufferIndex = (bufferIndex + 1) % bufferSize;
  delay(100); // ~10 Hz sampling
}

void sendBatch() {
  Serial.print("Uploading ");
  Serial.print(bufferIndex);
  Serial.println(" readings...");

  Serial.println("Turning on Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
    delay(600);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi failed. Skipping send.");
    return;
  }

  HTTPClient http;
  http.begin(serverUrl);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-api-key", apiKey);

  String json = "[";
  for (int i = 0; i < bufferIndex; i++) {
    json += "{\"timestamp\":" + String(buffer[i].timestamp) +
            ",\"hr\":" + String(buffer[i].hr) +
            ",\"spo2\":" + String(buffer[i].spo2) + "}";
    if (i < bufferIndex - 1) json += ",";
  }
  json += "]";

  int responseCode = http.POST(json);
  Serial.print("POST response: ");
  Serial.println(responseCode);
  http.end();

  bufferIndex = 0;

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  Serial.println("Wi-Fi off. Data sent.");
}