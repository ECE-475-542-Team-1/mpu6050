#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>

MPU6050 imu;

const String Current_time;

// Low-pass filter
float prevFilteredMag = 0;
float alpha = 0.9;

// Buffer
const int bufferSize = 10000;
float magBuffer[bufferSize];
unsigned long timeBuffer[bufferSize];
int bufferIndex = 0;

unsigned long lastBreathTime = 0;

const char* ssid = "iPhone (156)";
const char* password = "key-1234";
const char* serverUrl = "http://172.20.10.2:3000/api/imu";
const char* apiKey = "my-secret-key";


struct Reading {
  unsigned long timestamp; 
  float respiratoryRate;
  
};

const int BATCH_SIZE_2 = 30;
Reading buffer[BATCH_SIZE_2];
int bufferIndex2 = 0;

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 15000; // 15 seconds

bool isStill(float* buffer, int size, float threshold) {
  float mean = 0;
  for (int i = 0; i < size; i++) {
    mean += buffer[i];
  }
  mean /= size;

  float variance = 0;
  for (int i = 0; i < size; i++) {
    variance += (buffer[i] - mean) * (buffer[i] - mean);
  }
  variance /= size;

  return variance < threshold;
}

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

  float axg = ax / 16384.0;
  float ayg = ay / 16384.0;
  float azg = az / 16384.0;

  float accMag = sqrt(axg * axg + ayg * ayg + azg * azg) - 1.1;
  float filteredMag = alpha * prevFilteredMag + (1 - alpha) * accMag;
  prevFilteredMag = filteredMag;

  magBuffer[bufferIndex] = accMag; //used to be filteredMag
  timeBuffer[bufferIndex] = millis();

  float sum = 0;
  for (int i = 0; i < bufferSize; i++) sum += magBuffer[i];
  float mag_average = sum / bufferSize;

  if (bufferIndex > 7) {
    int prev = (bufferIndex - 1 + bufferSize) % bufferSize;
    int prev2 = (bufferIndex - 2 + bufferSize) % bufferSize;
    int prev3 = (bufferIndex - 3 + bufferSize) % bufferSize;
    int prev4 = (bufferIndex - 4 + bufferSize) % bufferSize;
    int prev5 = (bufferIndex - 5 + bufferSize) % bufferSize;
    int prev6 = (bufferIndex - 6 + bufferSize) % bufferSize;
    int prev7 = (bufferIndex - 7 + bufferSize) % bufferSize;

    float delta = fabs(magBuffer[prev] - filteredMag);

    Serial.print("["); 
    Serial.print(magBuffer[prev7]);
    Serial.print(magBuffer[prev6]);
    Serial.print(magBuffer[prev5]);
    Serial.print(magBuffer[prev4]);
    Serial.print(magBuffer[prev3]);
    Serial.print(magBuffer[prev2]);
    Serial.print(magBuffer[prev]);
    Serial.print("[");
    Serial.println(); 

    if (
      fabsf(magBuffer[prev7] - magBuffer[prev6]) > 0.0 &&
      fabsf(magBuffer[prev6] - magBuffer[prev5]) > 0.0 &&
      fabsf(magBuffer[prev5] - magBuffer[prev4]) > 0.0 &&
      fabsf(magBuffer[prev4] - magBuffer[prev3]) > 0.0 &&
      fabsf(magBuffer[prev3] - magBuffer[prev2]) > 0.0 &&
      fabsf(magBuffer[prev2] - magBuffer[prev]) > 0.0 
      // mag_average > 0.06 &&
      // delta > 0.04 && 
      // !isStill(magBuffer, bufferSize, 0.0005)
    ) {
      
      unsigned long now = millis();
      if (now - lastBreathTime > 3000) {
        float respiratoryRate= 60000.0 / (now - lastBreathTime);
        lastBreathTime = now;
        Serial.print("Breath detected - BPM: ");
        Serial.println(respiratoryRate);

        buffer[bufferIndex2++] = {
        .timestamp = millis() / 1000.0,
        .respiratoryRate = respiratoryRate
      };
      }
    }
  }

  bufferIndex = (bufferIndex + 1) % bufferSize;

  if ((millis() - lastSendTime > SEND_INTERVAL) && bufferIndex2 > 0) {
    lastSendTime = millis();
    sendBatch();
  }
  
  delay(100);
}


void sendBatch() {
  Serial.print("Uploading ");
  Serial.print(bufferIndex2);
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
  for (int i = 0; i < bufferIndex2; i++) {
    json += "{\"timestamp\":" + String(buffer[i].timestamp) +
            ",\"respiratoryRate\":" + String(buffer[i].respiratoryRate) + "}";
    if (i < bufferIndex2 - 1) json += ",";
  }
  json += "]";

  int responseCode = http.POST(json);
  Serial.print("POST response: ");
  Serial.println(responseCode);
  http.end();

  bufferIndex2 = 0;

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  Serial.println("Wi-Fi off. Data sent.");
}
