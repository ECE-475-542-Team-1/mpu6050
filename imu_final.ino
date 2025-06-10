#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>

MPU6050 imu;

//

// Buffer
const int bufferSize = 10000;
float magBuffer[bufferSize];
unsigned long timeBuffer[bufferSize];
int bufferIndex = 0;


const char* ssid = "Katharine's iPhone";
const char* password = "key-1234";
const char* serverUrl = "http://35.93.66.232/api/imu";
const char* apiKey = "5b15190c-ea7b-454a-aec6-a9681c35b857";


struct Reading {
  unsigned long timestamp; 
  float respiratoryRate;
  
};

const int BATCH_SIZE_2 = 30;
Reading buffer[BATCH_SIZE_2];
int bufferIndex2 = 0;

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 15000; // 15 seconds

// Filter parameters
const float accAlpha = 0.2;          // For raw acceleration smoothing
const float baselineAlphaSlow = 0.995; // Slow baseline adaptation
const float baselineAlphaFast = 0.9;  // Fast baseline adaptation

// Detection parameters
const float breathThreshold = 0.015;   // Adjusted threshold
const unsigned long breathCooldown = 2500; // Min time between breaths (ms)
const unsigned long settleTime = 8000;    // Baseline settling time (ms)
const float minBreathBPM = 6.0;       // Minimum valid breath rate
const float maxBreathBPM = 30.0;      // Maximum valid breath rate

// Variables
float filteredAccMag = 1.0;  // Filtered acceleration magnitude
float baseline = 1.0;
unsigned long lastBreathTime = 0;
bool initialized = false;

// For dynamic threshold adjustment
float recentPeaks[5] = {0};
int peakIndex = 0;
float dynamicThreshold = breathThreshold;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize IMU
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("IMU connection failed!");
    while (1);
  }
  
  // Configure IMU
  imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // Â±2g range
  imu.setDLPFMode(MPU6050_DLPF_BW_5); // Low pass filter at 5Hz
  
  Serial.println("IMU initialized. Hold still for baseline calibration...");
  
  // Initial baseline calibration
  calibrateBaseline();
  initialized = true;
}

void calibrateBaseline() {
  const int samples = 200;
  float sum = 0;
  
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az;
    imu.getAcceleration(&ax, &ay, &az);
    float axg = ax / 16384.0;
    float ayg = ay / 16384.0;
    float azg = az / 16384.0;
    
    // Calculate magnitude
    float accMag = sqrt(axg*axg + ayg*ayg + azg*azg);
    
    // Initial filtering
    filteredAccMag = accAlpha * filteredAccMag + (1 - accAlpha) * accMag;
    sum += filteredAccMag;
    delay(10);
  }
  
  baseline = sum / samples;
}

void updateDynamicThreshold(float peakValue) {
  // Store recent peaks in circular buffer
  recentPeaks[peakIndex] = peakValue;
  peakIndex = (peakIndex + 1) % 5;
  
  // Calculate average of recent peaks
  float avgPeak = 0;
  int count = 0;
  for (int i = 0; i < 5; i++) {
    if (recentPeaks[i] > 0) {
      avgPeak += recentPeaks[i];
      count++;
    }
  }
  
  if (count > 0) {
    avgPeak /= count;
    // Set threshold to 60% of average peak height
    dynamicThreshold = avgPeak * 0.6;
    // Ensure threshold stays within reasonable bounds
    dynamicThreshold = constrain(dynamicThreshold, breathThreshold*0.5, breathThreshold*2.0);
  }
}

void loop() {
  if (!initialized) return;

  int16_t ax, ay, az;
  imu.getAcceleration(&ax, &ay, &az);
  float axg = ax / 16384.0;
  float ayg = ay / 16384.0;
  float azg = az / 16384.0;
  
  // Calculate magnitude (orientation-independent)
  float accMag = sqrt(axg*axg + ayg*ayg + azg*azg);
  
  // Apply low-pass filter
  filteredAccMag = accAlpha * filteredAccMag + (1 - accAlpha) * accMag;

  unsigned long now = millis();

  // Adaptive baseline tracking
  float currentAlpha = (now < settleTime) ? baselineAlphaFast : baselineAlphaSlow;
  baseline = currentAlpha * baseline + (1 - currentAlpha) * filteredAccMag;

  // Calculate deviation from baseline
  float deviation = filteredAccMag - baseline;

  // Detect breaths only after settling period
  if (now > settleTime) {
    static float lastDeviation = 0;
    static bool wasRising = false;
    bool isRising = (deviation > lastDeviation);
    
    // Peak detection (when rising stops)
    if (wasRising && !isRising && lastDeviation > dynamicThreshold) {
      float bpm = 60000.0 / (now - lastBreathTime);
      
      // Only count reasonable respiration rates
      if (bpm > minBreathBPM && bpm < maxBreathBPM && (now - lastBreathTime > breathCooldown)) {
        updateDynamicThreshold(lastDeviation);
        lastBreathTime = now;
        Serial.print("Breath detected - BPM: ");
        Serial.print(bpm);
        Serial.print(" | Threshold: ");
        Serial.println(dynamicThreshold, 4);

        buffer[bufferIndex2++] = {
        .timestamp = millis() / 1000.0,
        .respiratoryRate = bpm
      };
      }
    }
    
    wasRising = isRising;
    lastDeviation = deviation;

    if ((millis() - lastSendTime > SEND_INTERVAL) && bufferIndex2 > 0) {
      lastSendTime = millis();
      sendBatch();
    }
  }

  

  // Debug output
  Serial.print("Mag: ");
  Serial.print(filteredAccMag, 4);
  Serial.print(" | Baseline: ");
  Serial.print(baseline, 4);
  Serial.print(" | Dev: ");
  Serial.print(deviation, 4);
  Serial.print(" | Thresh: ");
  Serial.println(dynamicThreshold, 4);

  delay(50); // ~20Hz sample rate
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