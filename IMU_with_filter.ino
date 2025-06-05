#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 imu;

float prevFilteredMag = 0;
float alpha = 0.9; // Low-pass filter smoothing factor

const int bufferSize = 100;
float magBuffer[bufferSize];
unsigned long timeBuffer[bufferSize];
int bufferIndex = 0;

unsigned long lastBreathTime = 0;

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

  // Convert to g (assuming 16-bit signed values and default range of ±2g)
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