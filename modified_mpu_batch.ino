#include "I2Cdev.h"
#include "MPU6050.h"
#include <WiFi.h>
#include <HTTPClient.h>

// WiFi credentials
const char* ssid = "iPhone (156)";
const char* password = "key-1234";

// HTTPS server
const char* serverUrl = "http://172.20.10.2:3000/api/data";
const char* apiKey = "my-secret-key";

// MPU setup
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;

// offset functions constants
const int usDelay = 3150; // Delay in ms to hold the sampling at 200Hz
const int NFast = 1000; // Number of quick readings for averaging, the higher the better
const int NSlow = 10000; // Number of slow readings for averaging, the higher the better
const int LinesBetweenHeaders = 5;

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

int LowValue[6];
int HighValue[6];
int Smoothed[6];
int LowOffset[6];
int HighOffset[6];
int Target[6];
int LinesOut;
int N;
int i;

// Sampling parameters
const int sampleIntervalMs = 200; // 5 samples per second
const int batchDurationMs = 10 * 1000; // 1 minutes
const int maxSamples = batchDurationMs / sampleIntervalMs;

String dataBatch[maxSamples];
int sampleIndex = 0;
unsigned long lastSampleTime = 0;
unsigned long startTime = 0;

void setup() {
  Serial.begin(38400);

  // // Connect to WiFi
  // WiFi.begin(ssid, password);
  // Serial.print("Connecting to WiFi");
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("\nConnected to WiFi");

  // Initialize I2C and MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (true);
  }
  Serial.println("MPU6050 initialized");


  /** Calibrate offsets for the MPU*/

  /** setup for calibrating offsets*/
    Initialize(); //Initializate and calibrate the sensor
  for (i = iAx; i <= iGz; i++) { 
    Target[i] = 0; // Fix for ZAccel 
    HighOffset[i] = 0;
    LowOffset[i] = 0;
  } 
  Target[iAz] = 16384; // Set the taget for Z axes
  SetAveraging(NFast); // Fast averaging
  PullBracketsOut();
  PullBracketsIn();
  Serial.println("-------------- DONE --------------");


  /***/

  // Timestamp for batching
  startTime = millis();

  // LED
  pinMode(LED_BUILTIN, OUTPUT);

  SetOffsets(HighOffset); // accepts an array for [ax, ay, az, gx, gy, gz] offsets
}

void loop() {
  unsigned long currentMillis = millis();

  // Sample at fixed interval
  if (currentMillis - lastSampleTime >= sampleIntervalMs && sampleIndex < maxSamples) {
    lastSampleTime = currentMillis;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Store sample in CSV format
    dataBatch[sampleIndex++] = String(ax) + "," + String(ay) + "," + String(az) + "," +
                               String(gx) + "," + String(gy) + "," + String(gz);



    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Blink LED
  }

  prev_ax = ax;
  prev_ay = ay;
  prev_az = az; 

  if (az - prev_az >= 1000) {
    
  }

  // Upload when batch is full
  if (currentMillis - startTime >= batchDurationMs) {
    Serial.println("Batch complete. Uploading...");

    // Join all samples into a single payload
    String payload = "";
    for (int i = 0; i < sampleIndex; i++) {
      payload += dataBatch[i] + "\n";
    }

    Serial.println(payload);

    // Upload to HTTPS server
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(serverUrl);
      http.addHeader("Content-Type", "text/plain");
      http.addHeader("X-API-KEY", apiKey);

      int httpResponseCode = http.POST(payload);
      if (httpResponseCode > 0) {
        Serial.print("POST Response Code: ");
        Serial.println(httpResponseCode);
      } else {
        Serial.print("Error sending POST: ");
        Serial.println(http.errorToString(httpResponseCode));
      }
      http.end();
    }

    // Reset for next batch
    sampleIndex = 0;
    startTime = currentMillis;
  }
}

/*Initializate function for calibrating offsets*/
void Initialize() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Serial.begin(9600);
  // Init the module
  Serial.println("Initializing MPU...");
  mpu.initialize();
  Serial.println("MPU initializated");
  // Check module connection
  Serial.println("Testing device connections...");
  if(mpu.testConnection() ==  false){
    Serial.println("MPU6050 connection failed");
  while(true);
  }
  else{
    Serial.println("MPU6050 connection successful");
  }

  Serial.println("\nPID tuning Each Dot = 100 readings");
  /*
  PID tuning (actually PI) works like this: changing the offset in the MPU6050 gives instant results, 
  allowing us to use the Proportional and Integral parts of the PID to find the ideal offsets. 
  The Integral uses the error from the set point (which is zero) and adds a fraction of this error to 
  the integral value. Each reading reduces the error towards the desired offset. The greater 
  the error, the more we adjust the integral value. 
  
  The Proportional part helps by filtering out noise from the integral calculation. The Derivative part is 
  not used due to noise and the sensor being stationary. With the noise removed, the integral value stabilizes 
  after about 600 readings. At the end of each set of 100 readings, the integral value is used for the actual 
  offsets, and the last proportional reading is ignored because it reacts to any noise.
  */
  Serial.println("\nXAccel\t\tYAccel\t\tZAccel\t\tXGyro\t\tYGyro\t\tZGyro");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  Serial.println("\n600 Readings");
  mpu.PrintActiveOffsets();
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  Serial.println("700 Total Readings");
  mpu.PrintActiveOffsets();
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  Serial.println("800 Total Readings");
  mpu.PrintActiveOffsets();
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  Serial.println("900 Total Readings");
  mpu.PrintActiveOffsets();
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  Serial.println("1000 Total Readings");
  mpu.PrintActiveOffsets();
  Serial.println("\nAny of the above offsets will work nicely \n\nProving the PID with other method:");
}

void SetAveraging(int NewN) {
  N = NewN;
  Serial.print("\nAveraging ");
  Serial.print(N);
  Serial.println(" readings each time");
}

void PullBracketsOut() {
  boolean Done = false;
  int NextLowOffset[6];
  int NextHighOffset[6];

  Serial.println("Expanding:");
  ForceHeader();

  while (!Done) {
    Done = true;
    SetOffsets(LowOffset); //Set low offsets
    GetSmoothed();
    for (i = 0; i <= 5; i++) { // Get low values
      LowValue[i] = Smoothed[i];
      if (LowValue[i] >= Target[i]) {
        Done = false;
        NextLowOffset[i] = LowOffset[i] - 1000;
      } 
      else {
        NextLowOffset[i] = LowOffset[i];
      }
    }
    SetOffsets(HighOffset);
    GetSmoothed();
    for (i = 0; i <= 5; i++) { // Get high values
      HighValue[i] = Smoothed[i];
      if (HighValue[i] <= Target[i]) {
        Done = false;
        NextHighOffset[i] = HighOffset[i] + 1000;
      } 
      else {
        NextHighOffset[i] = HighOffset[i];
      }
    } 
    ShowProgress();
    for (int i = 0; i <= 5; i++) {
      LowOffset[i] = NextLowOffset[i]; 
      HighOffset[i] = NextHighOffset[i];
    }
  }
}

void PullBracketsIn() {
  boolean AllBracketsNarrow;
  boolean StillWorking;
  int NewOffset[6];

  Serial.println("\nClosing in:");
  AllBracketsNarrow = false;
  ForceHeader();
  StillWorking = true;
  while (StillWorking) {
    StillWorking = false;
    if (AllBracketsNarrow && (N == NFast)) {
      SetAveraging(NSlow);
    } 
    else {
      AllBracketsNarrow = true;
    }
    for (int i = 0; i <= 5; i++) {
      if (HighOffset[i] <= (LowOffset[i] + 1)) {
        NewOffset[i] = LowOffset[i];
      } 
      else { // Binary search
        StillWorking = true;
        NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
        if (HighOffset[i] > (LowOffset[i] + 10)) {
          AllBracketsNarrow = false;
        }
      } 
    }
    SetOffsets(NewOffset);
    GetSmoothed();
    for (i = 0; i <= 5; i++) { // Closing in
      if (Smoothed[i] > Target[i]) { // Use lower half
        HighOffset[i] = NewOffset[i];
        HighValue[i] = Smoothed[i];
      } 
      else { // Use upper half
        LowOffset[i] = NewOffset[i];
        LowValue[i] = Smoothed[i];
      } 
    }
    ShowProgress();
  } 
} 

void ForceHeader() {
  LinesOut = 99;
}

/*Function to smooth the read values*/
void GetSmoothed() {
  int16_t RawValue[6];
  long Sums[6];
  for (i = 0; i <= 5; i++) {
    Sums[i] = 0;
  }
  
/* Get Sums*/
  for (i = 1; i <= N; i++) { 
    mpu.getMotion6( & RawValue[iAx], & RawValue[iAy], & RawValue[iAz], & RawValue[iGx], & RawValue[iGy], & RawValue[iGz]);
    delayMicroseconds(usDelay);
    for (int j = 0; j <= 5; j++){
      Sums[j] = Sums[j] + RawValue[j];
    }
  } 
  for (i = 0; i <= 5; i++) {
    Smoothed[i] = (Sums[i] + N / 2) / N;
  }
} 

/*Function for configure the oba=tained offsets*/
void SetOffsets(int TheOffsets[6]) {
  mpu.setXAccelOffset(TheOffsets[iAx]);
  mpu.setYAccelOffset(TheOffsets[iAy]);
  mpu.setZAccelOffset(TheOffsets[iAz]);
  mpu.setXGyroOffset(TheOffsets[iGx]);
  mpu.setYGyroOffset(TheOffsets[iGy]);
  mpu.setZGyroOffset(TheOffsets[iGz]);
}

/*Print the progress of the reading averages, add formatting for better visualization*/
void ShowProgress() {
 /*Header*/
  if (LinesOut >= LinesBetweenHeaders) { 
    Serial.println("\t\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
    LinesOut = 0;
  } 
  Serial.print(' ');
  for (i = 0; i <= 5; i++) {
    Serial.print('[');
    Serial.print(LowOffset[i]),
    Serial.print(',');
    Serial.print(HighOffset[i]);
    Serial.print("] --> [");
    Serial.print(LowValue[i]);
    Serial.print(',');
    Serial.print(HighValue[i]);
    if (i == 5) {
      Serial.println("]");
    } 
    else {
      Serial.print("]\t");
    }
  }
  LinesOut++;
} 