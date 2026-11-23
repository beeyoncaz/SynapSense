#include <Arduino.h>
#include "LSM6DS3.h"
#include "Wire.h"

LSM6DS3 imu(I2C_MODE, 0x6A);     // IMU on the internal I2C bus, address 0x6A

#define SAMPLE_RATE  50          // samples per second
#define WINDOW_SEC   3           // seconds of data to keep
#define WINDOW_SIZE  (SAMPLE_RATE * WINDOW_SEC)   // 150 samples

#ifndef LED_RED
  #define LED_RED   LED_BUILTIN
#endif
#ifndef LED_GREEN
  #define LED_GREEN LED_BUILTIN
#endif

// Window (ring buffer): newest sample overwrites the oldest
float ax[WINDOW_SIZE], ay[WINDOW_SIZE], az[WINDOW_SIZE];   // acceleration (g)
float gx[WINDOW_SIZE], gy[WINDOW_SIZE], gz[WINDOW_SIZE];   // rotation (deg/s)
int idx = 0;                     // where the next sample goes

unsigned long lastSample = 0;

void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH);      // all LEDs off (active LOW)
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  Serial.begin(115200);

  if (imu.begin() != 0) {
    Serial.println("IMU error");
    while (1) {
      Serial.println("IMU error");     // repeat so you can see it
      digitalWrite(LED_RED, LOW);
      delay(200);
      digitalWrite(LED_RED, HIGH);
      delay(200);
    }
  }
  Serial.println("IMU ok");
}

void loop() {
  // Green LED blinks once a second = code is running
  digitalWrite(LED_GREEN, (millis() / 500) % 2);

  // Take a sample every 20 ms (50 Hz)
  if (millis() - lastSample >= 1000 / SAMPLE_RATE) {
    lastSample = millis();

    ax[idx] = imu.readFloatAccelX();
    ay[idx] = imu.readFloatAccelY();
    az[idx] = imu.readFloatAccelZ();
    gx[idx] = imu.readFloatGyroX();
    gy[idx] = imu.readFloatGyroY();
    gz[idx] = imu.readFloatGyroZ();

    // Total movement strength (used later by the detector)
    float accMag = sqrt(ax[idx]*ax[idx] + ay[idx]*ay[idx] + az[idx]*az[idx]);

    // Teleplot
    Serial.print(">ax:");  Serial.println(ax[idx]);
    Serial.print(">ay:");  Serial.println(ay[idx]);
    Serial.print(">az:");  Serial.println(az[idx]);
    Serial.print(">gx:");  Serial.println(gx[idx]);
    Serial.print(">gy:");  Serial.println(gy[idx]);
    Serial.print(">gz:");  Serial.println(gz[idx]);
    Serial.print(">accMag:"); Serial.println(accMag);

    idx = (idx + 1) % WINDOW_SIZE;   // move to next slot, wrap around at the end
  }
}