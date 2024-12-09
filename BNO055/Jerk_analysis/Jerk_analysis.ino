#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Mouse.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

float lin_accx, lin_accy, lin_accz;
float prevx = 0, prevy = 0, prevz = 0;
float jerkX = 0, jerkY = 0, jerkZ = 0;
const float noiseThreshold = 0.15;
unsigned long prevTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for serial port

  if (!bno.begin()) {
    Serial.println("Error: BNO055 not detected. Check connections.");
    while (1);
  }

  bno.setExtCrystalUse(true);
  Mouse.begin();
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0; // Time in seconds

  // Ensure calibration
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  if (gyro < 3) {
    Serial.println("Gyro not calibrated.");
    return;
  }

  // Read linear acceleration
  imu::Vector<3> lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  lin_accx = -lin_acc.x();
  lin_accy = lin_acc.y();
  lin_accz = lin_acc.z();

  // Apply noise threshold
  lin_accx = abs(lin_accx) < noiseThreshold ? 0 : lin_accx;
  lin_accy = abs(lin_accy) < noiseThreshold ? 0 : lin_accy;
  lin_accz = abs(lin_accz) < noiseThreshold ? 0 : lin_accz;

  // Calculate jerk
  if (deltaTime > 0) {
    jerkX = (lin_accx - prevx) / deltaTime;
    jerkY = (lin_accy - prevy) / deltaTime;
    jerkZ = (lin_accz - prevz) / deltaTime;

    prevx = lin_accx;
    prevy = lin_accy;
    prevz = lin_accz;
    prevTime = currentTime;

    // Print results
    //Serial.print("Jerk X: ");
    Serial.print(jerkX);
    Serial.print(" ");
    Serial.print(jerkY);
    Serial.print(" ");
    Serial.println(jerkZ);
  }

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
