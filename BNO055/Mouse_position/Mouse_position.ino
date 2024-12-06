#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Mouse.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/

float velocityX = 0, velocityY = 0;   // Velocities in X and Y directions
float positionX = 0, positionY = 0;  // Current positions
float prevPositionX = 0, prevPositionY = 0; // Previous positions
unsigned long prevTime = 0;          // To calculate delta time
const float noiseThreshold = 0.1;    // Threshold to filter small accelerations
const float friction = 0.7;         // Friction factor to reduce drift
const float sensitivity = 200;       // Cursor sensitivity scaling factor


void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

    // Start Mouse control
  Mouse.begin();
  
  prevTime = millis();
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2  // Get calibration status
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  // Ensure system calibration is adequate
  if (accel < 3) {
bno.getCalibration(&system, &gyro, &accel, &mag);
 Serial.print("CALIBRATION: Sys=");
Serial.print(system, DEC);
Serial.print(" Gyro=");
Serial.print(gyro, DEC);
Serial.print(" Accel=");
Serial.print(accel, DEC);
Serial.print(" Mag=");
 Serial.println(mag, DEC);
 
    return;
  }
  // Get linear acceleration
  imu::Vector<3> lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Get time interval
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0; // Convert ms to seconds
  prevTime = currentTime;

  // Extract acceleration values
  float ax = lin_acc.x();
  float ay = lin_acc.y();

  // Ignore small accelerations (noise filtering)
  if (abs(ax) < noiseThreshold) ax = 0;
  if (abs(ay) < noiseThreshold) ay = 0;

  // Update velocities
  velocityX = (velocityX + ax * deltaTime*100) * friction;
  velocityY = (velocityY + ay * deltaTime*100) * friction;

  // Update positions
  positionX += velocityX * deltaTime;
  positionY += velocityY * deltaTime;

  // Calculate relative movement
  float deltaX = (positionX - prevPositionX) * sensitivity;
  float deltaY = (positionY - prevPositionY) * sensitivity;

  // Update previous positions
  prevPositionX = positionX;
  prevPositionY = positionY;

  // Move the mouse cursor relative to its current position
  Mouse.move(-deltaX, deltaY);

  // Debugging: Print acceleration, velocity, and position
  Serial.print("Ax: ");
  Serial.print(ax);
  Serial.print(" Ay: ");
  Serial.print(ay);
  Serial.print(" | Vx: ");
  Serial.print(velocityX);
  Serial.print(" Vy: ");
  Serial.print(velocityY);
  Serial.print(" | Px: ");
  Serial.print(positionX);
  Serial.print(" Py: ");
  Serial.println(positionY);

  //delay(10); // Small delay for smoother motion

  /* Display the floating point data */
  // Serial.print("X: ");
  // Serial.print(euler.x());
  // Serial.print(" Y: ");
  // Serial.print(euler.y());
  // Serial.print(" Z: ");
  // Serial.print(euler.z());
  // Serial.print("\t\t");

  /*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  */

  /* Display calibration status for each sensor. */
  // uint8_t system, gyro, accel, mag = 0;
  // bno.getCalibration(&system, &gyro, &accel, &mag);
  // Serial.print("CALIBRATION: Sys=");
  // Serial.print(system, DEC);
  // Serial.print(" Gyro=");
  // Serial.print(gyro, DEC);
  // Serial.print(" Accel=");
  // Serial.print(accel, DEC);
  // Serial.print(" Mag=");
  // Serial.println(mag, DEC);

  //delay(BNO055_SAMPLERATE_DELAY_MS);
}
