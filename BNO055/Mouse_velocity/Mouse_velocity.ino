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
#define BNO055_SAMPLERATE_DELAY_MS (30)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/

float velocityX = 0, velocityY = 0; // Initial velocities
unsigned long prevTime = 0;         // To calculate time intervals
const float friction = 0.7;        // Friction factor to reduce drift
const float noiseThreshold = 0.1;  // Threshold to ignore noise

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
  prevTime = millis();
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/

float acc_x;
float acc_y;
float acc_z;
float sensitivity=10;



void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Get calibration status
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
  // Get time interval
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0; // Convert ms to seconds
  prevTime = currentTime;

  // Update velocity with friction applied
  velocityX = (velocityX + lin_acc.x() * deltaTime*50) * friction;
  velocityY = (velocityY + lin_acc.y() * deltaTime*50) * friction;

  // // Apply noise threshold
 if (abs(velocityX) < noiseThreshold) velocityX = 0;
 if (abs(velocityY) < noiseThreshold) velocityY = 0;

  // Clamp velocity to prevent excessive cursor speed
  //float clampedVelX = constrain(velocityX, -10, 10);
  //float clampedVelY = constrain(velocityY, -10, 10);

  // Move the mouse cursor
  const float sensitivity = 20;
  Mouse.move(-velocityX*sensitivity, velocityY*sensitivity);


  // Debugging: Print calibration and velocity data
  Serial.print("Calibration acc=");
  Serial.print(accel);
  Serial.print(" VelX=");
  Serial.print(velocityX*sensitivity);
  Serial.print(" VelY=");
  Serial.println(velocityY*sensitivity);

  //delay(10); // Adjust for smoother response

  /* Display the floating point data */
  // Serial.print("X: ");
  // Serial.print(lin_acc.x());
  // Serial.print(" Y: ");
  // Serial.print(lin_acc.y());
  // Serial.print(" Z: ");
  // Serial.print(lin_acc.z());
  // Serial.print("\t\t");
// acc_x=lin_acc.x();
// acc_y=lin_acc.y();
// acc_z=lin_acc.z();
// Serial.print(lin_acc.x());
// Serial.print("\t");
// Serial.print(lin_acc.y());
// Serial.print("\t");
// Serial.println(lin_acc.z());



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
 

  // delay(BNO055_SAMPLERATE_DELAY_MS);
  // Mouse.move(acc_x*sensitivity,acc_y*sensitivity,0);
}
