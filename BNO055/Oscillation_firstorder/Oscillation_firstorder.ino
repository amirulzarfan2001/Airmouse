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
Mouse.begin();
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
float lin_accx;
float lin_accy;
float lin_accz;
float damp_accx;
float damp_accy;
float damp_accz;
float sensitivity=20;
const float noiseThreshold = 0.15;  // Threshold to ignore noise

const float friction = 1;        // Friction factor to reduce drift
float int_sensitivity= 50;
const unsigned long interval = 1; //f=100Hz // Time interval in milliseconds (e.g., 1000 ms = 1 second)
unsigned long prevTime = 0;
float deltaTime=0.001;
int i=0;
float velocityX = 0, velocityY = 0, velocityZ = 0; // Initial velocities

// float alpha = 0.8;  // Complementary filter weight

float accel_velocity = 0, gyro_velocity = 0;
float velocity_filtered = 0;

float gyrox;
float gyroy;
float gyroz;

 float curr_x;
 float prev_x;

// Define simulation parameters
const float f_s = 1000.0; // Sampling frequency in Hz
const float f_c = 10;   // Cutoff frequency in Hz

// Calculate filter parameter alpha
const float RC = 1.0 / (2 * 3.14159 * f_c);
const float dt = 1.0 / f_s; // Sampling interval
const float alpha = RC / (RC + dt);

// Variables for filtering
float x_prev = 0.0; // Previous input
float y_prev = 0.0; // Previous output

void loop(void)
{
    unsigned long currentTime = millis();
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
 // Get linear acceleration

   uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  // Ensure system calibration is adequate
  if (gyro < 3) {
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
  imu::Vector<3> lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  lin_accx=-lin_acc.x();
  lin_accy=lin_acc.y();
  lin_accz=lin_acc.z();
  imu::Vector<3> gyro_r = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  gyrox=gyro_r.x();
  gyroy=gyro_r.y();
  gyroz=gyro_r.z();

    // float linear_accel = lin_accx; // Get linear acceleration (m/s^2)
    // float gyro_rate = gyrox;        // Get gyroscope rate (deg/s)

    // // Integrate acceleration for velocity
    // accel_velocity =(accel_velocity+ linear_accel * dt)*friction;

    // // Calculate gyroscope-based velocity adjustment (optional)
    // gyro_velocity =(gyro_rate+ gyro_rate * dt)*friction; 

    // // Apply complementary filter
    // velocity_filtered = alpha * (velocity_filtered + linear_accel * dt) + (1 - alpha) * gyro_velocity;
    // Mouse.move(velocity_filtered*5,0.0);
    // Serial.println(velocity_filtered);

// Serial.print(velocityX);
// Serial.print(" ");  // Separate values with spaces
// Serial.print(velocityY);
// Serial.print(" ");
// Serial.println(velocityZ);  // End with a newline


    //Apply noise threshold
if (abs(lin_accx) < noiseThreshold) lin_accx = 0;
if (abs(lin_accy) < noiseThreshold) lin_accy = 0;
if (abs(lin_accz) < noiseThreshold) lin_accz = 0;

  // Get time interval

//   if (abs(velocityX) < noiseThreshold) velocityX = 0;
// if (abs(velocityY) < noiseThreshold) velocityY = 0;
// if (abs(velocityZ) < noiseThreshold) velocityZ = 0;

//   //Serial.println(t,10);
//   // Update velocity with friction applied

  if( currentTime- prevTime>=interval){

  prevTime = currentTime;
    //   i+=1;
    // Serial.println("Task executed");
    // Serial.println(i);
  //Serial.println(t,10);
    velocityX = (velocityX + lin_accx * deltaTime) ;
  velocityY = (velocityY + lin_accy* deltaTime) ;
  velocityZ = (velocityZ + lin_accz * deltaTime);
  // float velocity_fricx=velocityX;
  // float velocity_fricy=velocityY;
  // velocity_fricx=velocity_fricx*0.95;
  // velocity_fricy=velocity_fricy*0.95;
 float x=velocityX;
    // Apply the high-pass filter
  float y = alpha * (y_prev + x - x_prev);
    // Update previous values
  x_prev = x;
  y_prev = y;
  Mouse.move(y*100, 0.0);
Serial.print(y*100);
Serial.print(" ");  // Separate values with spaces
Serial.print(velocityX);
Serial.print(" ");
Serial.println(velocityZ);  // End with a newline
//Mouse.move(velocity_fricx*3, velocity_fricy*3);
  }


// velocity_filtered = alpha * (velocityX) + (1 - alpha) * -gyroz*0.2;
//Mouse.move(velocity_filtered*sensitivity, 0.0);

//Mouse.move(velocityX*sensitivity, velocityY*sensitivity);
 // Mouse.move(-lin_accx*sensitivity, lin_accy*sensitivity);
// Serial.print(abs(lin_accx));
// Serial.print(" ");  // Separate values with spaces
// Serial.print(abs(lin_accy));
// Serial.print(" ");
// Serial.println(abs(lin_accz));  // End with a newline

// Serial.print(velocityX*sensitivity);
// Serial.print(" ");  // Separate values with spaces
// Serial.print(velocityY*sensitivity);
// Serial.print(" ");
// Serial.println(velocityZ*sensitivity);  // End with a newline

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
//  // uint8_t system, gyro, accel, mag = 0;
//   bno.getCalibration(&system, &gyro, &accel, &mag);
//   Serial.print("CALIBRATION: Sys=");
//   Serial.print(system, DEC);
//   Serial.print(" Gyro=");
//   Serial.print(gyro, DEC);
//   Serial.print(" Accel=");
//   Serial.print(accel, DEC);
//   Serial.print(" Mag=");
//   Serial.println(mag, DEC);


}
