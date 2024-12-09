#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

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
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/

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

const unsigned long interval = 100; //f=100Hz // Time interval in milliseconds (e.g., 1000 ms = 1 second)
unsigned long prevTime = 0;
float deltaTime=0.1;

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
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);


if( currentTime- prevTime>=interval){

  prevTime = currentTime;

float x=euler.x();
 float t=x;
    // Apply the high-pass filter
  float y = alpha * (y_prev + t - x_prev);
    // Update previous values
  x_prev = t;
  y_prev = y;
  /* Display the floating point data */
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(euler.x());
  Serial.print(" ");
  Serial.println(euler.z());
  //Serial.print("\t\t");
}

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
}
