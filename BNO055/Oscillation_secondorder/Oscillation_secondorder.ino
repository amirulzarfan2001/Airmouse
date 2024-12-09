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

// Sampling frequency (fixed for this example)
const float f_s = 100.0; // Sampling frequency in Hz

// State variables (use arrays to allow flexibility for multiple filters)
float x_prev[2] = {0.0, 0.0}; // Previous inputs
float y_prev[2] = {0.0, 0.0}; // Previous outputs

// Function to calculate second-order high-pass filter
float highPassFilter(float x, float f_c, float zeta) {
  // Pre-calculate coefficients based on f_c and zeta
  float K = 2 * f_s * tan(3.14159 * f_c / f_s);
  float d = 1 + 2 * zeta * K + K * K;

  float a0 = 1 / d;
  float a1 = -2 / d;
  float a2 = 1 / d;
  float b1 = 2 * (K * K - 1) / d;
  float b2 = (1 - 2 * zeta * K + K * K) / d;

  // Apply the filter equation
  float y = a0 * x + a1 * x_prev[0] + a2 * x_prev[1] - b1 * y_prev[0] - b2 * y_prev[1];

  // Update state variables
  x_prev[1] = x_prev[0];
  x_prev[0] = x;
  y_prev[1] = y_prev[0];
  y_prev[0] = y;

  return y; // Return the filtered value
}

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
float gyrox;
float gyroy;
float gyroz;
float velocityX = 0, velocityY = 0, velocityZ = 0; // Initial velocities
const float noiseThreshold = 0.15;  // Threshold to ignore noise
const unsigned long interval = 10; //f=100Hz // Time interval in milliseconds (e.g., 1000 ms = 1 second)
float deltaTime=0.01;
unsigned long prevTime = 0;

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




    //Apply noise threshold
if (abs(lin_accx) < noiseThreshold) lin_accx = 0;
if (abs(lin_accy) < noiseThreshold) lin_accy = 0;
if (abs(lin_accz) < noiseThreshold) lin_accz = 0;

  // Get time interval

//   if (abs(velocityX) < noiseThreshold) velocityX = 0;
// if (abs(velocityY) < noiseThreshold) velocityY = 0;
// if (abs(velocityZ) < noiseThreshold) velocityZ = 0;



  if( currentTime- prevTime>=interval){

  prevTime = currentTime;

    velocityX = (velocityX + lin_accx * deltaTime) ;
  velocityY = (velocityY + lin_accy* deltaTime) ;
  velocityZ = (velocityZ + lin_accz * deltaTime);

float y=highPassFilter(velocityX, 0.01, 1);
  //Mouse.move(y*100, 0.0);
Serial.print(y);
Serial.print(" ");  // Separate values with spaces
Serial.print(velocityX);
Serial.print(" ");
Serial.println(velocityZ);  // End with a newline

  }


}
