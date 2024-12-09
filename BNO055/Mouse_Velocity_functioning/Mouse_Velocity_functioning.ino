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
float reset_to_zero(float current_data, float previous_data, float threshold, unsigned long *stable_time, unsigned long reset_duration) {
    // Check if the data is stable within the threshold
    if (fabs(current_data - previous_data) < threshold) {
        if (*stable_time == 0) {
            // Start the stability timer if it's not already running
            *stable_time = millis();
        } else if (millis() - *stable_time > reset_duration) {
            // If the data is stable for the reset duration, reset to zero
            return 0.0;
        }
    } else {
        // Reset the stability timer if the data changes
        *stable_time = 0;
    }

    // Return the current data if no reset is performed
    return current_data;
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
const float noiseThreshold = 0.01;  // Threshold to ignore noise
const float noiseThreshold_ac = 0.4;  // Threshold to ignore noise
const unsigned long interval = 100; //f=100Hz // Time interval in milliseconds (e.g., 1000 ms = 1 second)
float deltaTime=0.1;
unsigned long prevTime = 0;

unsigned long stable_time = 0;
unsigned long reset_duration = 10; // Time (ms) to wait before resetting (e.g., 2 seconds)
float x;
float currentdata;

// Define simulation parameters
const float f_s = 10.0; // Sampling frequency in Hz
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
    float previous_data=velocityX;
    float previous_dataY=velocityY;
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
// if (abs(lin_accx) < noiseThreshold_ac) lin_accx = 0;
// if (abs(lin_accy) < noiseThreshold_ac) lin_accy = 0;
// if (abs(lin_accz) < noiseThreshold_ac) lin_accz = 0;
//lin_accx=lin_accx*10;

  // Get time interval

//   if (abs(velocityX) < noiseThreshold) velocityX = 0;
// if (abs(velocityY) < noiseThreshold) velocityY = 0;
// if (abs(velocityZ) < noiseThreshold) velocityZ = 0;



  if( currentTime- prevTime>=interval){

  prevTime = currentTime;


  velocityX = (velocityX + y * deltaTime) ;
  velocityY = (velocityY + lin_accy* deltaTime);
  velocityZ = (velocityZ + lin_accz * deltaTime);
  // if (abs(velocityX) < noiseThreshold) velocityX = 0;
  // if (abs(velocityY) < noiseThreshold) velocityY = 0;
  // if (abs(velocityZ) < noiseThreshold) velocityZ = 0;
   float t=velocityX;
    // Apply the high-pass filter
  float y = alpha * (y_prev + t - x_prev);
    Update previous values
  x_prev = t;
  y_prev = y;
  
unsigned long reset_duration = 100; // Time (ms) to wait before resetting (e.g., 2 seconds)
//  velocityX= reset_to_zero(velocityX, previous_data, 0.01, &stable_time, reset_duration);
//   velocityY= reset_to_zero(velocityY, previous_dataY, 0.01, &stable_time, reset_duration);

float sense=1;
  //Mouse.move(velocityX*sense, velocityY*sense);
Serial.print(velocityX*10);
Serial.print(" ");  // Separate values with spaces
Serial.print(y);
Serial.print(" ");
Serial.println(0.2);  // End with a newline

  }




}