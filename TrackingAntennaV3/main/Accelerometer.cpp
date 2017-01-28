/**
 * @file   Magnetometer.cpp
 * @Author Michael Lenover (WARG)
 * @date   January 25, 2017
 */

#include "Logger.hpp"
#include "Accelerometer.hpp"
#include <Adafruit_LSM303_U.h>
#include <Math.h>

//Create accelerometer object
static Adafruit_LSM303_Accel_Unified accel;

//Declare accelerometer event variable
static sensors_event_t accelEvent;

//Initializes accelerometer struct
<<<<<<< HEAD
extern volatile AccelerometerData accelerometerData;
=======
extern volatile Accelerometer accelerometer;
>>>>>>> origin/Accelerometer

//Initializes accelerometer
bool initAccelerometer(){
 //Defines accel object based on compass id
 accel = Adafruit_LSM303_Accel_Unified(COMPASS_ID);

 //Initializes accelerometer
  if(!accel.begin()){
    error("\nAccelerometer failed to be detected.");
<<<<<<< HEAD
    return 0;
=======
>>>>>>> origin/Accelerometer
 }

 return 1;
}

//Retrieves accelerometer data
<<<<<<< HEAD
void getGravity(){
=======
void getGravity(float *pNegGravity){
>>>>>>> origin/Accelerometer
  //Gets accelerometer event data
  accel.getEvent(&accelEvent);
 
  //Assigns data to accelerometer struct variables
<<<<<<< HEAD
  accelerometerData.x = accelEvent.acceleration.x;
  accelerometerData.y = accelEvent.acceleration.y;
  accelerometerData.z = accelEvent.acceleration.z;
=======
  accelerometer.x = accelEvent.acceleration.x;
  accelerometer.y = accelEvent.acceleration.y;
  accelerometer.z = accelEvent.acceleration.z;
>>>>>>> origin/Accelerometer
}
