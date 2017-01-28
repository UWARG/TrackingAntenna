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
static Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Mag_Unified(COMPASS_ID);

//Declare accelerometer event variable
static sensors_event_t accelEvent;

//Initializes accelerometer struct
extern volatile AccelerometerData accelerometerData;

//Initializes accelerometer
bool initAccelerometer(){
 //Defines accel object based on compass id
 accel = Adafruit_LSM303_Accel_Unified(COMPASS_ID);

 //Initializes accelerometer
  if(!accel.begin()){
    error("\nAccelerometer failed to be detected.");
    return 0;
 }
 return 1;
}

//Retrieves accelerometer data
void getGravity(){
  //Gets accelerometer event data
  accel.getEvent(&accelEvent);
 
  //Assigns data to accelerometer struct variables
  accelerometerData.x = accelEvent.acceleration.x;
  accelerometerData.y = accelEvent.acceleration.y;
  accelerometerData.z = accelEvent.acceleration.z;
}
