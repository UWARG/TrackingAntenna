/**
 * @file   Accelerometer.cpp
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
AccelerometerData accel_data;

//Initializes accelerometer
bool initAccelerometer(){
 //Defines accel object based on compass id
 accel = Adafruit_LSM303_Accel_Unified(ACCELEROMETER_ID);

 //Initializes accelerometer
  if(!accel.begin()){
    error("Accelerometer failed to be detected.");
    return 0;
 }
 return 1;
 debug("Accelerometer initialized succesfully");
}

//Retrieves accelerometer data. Values in m/s^2 for each direction
void parseAcceleration(){
  //Gets accelerometer event data
  accel.getEvent(&accelEvent);
 
  //Assigns data to accelerometer struct variables
  accel_data.x = accelEvent.acceleration.x;
  accel_data.y = accelEvent.acceleration.y;
  accel_data.z = accelEvent.acceleration.z;
}
