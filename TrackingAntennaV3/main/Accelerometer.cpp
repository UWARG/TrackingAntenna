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
static Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(COMPASS_ID);

//Declare accelerometer event variable
static sensors_event_t accelEvent;

//Initializes accelerometer struct
volatile AccelerometerData accelerometer_data;

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

//Retrieves accelerometer data. Values range from -10 to 10 for each vector component.
void getGravity(){
  //Gets accelerometer event data
  accel.getEvent(&accelEvent);
 
  //Assigns data to accelerometer struct variables
  accelerometer_data.x = accelEvent.acceleration.x;
  accelerometer_data.y = accelEvent.acceleration.y;
  accelerometer_data.z = accelEvent.acceleration.z;
}
