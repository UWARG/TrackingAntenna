/**
 * @file   Magnetometer.cpp
 * @Author Michael Lenover (WARG)
 * @date   January 25, 2017
 */

#include "Logger.hpp"
#include "Accelerometer.hpp"
#include <Adafruit_LSM303_U.h>

//Create accelerometer object
static Adafruit_LSM303_Accel_Unified accel;

//Declare accelerometer event variable
static sensors_event_t accelEvent;

//Initializes accelerometer struct
volatile Accelerometer accelerometer;

//Initializes accelerometer
void initAccelerometer(){
 //Defines accel object based on compass id
 accel = Adafruit_LSM303_Accel_Unified(COMPASS_ID);

 //Initializes accelerometer
 if(!accel.begin()){
   error("\nAccelerometer failed to be detected.");
 }
}

//Retrieves accelerometer data
void getGravity(float *pNegGravity){
 //Gets accelerometer event data
 accel.getEvent(&accelEvent);
 
 //Assigns data to accelerometer struct variables
 accelerometer.x = accelEvent.acceleration.x;
 accelerometer.y = accelEvent.acceleration.y;
 accelerometer.z = accelEvent.acceleration.z;
}

