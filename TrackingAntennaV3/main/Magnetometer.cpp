#include "Logger.hpp"
#include "Magnetometer.hpp"
#include <Adafruit_LSM303_U.h>
#include <MatrixMath.h> //Used for magnetometer calibration
#include <MatrixMath.h>

//Initializes magnetometer object
static Adafruit_LSM303_Mag_Unified mag; 
//Create magnetometer object
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(COMPASS_ID);

//Declare magnetometer event variable
static sensors_event_t magEvent;

//Initializes magnetometer struct
volatile Magnetometer magnetometer;
sensors_event_t magEvent;

//Initialize magnetometer
void initMagnetometer(){
  //Defines mag object based on compass id
  mag = Adafruit_LSM303_Mag_Unified(COMPASS_ID);

  
  //Enables auto-range
  mag.enableAutoRange(true);
  
  //Initializes magnetometer
  //Initializes magnetomer
  if(!mag.begin()){
      //If initialization fails, send error message
      error("Compass failed to be detected.");
@@ -39,10 +34,7 @@ void initMagnetometer(){


//Retrieves and calibrates magnetic north from magentometer
void getMagneticNorth(){
  
  //Initialize compass vector array
  float mag_north_comp[3];
void getMagneticNorth(float *mag_north_comp){
  
  //Get magnetic north from magnetometer
  mag.getEvent(&magEvent);
@@ -53,7 +45,6 @@ void getMagneticNorth(){
  mag_north_comp[2] = magEvent.magnetic.z;
  
  #if CalibrateMagnetometer //**MAY BE REMOVED IN FUTURE**
  
  //Create temporary array for partially calibrated magnetometer data
  float raw_minus_bias[3];

@@ -81,13 +72,7 @@ void getMagneticNorth(){
  //Calibrate magnetic north data
  Matrix.Subtract((float*)mag_north_comp, (float*)bias_vec, 3, 1, (float*)raw_minus_bias);
  Matrix.Multiply((float*)soft_iron_transform, (float*)raw_minus_bias, 3, 3, 1, (float*)mag_north_comp);

  #endif
  
  //Assigns data to magnetometer struct variables
  magnetometer.x = mag_north_comp[0];
  magnetometer.y = mag_north_comp[1];
  magnetometer.z = mag_north_comp[2];
}


