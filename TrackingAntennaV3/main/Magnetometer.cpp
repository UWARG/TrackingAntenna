/**
 * @file   Magnetometer.cpp
 * @Author Michael Lenover (WARG)
 * @date   January 5, 2017
 */

#include "Logger.hpp"
#include "Magnetometer.hpp"
#include "Util.hpp"
#include <Adafruit_LSM303_U.h>
#include <MatrixMath.h> //Used for magnetometer calibration
#include <Math.h> //Used for angle calculation in getMagneticNorth()

//Create magnetometer object
static Adafruit_LSM303_Mag_Unified mag;

//Declare magnetometer event variable
static sensors_event_t magEvent;


//Initializes magnetometer struct
MagnetometerData mag_data;

//Initialize magnetometer. Returns true if successful, false otherwise
bool initMagnetometer(){
  //Defines mag object based on compass id
  mag = Adafruit_LSM303_Mag_Unified(MAGNETOMETER_ID);
  
  //Enables auto-range
  mag.enableAutoRange(true);
  
  //Initializes magnetometer
  if(!mag.begin()){
    //If initialization fails, send error message
    error("Compass failed to be detected.");
    return false;
  }
  info("Compass initialized successfully");
  return true;
}

//Calibrate magnetometer values to compensate for soft iron in tracking antenna
void calibrateMagnetometer(float *mag_north_comp){
  //Create temporary array for partially calibrated magnetometer data
  float raw_minus_bias[3];

  //Define soft iron transform matrix for calibration
  float soft_iron_transform[3][3];
  
  soft_iron_transform[0][0] = 1.0717;
  soft_iron_transform[0][1] = -0.0171;
  soft_iron_transform[0][2] = -0.0112;
  soft_iron_transform[1][0] = -0.0171;
  soft_iron_transform[1][1] = 1.0714;
  soft_iron_transform[1][2] = 0.0029;
  soft_iron_transform[2][0] = -0.0112;
  soft_iron_transform[2][1] = 0.0029;
  soft_iron_transform[2][2] = 1.0554;
  Matrix.Invert((float*)soft_iron_transform, 3);
  
  //Define bias vector for calibration
  float bias_vec[3];
  
  bias_vec[0] = 9.6778;
  bias_vec[1] = 2.1963;
  bias_vec[2] = 3.4728;
  
  //Calibrate magnetic north data
  Matrix.Subtract((float*)mag_north_comp, (float*)bias_vec, 3, 1, (float*)raw_minus_bias);
  Matrix.Multiply((float*)soft_iron_transform, (float*)raw_minus_bias, 3, 3, 1, (float*)mag_north_comp);
}

//Retrieves magnetic north vector from magentometer. Values in micro Teslas for each direction.
void getMagneticNorth(){
  float mag_north_comp[3];
  //Initialize temporary variable for vector storage
  
  //Get magnetic north from magnetometer
  mag.getEvent(&magEvent);
  
  //Assign raw magnetic north data to array
  mag_north_comp[0] = magEvent.magnetic.x;
  mag_north_comp[1] = magEvent.magnetic.y;
  mag_north_comp[2] = magEvent.magnetic.z;
  
  #if CALIBRATE_MAGNETOMETER //**MAY BE REMOVED IN FUTURE**
  calibrateMagnetometer((float*)mag_north_comp);
  #endif
  
  mag_data.x = mag_north_comp[0];
  mag_data.y = mag_north_comp[1];
  mag_data.z = mag_north_comp[2];
  mag_data.heading = degrees(atan2(mag_data.y, mag_data.x));
}

