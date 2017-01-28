#include "Logger.hpp"
#include "Magnetometer.hpp"
#include "Util.hpp"
#include <Adafruit_LSM303_U.h>
#include <MatrixMath.h> //Used for magnetometer calibration
#include <MatrixMath.h>

//Create magnetometer object
static Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(COMPASS_ID);

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
  }
}

//Retrieves and calibrates magnetic north from magentometer
void getMagneticNorth(){
  
  //Initialize temporary variable for vector storage
  float mag_north_comp[3];
  
  //Get magnetic north from magnetometer
  mag.getEvent(&magEvent);
  
  //Assign raw magnetic north data to array
  mag_north_comp[0] = magEvent.magnetic.x;
  mag_north_comp[1] = magEvent.magnetic.y;
  mag_north_comp[2] = magEvent.magnetic.z;
  
  #if CalibrateMagnetometer //**MAY BE REMOVED IN FUTURE**
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
  #endif

  magnetometer.angle = vectorToAngle(magnetometer.x, magnetometer.y);
}
