/*

 This sketch connects to a a telnet server (http://www.google.com)
 using an Arduino Wiznet Ethernet shield.  You'll need a telnet server 
 to test this with.

 Circuit:
 * Ethernet shield attached to pins 10, 11, 12, 13
 
  servoPan is the pan servo, pwm to pin 5 (yz) .
  servoTilt is the tilt servo, pwm6 (xz).


Servo Information: 
Model No. HITEC HS-785HB
    
    Information taken from: http://www.robotshop.com/ca/en/hitec-hs785hb-servo-motor.html IS INCORRECT
    
    Motor does not rotate according to specs, ref:https://www.servocity.com/html/hs-785hb_3_5_rotations.html (watch the video)

    using servo.attach(pin, 600, 2400):
    ***These values have been empirically obtained for the servo that is mounted in the pan position
    "full" left (610 PWM) = 0 deg
    midpoint (1500 PWM) = 1414 deg
    "full" right (2390 PWM) = 2787 deg

    ***used PWM just shy of limits because the limits put the motor in continuous rotation mode, rather than positional mode
    Therefore:
    Deg --> PWM = (2390 - 610)PWM / 2787 deg = 0.63868 PWM/deg
    PWM --> deg = 1.56573 deg/PWM

Magnetometer(Compass) Information:
Model No. ST LSM303DLHC
  
    Information taken from: https://www.adafruit.com/products/1120
  
    Communication between compass and board is over i2c.
    All readings are in micro-Teslas
    Compass attached to pins: SCL, SDA, 3.3V, GND
  
    Note: Compass points to magnetic north, not true north

    Calibration data for compass (not general, specific to one particular unit):
    
    Accel Minimums (X, Y, Z): -11.53, -11.69, -10.83
    Accel Maximums (X, Y, Z): 10.75, 11.89, 17.10
    Mag Minimums (X, Y, Z): -46.73, -54.73, -51.94
    Mag Maximums (X, Y, Z): 64.91, 57.36, 59.08

*/

//-----------------------------------LIBRARIES----------------------------

#include <SPI.h>
#include <Ethernet.h>
#include <math.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <MatrixMath.h>

//Adafruit Libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h> 

//-----------------------------------DEFINITIONS----------------------------

#define M_PI 3.14159265358979323
#define SERVO_MIDPOINT 1500
#define SERVO_INITIALIZE 0              //Used to set servo motors to their mid point
#define EARTH_RADIUS 6371000            //in m
#define DEGREE_TO_PWM 0.6725            //Conversion factor of degrees to PWM
#define GEAR_RATIO_XZ 7                 //Gear ratio used to reduce angular range in which motor operates
#define GEAR_RATIO_XY 1
#define COMPASS_ID 12345
#define GRAVITY_CONSTANT 9.80665
#define Size 3                          //Size of square matrices and length of vectors
#define DEBUG false                     //When "true" turns on the printing to serial for testing

//Define GPS coordinates of antenna at competition (reference), use GPS app on phone while on site to locate
//#define latORG 0                        //Manually enter South Port Latitude
//#define lonORG 0                        //Manually enter South Port Longitude
#define latORG 43.5311386               //Latitude of antenna at Flying Dutchmen flight center
#define lonORG -80.5771163              //Longitude of antenna at Flying Dutchmen flight center

//Manual calibration booleans and offsets
//If NorthCalibrate = true, it must be calibrated first, then "TrueSouth" value can be read colinear with calibrated True North
#define NorthCalibrate false             //Indicates when north direction must be manually set ("true" if setting manually, "false" if compass is reading True North correctly)
#define SouthCalibrate false             //Indicates when compass is calibrated improperly, use manually input south angle to calibrate ("true" if setting manually, "false" if compass is reading South colinear with True North)
#define NorthOffset 7                   //Added to "HeadingTrue" to point compass at True North, used if "NorthCalibrate" = true (if compass is reading North as West of True North, then NorthOffset > 0, opposite for East)
#define TrueSouth 180                   //Angle given by compass when pointing to True South, used if "SouthCalibrate" = true ("TrueSouth" equals reading taken colinear with calibrated True North reading)

////Define calibration values for magnetometer, for old sensor
//#define MinMag_X -48.36
//#define MinMag_Y -55.73
//#define MinMag_Z -54.08
//#define MaxMag_X 66.36
//#define MaxMag_Y 60.73
//#define MaxMag_Z 75.20

//Define calibration values for magnetometer, for new sensor
#define MinMag_X -53.0909
#define MinMag_Y -59.7273
#define MinMag_Z -93.2653
#define MaxMag_X 61.4545
#define MaxMag_Y 54.2727
#define MaxMag_Z 26.0204

//-----------------------------------GLOBAL VARIABLES----------------------------

//Initialization Variables
boolean L1 = true;                        //Line 1 marker, use comman count to assign commaCnt values to pertinent headings
boolean InitializeAntenna = true;         //Used to control for loop for locating antenna's GPS coordinates                 DO NOT USE UNLESS SPI CONNECTION IS WORKING PROPERLY
int ConnectionStatus = 0;                 //Used to identify connection status of ethernet, for connection failure error messages

//Counts to describe the positions of delimiters in incoming ground station data and number of data packages received
int commaCnt = 0;
int lineCnt = 0;

//Comma positions of important data
int latCnt = 0;
int longCnt = 0;
int altitudeCnt = 0;

//String objects used to hold incoming values from ground station
String Data = "";
String Column = "";
const char *buff = Column.c_str(); //Uses pointer "buff" to continuously assign new data to a string in Column

//Angles
int ThetaPanRef = 0;                //Pan reference angle, used in servo functions to determine angular difference in current positions of antenna and plane
int ThetaTiltRef = 0;               //Tilt reference angle, used in servo functions to determine angular difference in current positions of antenna and plane
int ThetaPan = 0;                   //Difference in pan angle between current position of plane and mid-point angle of servo
int ThetaTilt = 0;                  //Difference in tilt angle between current position of plane and mid-point angle of servo
int LastThetaPan;                   //Records the last pan input angle for determining whether compass crosses south axis

unsigned long servoPanLastWrite = 0;
unsigned long servoTiltLastWrite = 0;

//Latitude, Longitude and Altitude variables
long double lat = 0;                //GPS latitude of plane
long double lon = 0;                //GPS longitude of plane
long double altitude = 0;           //GPS altitude of plane

/***      COMPASS & ACCELEROMETER     ***/
//Declare sensor variables
sensors_event_t accelEvent;         //Accelerometer Data
sensors_event_t magEvent;           //Magnetic Compass Data

//Declination values: South Port Manitoba on 04/30/2016 = 4.136 deg E, Waterloo Ontario on 03/09/2016 = 9.633deg W, Flying Dutchmen flight center on 04/10/2016 = -9.624 W
//East declinations are POSITIVE, West declinations are NEGATIVE
float declination = -9.624;

//Declare vectors for sensor calculations
//float Gravity[Size];
float NegGravity[Size];            //Vertical direction (negative of gravity)
float MagNorthComp[Size];           //Magnetic north in compass frame of reference
float HeadingAnt[Size];             //Heading of antenna in horizontal plane of inertial frame of reference, colinear with x-axis of compass projected into horizontal inertial plane
float UnitInertX[Size];             //X-Axis in inertial frame of reference
float UnitInertY[Size];             //Y-Axis in inertial frame of reference
float UnitInertZ[Size];             //Z-Axis in inertial frame of reference
//Change of basis matrix for converting vectors from compass frame of reference to inertial frame of reference
//Multiply compass frame of reference vector on left
float ChangeBasis[Size][Size];
//Change of basis matrix for converting vectors from inertial frame of reference to compass frame of reference
//Multiply inertial frame of reference vector on left
float RevertBasis[Size][Size];
float SoftIronTransform[Size][Size];
float BiasVec[Size];

//--------------------------------------------------------Matrix Functions---------------------------------------------------------------

//Multiplication by a scalar
void ScalarMult(float *pMatA, float constant, int m, int n, float *pMatRes){
  int i, j;
  
  for (i = 0; i < m; i++){
    for (j = 0; j < n; j++){
      pMatRes[n * i + j] = constant * pMatA[n * i + j];
    }
  }
}

//Dot Product
float DotProduct(float *pVecA, float *pVecB, int SizeA, int SizeB){
  int i;
  
  if(SizeA == SizeB){
    float result = 0;
    
    for (i=0; i<SizeA; i++){
      result += pVecA[i] * pVecB[i];
    }
    return result;
  }
}

//Dot Product Projection, projects vector B onto vector A
float DotProductProject(float *pVecA, float *pVecB, int SizeA, int SizeB, float *pProj){  
  if(SizeA == SizeB){
    float DotAB = (DotProduct((float*)pVecA, (float*)pVecB, SizeA, SizeA));
    float MagnitudeA_SQRD = (DotProduct((float*)pVecA, (float*)pVecA, SizeA, SizeA));
    float Quotient = DotAB / MagnitudeA_SQRD;
    ScalarMult((float*)pVecA, Quotient, SizeA, 1, (float*)pProj);
  }
}

//Dot Angle, calculates the angle between two vectors
float DotProductAngle(float *pVecA, float *pVecB, int SizeA, int SizeB){
  float Dot = DotProduct((float*)pVecA, (float*)pVecB, SizeA, SizeA);
  float MagnitudeA = sqrt(DotProduct((float*)pVecA, (float*)pVecA, SizeA, SizeA));
  float MagnitudeB = sqrt(DotProduct((float*)pVecB, (float*)pVecB, SizeA, SizeA));
  float Theta = acos(Dot / (MagnitudeA * MagnitudeB)) * 180 / M_PI;
  
  return Theta;
}

//Cross Product
void CrossProduct_1x3(float *pVecA, float *pVecB, int SizeA, int SizeB, float *resultVec){
  int i, j;
  resultVec[SizeA];

  if(SizeA == SizeB && SizeA == 3){
    j = 1;
    
    for (i=0; i<SizeA; i++){
      if(j == 2 && i == 1){
        resultVec[i] = pVecA[j]*pVecB[j-2] - pVecA[j-2]*pVecB[j];
        j = 0;
      }
      else{
        resultVec[i] = pVecA[j]*pVecB[j+1] - pVecA[j+1]*pVecB[j];
        j++;
      }
    }
  }
}

//Project vector onto 2D plane in Rn                                            //METHOD FROM http://people.ucsc.edu/~lewis/Math140/Ortho_projections.pdf --> DOES NOT WORK AS WELL AS METHOD BELOW
//void ProjectVecToPlane(float *pVec, float *pBase1, float *pBase2, int Rn, float *pProj){
//  float Base1Scaled[Rn];
//  float Base2Scaled[Rn];
//  
//  float Dot1 = DotProduct((float*)pVec, (float*)pBase1, Rn, Rn);
//  float Dot2 = DotProduct((float*)pVec, (float*)pBase2, Rn, Rn);
//  ScalarMult((float*)pBase1, Dot1, Rn, 1, (float*)Base1Scaled);
//  ScalarMult((float*)pBase2, Dot2, Rn, 1, (float*)Base2Scaled);
//  
//  Matrix.Add((float*)Base1Scaled, (float*)Base2Scaled, Rn, Rn, (float*)pProj);
//}

//Project vector in Rn onto 2D plane --> Projection = A * [A * A(tr)](inv) * A(tr) * Vec
//Where A is a column matrix representing the span of the plane, (tr) denotes transpose, (inv) denotes inverse, Vec is the vector to be projected
void ProjectVecToPlane(float *pVec, float *pBase1, float *pBase2, int Rn, float *pProjVec){
  int i, j;
  float Matrix_A[Rn][2];
  float Matrix_ATr[2][Rn];
  float Matrix_ATrxA[2][2];
  float Product_AxATrxA[Rn][2];
  float ProjectionMatrix[Rn][Rn];

  //Construct A from two linearly independent basis vectors
  for(i=0; i<Rn ; i++){
    Matrix_A[i][0] = pBase1[i];
  }
  for(j=0; j<Rn ; j++){
    Matrix_A[j][1] = pBase2[j];
  }

//  if(DEBUG){
//    Matrix.Print((float*)Matrix_A, Rn, 2, "Column Matrix A");
//  }
  
  //Calculate projection matrix
  Matrix.Transpose((float*)Matrix_A, Rn, 2, (float*)Matrix_ATr);
  Matrix.Multiply((float*)Matrix_ATr, (float*)Matrix_A, 2, Rn, 2, (float*)Matrix_ATrxA);
  Matrix.Invert((float*)Matrix_ATrxA, 2);
//  Matrix.Print((float*)Matrix_ATrxA, 2, 2, "Inverted A^T*A");
  Matrix.Multiply((float*)Matrix_A, (float*)Matrix_ATrxA, Rn, 2, 2, (float*)Product_AxATrxA);
  Matrix.Multiply((float*)Product_AxATrxA, (float*)Matrix_ATr, Rn, 2, Rn, (float*)ProjectionMatrix);
//  Matrix.Print((float*)ProjectionMatrix, Rn, Rn, "Projection Matrix");

  //Multiply vector on left by projection matrix to produce projected result
  Matrix.Multiply((float*)ProjectionMatrix, (float*)pVec, Rn, Rn, 1, (float*)pProjVec);
}

//Normalize vectors (make length == 1)
void Normalize(float *pVec, int VecSize, float *pNorm){
  int i;

  for(i=0; i<VecSize; i++){ 
      pNorm[i] = pVec[i] / sqrt(DotProduct((float*)pVec, (float*)pVec, VecSize, VecSize));
  }
}

//-----------------------------------INITIALIZATION VALUES----------------------------

// Enter a MAC address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {0x90,0xA2,0xDA,0x0F,0x2C,0x9A};
IPAddress ip(192,168,1,199); //this is for the arduino

// Enter the IP address of the server you're connecting to:
IPAddress server(192,168,1,101);  //pi or computer that is hosting the data

/****        END OF INITIALIZATION      **********/

// Initialize the Ethernet client library
// with the IP address and port of the server 
// that you want to connect to (port 23 is default for telnet;
// if you're using Processing's ChatServer, use  port 10002):
EthernetClient client;

//Servo Objects
Servo servoPan;   // create servo object to control a servo 
Servo servoTilt;  // create servo object to control a servo 

//Initializes the compass library with an arbitrarily set compass id
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(COMPASS_ID);          //Create compass object to read magnetic values
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(COMPASS_ID);    //Create compass object to read acceleration values

void setup(){  
  //Attaches the servos
  //usage: servo.attach(pin, minPWM, maxPWM)
  servoPan.attach(5, 600, 2400);
  servoTilt.attach(6, 600, 2400);
    
   /* Enable auto-gain */
  mag.enableAutoRange(true);
  
  //Initialize the compass
  if(!mag.begin()){
    Serial.println("\nCompass failed to be detected.");
  }
  //Initialize the accelerometer
  if(!accel.begin()){
    Serial.println("\nAccelerometer failed to be detected.");
  }

  //Initialize Ethernet communication
  Ethernet.begin(mac, ip);
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
   while (!Serial) {
     // wait for serial port to connect. Needed for Leonardo only
  }

  //Give the Ethernet shield a second to initialize:
  delay(1000);
  Serial.println("connecting...");
  ConnectionStatus = client.connect(server, 1234);

  // if you get a connection, report back via serial:
  if (ConnectionStatus) {
    Serial.println("connected");
  }
  else {  // if you didn't get a connection to the server:
    Serial.println("connection failed");
    if(ConnectionStatus == -1){
      Serial.println("connection timed out");
    }
    else if(ConnectionStatus == -2){
      Serial.println("invalid server");
    }
    else if(ConnectionStatus == -3){
      Serial.println("truncated");
    }
    else if(ConnectionStatus == -4){
      Serial.println("invalid response");
    }
    Serial.print("ConnectionStatus: "); Serial.println(ConnectionStatus);
  }

  //Set up soft iron matrix and bias vector for calibration of magnetometer data
  //Define soft iron transform matrix
  SoftIronTransform[0][0] = 1.0717;
  SoftIronTransform[0][1] = -0.0171;
  SoftIronTransform[0][2] = -0.0112;
  SoftIronTransform[1][0] = -0.0171;
  SoftIronTransform[1][1] = 1.0714;
  SoftIronTransform[1][2] = 0.0029;
  SoftIronTransform[2][0] = -0.0112;
  SoftIronTransform[2][1] = 0.0029;
  SoftIronTransform[2][2] = 1.0554;
  Matrix.Invert((float*)SoftIronTransform, 3);
  
  //Define bias vector
  BiasVec[0] = 9.6778;
  BiasVec[1] = 2.1963;
  BiasVec[2] = 3.4728;
}

// ---------------------------------GPS FUNCTIONS--------------------------------------

//Calculates distance between two points (latitude and longitude), in meters
long double getDistance(long double Lat1, long double Lon1, long double Lat2, long double Lon2){ 

  long double distance = 0;
  long double disLat = (Lat2 - Lat1) * M_PI / 180;
  long double disLon = (Lon2 - Lon1) * M_PI / 180;

  //"Haversine" formula for calculating distance between two points given latitude and longitude
  long double a = sin(disLat / 2) * sin(disLat / 2) + cos(Lat1 * M_PI / 180) * cos(Lat2 * M_PI / 180) * sin(disLon / 2) * sin(disLon / 2);
  distance = EARTH_RADIUS * (2 * atan2(sqrt(a),sqrt(1 - a)));

  if(Lat1 > Lat2 && Lon2 == Lon1){
    distance = distance * -1;
  }
  if(Lon1 > Lon2 && Lat2 == Lat1){
    distance = distance * -1;
  }

  return distance;
}

//--------------------------------------------------------Servo Functions---------------------------------------------------------------

//Calculate angle between air plane and true north
float GetThetaXY(long double lonDiff, long double latDiff){
  float ThetaXY = degrees(atan2(lonDiff, latDiff));

  return ThetaXY; 
}

////Calcuated angle of plane off of horizontal                                                              DON'T USE IF ALTITUDE GIVEN TAKES INTO ACCOUNT ANTENNA ALTITUDE
//float GetThetaXZ(long double distance, long double height, long double altitudeAnt){
//  long double altDiff = height - altitudeAnt;
//  float ThetaXZ = degrees(atan2(altDiff, distance));
//  
//  return ThetaXZ;
//}

//Calcuated angle of air plane off of horizontal
//Height given is distance from ground, incoming altitude data accounts for altitude of location
float GetThetaXZ(long double distance, long double height){
  float ThetaXZ = degrees(atan2(height, distance));  
  return ThetaXZ;
}

//Set servo to mid-point and calculate tilt angle
float SetTiltOffset(float *pUnitInertX, float *pUnitInertY){
  const float CompassX[Size] = {1, 0, 0};
  float ProjCompassX[Size];
  float CheckDir[Size];
  int ThetaInitial;
  
  ProjectVecToPlane((float*)CompassX, (float*)pUnitInertX, (float*)pUnitInertY, Size, (float*)ProjCompassX);    //Project compass x-axis into horizontal plane
  ThetaInitial = DotProductAngle((float*)CompassX, (float*)ProjCompassX, Size, Size);

  //Check direction of ThetaInitial
  CrossProduct_1x3((float*)CompassX, (float*)ProjCompassX, Size, Size, (float*)CheckDir);
  if(CheckDir[1] < 0){
    ThetaInitial = -1 * ThetaInitial;
  }
  
//  if(DEBUG){
//  Matrix.Print((float*)CompassX, Size, 1, "Compass X-Axis");
//  Matrix.Print((float*)ProjCompassX, Size, 1, "Antenna Heading");
//  Matrix.Print((float*)CheckDir, Size, 1, "CheckDir Vector");
//  }
  
  return ThetaInitial;
}

//Sets servo motor to position by converting input angle into PWM
void SetPan(int Theta){
  //Account for rollover condition, pan motion should be continuous when transitioning from -180-->180 or vice versa (due south)
  if(LastThetaPan < -120 && Theta > 0){
    LastThetaPan = Theta - 360;
  }
  else if(LastThetaPan > 120 && Theta < 0){
    LastThetaPan = Theta + 360;
  }
  else{
    LastThetaPan = Theta;
  }
  
  servoPanLastWrite = millis();
  int CalculatePWM = SERVO_MIDPOINT + LastThetaPan * GEAR_RATIO_XY * DEGREE_TO_PWM;
  
  //Give the servo time to move without stopping the entire program, and check for PWM past limits (i.e. do not run in continuous mode)
  if(servoPanLastWrite + 15 > millis() && CalculatePWM > 610 && CalculatePWM < 2390){
    servoPan.writeMicroseconds(CalculatePWM);
  }
}

//Sets servo motor to position by converting input angle into PWM
void SetTilt(int Theta){   
  servoTiltLastWrite = millis();
  
  if(servoTiltLastWrite + 15 > millis()){    //Give the servo time to move without stopping the entire program
    servoTilt.writeMicroseconds(SERVO_MIDPOINT +  Theta * GEAR_RATIO_XZ * DEGREE_TO_PWM);
  }   
}

//--------------------------------------------------------Compass Functions---------------------------------------------------------------

//Create magnetic north and acceleration vectors
//void GetVectorData(long double *pMagNorthComp, long double *pNegGravity){
void GetVectorData(float *pBiasVec, float *pSoftIronTransform, float *pMagNorthComp, float *pNegGravity){
  float RawMinusBias[Size];
  //Create magnetic north vector
  mag.getEvent(&magEvent);

//  //Assign and calibrate magnetic data
//  pMagNorthComp[0] = (magEvent.acceleration.x - MinMag_X) / (MaxMag_X - MinMag_X) * 2 - 1;
//  pMagNorthComp[1] = (magEvent.acceleration.y - MinMag_Y) / (MaxMag_Y - MinMag_Y) * 2 - 1;
//  pMagNorthComp[2] = (magEvent.acceleration.z - MinMag_Z) / (MaxMag_Z - MinMag_Z) * 2 - 1;
  
  //Assign and calibrate magnetic data
  pMagNorthComp[0] = magEvent.magnetic.x;
  pMagNorthComp[1] = magEvent.magnetic.y;
  pMagNorthComp[2] = magEvent.magnetic.z;

  Matrix.Subtract((float*)pMagNorthComp, (float*)pBiasVec, 3, 1, (float*)RawMinusBias);
  Matrix.Multiply((float*)pSoftIronTransform, (float*)RawMinusBias, 3, 3, 1, (float*)pMagNorthComp);  

  //Create gravity vector
//  long double Gravity[Size];
  accel.getEvent(&accelEvent);

  pNegGravity[0] = accelEvent.acceleration.x;
  pNegGravity[1] = accelEvent.acceleration.y;
  pNegGravity[2] = accelEvent.acceleration.z;

//  Gravity[0] = accelEvent.acceleration.x;       //FOR SOME REASON THE ACCELEROMETER IS GIVING GRAVITY AS POINTING UP
//  Gravity[1] = accelEvent.acceleration.y;
//  Gravity[2] = accelEvent.acceleration.z;

//    pNegGravity[0] = -1 * Gravity[0];
//    pNegGravity[1] = -1 * Gravity[1];
//    pNegGravity[2] = -1 * Gravity[2];

  //Display raw data when debugging
//  if(DEBUG){
//      Matrix.Print((float*)pMagNorthComp, Size, 1, "Raw Magnetic Data in Compass Frame of Reference"); 
//      Matrix.Print((float*)pNegGravity, Size, 1, "Raw Acceleration Data in Compass Frame of Reference"); 
//  }
}

//Define inertial axes using projection and cross product, orthonormal basis in R3
//Magnetic compass vector readings should be in InertX-InertZ plane
void DefineInertAxes(float *pMagNorthComp, float *pNegGravity, float *pUnitInertX, float *pUnitInertY, float *pUnitInertZ){
  //Define vertical axis in inertial frame of reference
  float InertZ[Size];
  DotProductProject((float*)pNegGravity, (float*)pMagNorthComp, Size, Size, (float*)InertZ);    //Project magnetic compass vector onto vertical gravity vector
  if(DotProductAngle((float*)pNegGravity, (float*)pMagNorthComp, Size, Size) > 90){             //Check whether magnetic compass vector points above or below horizontal to make sure z-axis is defined as up
    ScalarMult((float*)InertZ, -1, 3, 1, (float*)InertZ);
  }
  Normalize((float*)InertZ, Size, (float*)pUnitInertZ);                                         //Normailze to produce unit vector
//  if(DEBUG){
//    Matrix.Print((float*)InertZ, Size, 1, "Inertial Z-Axis");
//    Matrix.Print((float*)pUnitInertZ, Size, 1, "Normalized Inertial Z-Axis");
//}

  //Define forward axis in horizontal plane of inertial frame of reference, using Gram-Schmidt process
  float InertX[Size];
  if(DotProductAngle((float*)pNegGravity, (float*)pMagNorthComp, Size, Size) > 90){
    Matrix.Add((float*)pMagNorthComp, (float*)InertZ, Size, 1, (float*)InertX);
  }
  else{
    Matrix.Subtract((float*)pMagNorthComp, (float*)InertZ, Size, 1, (float*)InertX);
  }
  Normalize((float*)InertX, Size, (float*)pUnitInertX);
//  if(DEBUG){
//    Matrix.Print((float*)InertX, Size, 1, "Inertial X-Axis");
//    Matrix.Print((float*)pUnitInertX, Size, 1, "Normalized Inertial X-Axis");
//}

  //Define lateral axis in horizontal plane of inertial frame of reference, cross product of two orthogonal vectors will give third basis vector
  float InertY[Size];
  CrossProduct_1x3((float*)pUnitInertZ, (float*)pUnitInertX, Size, Size, (float*)InertY);
  Normalize((float*)InertY, Size, (float*)pUnitInertY);
}

//Project compass x-axis onto the inertial horizontal plane and convert it to the inertial frame of reference, gives antenna heading vector
void ProjectCompX(float *pChangeBasis, float *pUnitInertX, float *pUnitInertY, float *pHeadingAnt){
  const float CompassX[Size] = {1, 0, 0};
  float ProjCompassX[Size];
  ProjectVecToPlane((float*)CompassX, (float*)pUnitInertX, (float*)pUnitInertY, Size, (float*)ProjCompassX);    //Project compass x-axis into horizontal plane
  Matrix.Multiply((float*)pChangeBasis, (float*)ProjCompassX, Size, Size, 1, (float*)pHeadingAnt);              //Convert projected compass x-axis to inertial frame of reference
//  if(DEBUG){
//    Matrix.Print((float*)pHeadingAnt, Size, 1, "Compass X-Axis Projected into Horizontal Plane and Converted to Inertial Coordinates");   //Z component should be zero
//    Matrix.Print((float*)pHeadingAnt, Size, 1, "Antenna Heading in Inertial Coordinates");   //Z component should be zero
//  }
}

//Create change of basis matrix, used to convert between compass and inertial coordinate systems
void DefineChangeBasis(float *pAxisX, float *pAxisY, float *pAxisZ, float *pMatrix, float *pMatrixRevert){
  int i, j, k;
  
  for(i=0; i<Size ; i++){
    pMatrixRevert[i * Size] = pAxisX[i];
  }
  for(j=0; j<Size ; j++){
    pMatrixRevert[j * Size + 1] = pAxisY[j];
  }
  for(k=0;k<Size ; k++){
    pMatrixRevert[k * Size + 2] = pAxisZ[k];
  }
  //Copy matrix to produce change of basis matrix for reverting back to compass coordinate system from inertial coordinate system
  Matrix.Copy((float*)pMatrixRevert, Size, Size, (float*)pMatrix);
  //Invert matrix to produce change of basis matrix for converting from compass to inertial coordinates
  //We know ChangeBasis is invertible because it is a square matrix and it is a basis (linearly independent)
  Matrix.Invert((float*)pMatrix, Size);   //Argument becomes result, so ChangeBasis is replaced by its inverted form
}

//Calculate angular antenna heading, relative to magnetic north
//Clockwise (E) is positive
float AntennaHeadingTrue(float *pChangeBasis, float *pUnitInertX, float *pHeadingAnt){
  float HeadingMagInert[Size];  //Vector in direction of magnetic north, in horizontal inertial frame of reference
  float CheckDir[Size];         //Vector used to check direction of theta
  float CheckDirNorm[Size];     //Normailzed CheckDir
  float ThetaOffset;

  //HeadingMagInert is the magnetic north vector converted to the inertial frame of reference and projected onto the horizontal plane
  //We can use the inertial unit x-axis because it was defined in plane with the magnetic north and z-axis vectors, therefore it already is the projection onto the horizontal plane
  Matrix.Multiply((float*)pChangeBasis, (float*)pUnitInertX, Size, Size, 1, (float*)HeadingMagInert);
//  if(DEBUG){
//    Matrix.Print((float*)HeadingMagInert, Size, 1, "Converted Magnetic Data in Inertial Frame of Reference");
//    Matrix.Print((float*)HeadingAnt, Size, 1, "Antenna Heading in Inertial Coordinates");
//  }
  ThetaOffset = DotProductAngle((float*)pHeadingAnt, (float*)HeadingMagInert, Size, Size);

  //Check whether the angle is in the positive or negative direction
  CrossProduct_1x3((float*)pHeadingAnt, (float*)HeadingMagInert, Size, Size, (float*)CheckDir);
  if(CheckDir[2] < 0){
    ThetaOffset = -1 * ThetaOffset;
  }
//  if(DEBUG){
//    Matrix.Print((float*)CheckDir, Size, 1, "CheckDir Vector");
//    Serial.print("\nCompass Heading Relative to Magnetic North (float) : "); Serial.println(ThetaOffset);
//  }

  //Calculate offset from true north using declination
  float HeadingTrue = ThetaOffset + declination;

  //Manually set North direction
  if(NorthCalibrate){
    HeadingTrue = HeadingTrue + NorthOffset;
  }

  if(HeadingTrue < -180){
    HeadingTrue = HeadingTrue + 360;
  }

  if(HeadingTrue > 180){
    HeadingTrue = HeadingTrue - 360;
  }

  //Calibrate compass so that it reads North and South as colinear
  //Take reading after setting NorthOffset
  if(SouthCalibrate){
    float SouthRatio = 360 - ((180.0 * 180.0) / abs(TrueSouth));
    if(TrueSouth < 0){
      if(HeadingTrue < TrueSouth){
        HeadingTrue = HeadingTrue * (180.0 / abs(TrueSouth)) + 360;
      }
      else if(HeadingTrue > 0){
        HeadingTrue = HeadingTrue * (SouthRatio / 180.0);
      }
      else{
        HeadingTrue = HeadingTrue * (180.0 / abs(TrueSouth));
      }
    }
    if(TrueSouth > 0){
      if(HeadingTrue > TrueSouth){
        HeadingTrue = HeadingTrue * (180.0 / abs(TrueSouth)) - 360;
      }
      else if(HeadingTrue < 0){
        HeadingTrue = HeadingTrue * (SouthRatio / 180.0);
      }
      else{
        HeadingTrue = HeadingTrue * (180.0 / abs(TrueSouth));
      }
    }
  }

  //Convert true North heading to value between -180 and 180
  if(HeadingTrue < -180){
    HeadingTrue = HeadingTrue + 360;
  }

  if(HeadingTrue > 180){
    HeadingTrue = HeadingTrue - 360;
  }

  //Return antenna heading as an angle, CW relative to true north
  return HeadingTrue;
}

//-------------------------------------MAIN------------------------------------------------------

void loop(){
  //If there are incoming bytes available from the server, read them and print them:
  if (client.available()) {
  char c = client.read();
  Data = String(c);

    //Set servo motors to their midpoints for automated calibration and calculate offset angles for servo reference
    //Locate antenna GPS coordinates, used to calculate distance to plane and altitude difference
    if(InitializeAntenna){
      //Set servo motors to midpoints and calcuates reference angles at midpoint positions
      servoTilt.writeMicroseconds(SERVO_MIDPOINT);
      servoPan.writeMicroseconds(SERVO_MIDPOINT);
      delay(10000);
      
      //Retrieves sensor (compass and accelerometer) data
      GetVectorData((float*)BiasVec, (float*)SoftIronTransform, (float*)MagNorthComp, (float*)NegGravity);
      //Defines an inertial frame of reference using the magnetic north vector read from the compass
      DefineInertAxes((float*)MagNorthComp, (float*)NegGravity, (float*)UnitInertX, (float*)UnitInertY, (float*)UnitInertZ);
      //Defines a change of basis matrix used to convert the magnetic north vector from the compass coordinate system to the inertial frame of reference
      DefineChangeBasis((float*)UnitInertX, (float*)UnitInertY, (float*)UnitInertZ, (float*)ChangeBasis, (float*)RevertBasis);    
      //Converts compass x-axis to inertial frame of reference, then projects converted vector into horizontal plane
      ProjectCompX((float*)ChangeBasis, (float*)UnitInertX, (float*)UnitInertY, (float*)HeadingAnt);
      //Sets reference angle for servo pan motors, used to locate angular position of motor's midpoint in relation to true north
      ThetaPanRef = AntennaHeadingTrue((float*)ChangeBasis, (float*)UnitInertX, (float*)HeadingAnt);
      //Sets reference angle for servo tilt motors, used to locate angular position of motor's midpoint in relation to the horizontal plane
      ThetaTiltRef = SetTiltOffset((float*) UnitInertX, (float*)UnitInertY);
  
      InitializeAntenna = false;
    }

    //Accumulate data characters in "Column" until the next comma is reached
    if (c != ','){
      Column += Data;
    }

    //Whenever char that is read is a comma do this
    if (c == ','){
      
      //Set the comma count of each important column
      if (L1 == true){  
        if (Column == "lat"){
          latCnt = commaCnt;
          //Serial.println(latCnt);
        }
       
        if (Column == "lon"){
          longCnt = commaCnt;
          //Serial.println(longCnt);
        }
       
        if (Column == "altitude"){
          altitudeCnt = commaCnt;
          //Serial.println(altitudeCnt);
        }
      }
        
      //If the comma count indicates the latitude column, copy number into "lat"
      if (commaCnt == latCnt && L1 == false){ 
         buff = Column.c_str();
         lat = strtod(buff,NULL);
      }
       
      //If the comma count indicates the longitude column, copy number into "lon"
      //West is negative, East is positive
      if (commaCnt == longCnt && L1 == false){
         buff = Column.c_str();
         lon = strtod(buff,NULL);
      }
      
      //If the comma count indicates the altitude column, copy number into "altitude"
      //Altitude indicates height of plane from ground, calibrated through ground station
      if (commaCnt == altitudeCnt && L1 == false){
         buff = Column.c_str();
         altitude = strtod(buff,NULL);
      }
           
      Column = "";        //Assign empty string to "Column"
      commaCnt += 1;      //Increment comma count
               
    }    //end if comma check     
        
    if (c =='\n'){ //when there is a new line do this

      L1 = false;         //Reset heading line indicator
      commaCnt = 0;       //Reset comma count
      lineCnt +=1;        //Increment line count
      Column = "";        //Assign empty string to Column
    
      if (L1 == false){
        //Retrieves sensor (compass and accelerometer) data
        GetVectorData((float*)BiasVec, (float*)SoftIronTransform, (float*)MagNorthComp, (float*)NegGravity);
        //Defines an inertial frame of reference using the magnetic north vector read from the compass
        DefineInertAxes((float*)MagNorthComp, (float*)NegGravity, (float*)UnitInertX, (float*)UnitInertY, (float*)UnitInertZ);
        //Defines a change of basis matrix used to convert the magnetic north vector from the compass coordinate system to the inertial frame of reference
        DefineChangeBasis((float*)UnitInertX, (float*)UnitInertY, (float*)UnitInertZ, (float*)ChangeBasis, (float*)RevertBasis); 
        //Converts compass x-axis to inertial frame of reference, then projects converted vector into horizontal plane
        ProjectCompX((float*)ChangeBasis, (float*)UnitInertX, (float*)UnitInertY, (float*)HeadingAnt);

        //Gives direction/amount that servo motor should pan this iteration, POSITIVE -> RIGHT, NEGATIVE -> LEFT
        ThetaPan = GetThetaXY(getDistance(latORG, lonORG, latORG, lon), getDistance(latORG, lonORG, lat, lonORG)) - ThetaPanRef;         
        //Gives direction/amount that servo motor should tilt this iteration, POSITIVE -> UP, NEGATIVE -> DOWN, Use if input altitude is already the difference between antenna and plane
        ThetaTilt = GetThetaXZ(getDistance(latORG, lonORG, lat, lon), altitude) - ThetaTiltRef;
//        //Gives direction/amount that servo motor should tilt this iteration, POSITIVE -> UP, NEGATIVE -> DOWN, Use if input altitude is from sea level
//        ThetaTilt = GetThetaXZ(getDistance(latORG, lonORG, lat, lon), altitude, altORG) - ThetaTiltRef;

        if(DEBUG){
          //REWRITE DEBUG
        }
        
        else{
          SetPan(ThetaPan);
          SetTilt(ThetaTilt);
        }
        //Servo::refresh();
      }
   
    }   //endif new line check
   
  
  //endif usable char check
  
  
  //Serial.print(c);

    
  }   //endif client available

  //As long as there are bytes in the serial queue, read them and send them out the socket if it's open:
  while (Serial.available() > 0){
    char inChar = Serial.read();
    if (client.connected()){
      client.print(inChar); 
    }
  }

  //If the server's disconnected, stop the client:
  if (!client.connected()){
    Serial.println();
    Serial.println("disconnecting.");
    client.stop();
    
    //Reset servo pan motors to midpoint before shutting down, useful for quick setup next time
    servoPan.writeMicroseconds(SERVO_INITIALIZE);  
    servoTilt.writeMicroseconds(SERVO_INITIALIZE);
    
    //Do nothing, use infinite while statement to stall program
    while(true);
  }
}

