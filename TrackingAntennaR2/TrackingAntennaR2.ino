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
    
    Information taken from: http://www.robotshop.com/ca/en/hitec-hs785hb-servo-motor.html
    
    PWM 600 -> angle 0
    PWM 2400 -> angle 1260
    
    Center = PWM 1500 -> angle 630
    1 degree = PWM 1.429

Magnetometer(Compass) Information:
Model No. ST LSM303DLHC
  
    Information taken from: https://www.adafruit.com/products/1120
  
    Communication between compass and board is over i2c.
    All readings are in micro-Teslas
    Compass attached to pins: SCL, SDA, 3.3V, GND
  
    Note: Compass points to magnetic north, not true north
  
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
#define SERVO_INITIALIZE 0  //Used to set servo motors to their mid point
#define EARTH_RADIUS 6371000 //in m
#define DEGREE_TO_PWM 0.7
#define GEAR_RATIO_XZ 7  //how much it is geared down by
#define GEAR_RATIO_XY 1
#define COMPASS_ID 12345
#define GRAVITY_CONSTANT 9.80665
#define SS 53  //Choose SPI slave-select pin
#define Size 3  //Size of square matrices and length of vectors
#define DEBUG false  //When "true" turns on the printing to serial for testing

//-----------------------------------GLOBAL VARIABLES----------------------------

/***      MAIN LOOP     ***/
//Counts to describe the positions of delimiters in incoming ground station data and number of data packages received
int commaCnt = 0;
int lineCnt = 0;

//String objects used to hold incoming values from ground station
String Data = "";
String Column = "";
const char *buff = Column.c_str(); //Uses pointer "buff" to continuously assign new data to a string in Column

//Comma positions of important data
int latCnt = 0;
int longCnt = 0;
int altitudeCnt = 0;

//Used to denote header line during parsing of plane's GPS data
boolean L1 = true;//Line 1 marker, use comman count to assign commaCnt values to pertinent headings

//Angles
//int ThetaXY = 0;
//int ThetaXZ = 0;
//int AntennaTilt = 0;
int ThetaPanRef = 0;
int ThetaTiltRef = 0;
int ThetaPan = 0;
int ThetaTilt = 0;

unsigned long servoPanLastWrite = 0;
unsigned long servoTiltLastWrite = 0;

/***      GPS     ***/
typedef struct _GPSData {
    long double latitude;
    long double longitude;
    float Gtime;
    float Gspeed;            //4 Bytes
    int altitude;
    int heading;            //2 Bytes
    char satellites;
    char positionFix;       //1 Byte
} GPSData;

int OutDATA = 0;
GPSData InDATA;   //InDATA is a struct

//Latitude, Longitude and Altitude variables
double lat = 0;  //GPS latitude of plane
double lon = 0;  //GPS longitude of plane
double altitude = 0;  //GPS altitude of plane
double latORG = 0;  //GPS latitude of antenna (reference)
double lonORG = 0;  //GPS longitude of antenna (reference)
double altORG = 0;  //GPS altitude of antenna (reference)

/***      COMPASS & ACCELEROMETER     ***/
//Declare sensor variables
sensors_event_t accelEvent;   //Accelerometer Data
sensors_event_t magEvent;   //Magnetic Compass Data

//Declination values: South Port Manitoba on 04/30/2016 = 4.136deg E, Waterloo Ontario on 03/09/2016 = 9.633deg W
//East declinations are POSITIVE, West declinations are NEGATIVE
float declination = -9.633;

//Declare vectors for sensor calculations
//float Gravity[Size];
float NegGravity[Size];   //Vertical direction (negative of gravity)
float MagNorthComp[Size];   //Magnetic north in compass frame of reference
float CompassXInert[Size];   //X-Axis of compass converted to inertial frame of reference
float HeadingAnt[Size];   //Heading of antenna in horizontal plane of inertial frame of reference, colinear with x-axis of compass projected into horizontal inertial plane
float UnitInertX[Size];   //X-Axis in inertial frame of reference
float UnitInertY[Size];   //Y-Axis in inertial frame of reference
float UnitInertZ[Size];   //Z-Axis in inertial frame of reference
float ChangeBasis[Size][Size];   //Change of basis matrix for converting vectors from compass frame of reference to inertial frame of reference, multiply compass vector on left


//-----------------------------------INITIALIZATION VALUES----------------------------

// Enter a MAC address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {0x90,0xA2,0xDA,0x0F,0x2C,0x9A};
IPAddress ip(192,168,1,199); //this is for the arduino


// Enter the IP address of the server you're connecting to:
IPAddress server(192,168,1,104);  //pi or computer that is hosting the data

/****        END OF INITIALIZATION      **********/

// Initialize the Ethernet client library
// with the IP address and port of the server 
// that you want to connect to (port 23 is default for telnet;
// if you're using Processing's ChatServer, use  port 10002):
EthernetClient client;

//Servo Objects
Servo servoPan;  // create servo object to control a servo 
Servo servoTilt;  // create servo object to control a servo 

//Initializes the compass library with an arbitrarily set compass id
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(COMPASS_ID); //Create compass object to read magnetic values
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(COMPASS_ID); //Create compass object to read acceleration values

void setup(){  // attaches the servos
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

  //Prints accelerometer data repeatedly so that antenna may be aligned vertically
//  VerticalAlignLoop();
  
  // start the Ethernet connection:
  Ethernet.begin(mac,ip);
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
   while (!Serial) {
     // wait for serial port to connect. Needed for Leonardo only
  }

  // give the Ethernet shield a second to initialize:
  delay(1000);
  Serial.println("connecting...");

  // if you get a connection, report back via serial:
  if (client.connect(server, 1234)) {
    Serial.println("connected");
  }
  else {  // if you didn't get a connection to the server:
    Serial.println("connection failed");
  }

  //Initialize SPI connection and retrieve antenna coordinates
  pinMode(SS, OUTPUT);  //Declare slave-select pin
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);  //Sets bit order for data transfer, determined by microprocessor
  LocateAntenna();  //Calculates antenna location using GPS coordinates
  double latORG = InDATA.latitude;  //Sets GPS latitude of antenna (reference)
  double lonORG = InDATA.longitude;  //Sets GPS longitude of antenna (reference)
  double altORG = InDATA.altitude;  //Sets GPS altitude of antenna (reference)
  
  //Set servo pan motor to midpoint and calcuates reference angles at midpoint position
//  servoTilt.writeMicroseconds(SERVO_INITIALIZE);
  servoPan.writeMicroseconds(SERVO_INITIALIZE);
  ThetaTiltRef = SetTiltOffset();
  ThetaPanRef = AntennaHeadingMag();
}

// ---------------------------------GPS FUNCTIONS--------------------------------------

//Initialize antenna location using on board GPS
void LocateAntenna(){  //Uses GPS data incoming through SPI connection to locate antenna, used as reference for plane coordinates (origin)

  int i;
//  char *pGPSData = (char *)(&InDATA);                                                 //MAY BE DIFFICULT TO USE SIZEOF TO SET
  uint16_t *pGPSData = (uint16_t*)(&InDATA);                                            //MAY NEED TO ADD LIBRARY, BUT MAY BE DIFFICULT TO USE SIZEOF TO SET FOR LOOP
  digitalWrite(SS, LOW);                                                                //INPUT (SDI) CONNECTS TO OUTPUT ON THE ARDUINO
  for (i=0; i < sizeof(GPSData); i+=2, pGPSData++){                                     //USE SIZEOF
//    *(pGPSData+i) = SPI.transfer16(OutDATA);                                          //THIS ALLOWS ME TO INCREMENT ONLY i IN THE FOR LOOP
//    *(&InDATA+i) = SPI.transfer16(OutDATA);                                           //JUMPS OVER STRUCT, +i CAUSES INCREMENT OF SIZE OF STRUCT WHEN WRITTEN THIS WAY
    *(pGPSData) = SPI.transfer16(OutDATA);
  }
  digitalWrite(SS, HIGH);
}

//double yCoord = 0;             //PROBABLY SHOULDN'T BE A GLOBAL VARIABLE
//double zCoord = 0;             //PROBABLY SHOULDN'T BE A GLOBAL VARIABLE
//double Hyp = 0;                //PROBABLY SHOULDN'T BE A GLOBAL VARIABLE

//void getYZCoordinates(double latitude, double longitude){ //Computes xCord, yCord and Hyp, also sets them as global variables                   COULD PROBABLY MOVE THESE INTO LOOP INSTEAD OF HAVING SEPARATE FUNCTION
//  zCoord = getDistance(latORG, lonORG, latitude, lonORG);                                                                                     //WOULD ELIMINATE YCOORD, ZCOORD AND HYP AS GLOBAL VARIABLES
//  yCoord = getDistance(latORG, lonORG, latORG, longitude);
//  Hyp = getDistance(latORG, lonORG, latitude, longitude);
//}

double getDistance(double Lat1, double Lon1, double Lat2, double Lon2){ //Calculates distance between two points (latitude and longitude), in meters

  double disLat = (Lat2 - Lat1) * M_PI / 180;
  double disLon = (Lon2 - Lon1) * M_PI / 180;

  //"Haversine" formula for calculating distance between two points given latitude and longitude
  double a = sin(disLat / 2) * sin(disLat / 2) + cos(Lat1 * M_PI / 180) * cos(Lat2 * M_PI / 180) * sin(disLon / 2) * sin(disLon / 2);
  return EARTH_RADIUS * (2 * atan2(sqrt(a),sqrt(1 - a)));

  //Pythagorean Theorem
  //Hyp = sqrt(sq(disLat) + sq(disLon)); //Comment out if using the Haversine formula
}

//--------------------------------------------------------Servo Functions---------------------------------------------------------------

//Calculate angle between plane and true north
//double GetThetaXY (long double lonDiff, long double latDiff){                                                   //CAN SIMPLY USE atan2 IN LOOP FUNCTION, DOESN'T NEED TO BE A FUNCTION
//  double ThetaXY = atan2(lonDiff, latDiff) * 180 / M_PI;
//
//  return ThetaXY; 
//}

//Calcuated angle of plane off of horizontal
double GetThetaXZ (long double distance, long double height){
  double altDiff = height - altORG;
  double ThetaXZ = atan2(altDiff, distance) * 180 / M_PI;
  
  return ThetaXZ;
}

//Calculate tilt angle when servo is at mid-point
int SetTiltOffset(){
  float CheckDir[Size];
  float CheckDirNorm[Size];
  int ThetaInitial;
  
  servoTilt.writeMicroseconds(SERVO_INITIALIZE);
  ThetaInitial = DotProductAngle((float*)CompassXInert, (float*)HeadingAnt, Size, Size);

  //Check direction of ThetaInitial
  CrossProduct_1x3((float*)CompassXInert, (float*)HeadingAnt, Size, Size, (float*)CheckDir);
  Normalize((float*)CheckDir, Size, (float*)CheckDirNorm);
  if(((CheckDirNorm[0] >= UnitInertY[0] + 0.03) || (CheckDirNorm[0] <= UnitInertY[0] - 0.03)) || ((CheckDirNorm[1] >= UnitInertY[1] + 0.03) || (CheckDirNorm[1] <= UnitInertY[1] - 0.03)) || ((CheckDirNorm[2] >= UnitInertY[2] + 0.03) || (CheckDirNorm[2] <= UnitInertY[2] - 0.03))){
//  if(CheckDir[1] > 0){                                                                                         //CAN I CHECK DIRECTIONS LIKE THIS WITH ORTHONORMAL BASES NOT DEFINED BY THE INDENTITY VECTORS? MAYBE NORMALIZE AND COMPARE TO UNIT VECTOR
    ThetaInitial = -1 * ThetaInitial;
  }
  return ThetaInitial;
}

//Calculate pan angle when servo is at mid-point
//int SetPanOffset(){
//  int ThetaInitial;
//
//  servoPan.writeMicroseconds(SERVO_INITIALIZE);
//  ThetaInitial = AntennaHeadingMag();
//  
//  //Check direction of ThetaInitial                     DON'T NEED THIS IF AntennaHeadingMag is -180<Theta<180 --> DON'T NEED FUNCTION IF THIS IS THE CASE
//  if(ThetaInitial > 180){
//    ThetaInitial = ThetaInitial - 360;
//  }
//  return ThetaInitial;
//}

void SetPan(int Theta){
   if(servoPanLastWrite + 15 > millis()){    //Give the servo time to move without stopping the entire program
      servoPan.writeMicroseconds(abs(SERVO_MIDPOINT + Theta * GEAR_RATIO_XY * DEGREE_TO_PWM));
      servoPanLastWrite = millis();
    }
}

void SetTilt(int Theta){   
    if(servoTiltLastWrite + 15 > millis()){    //Give the servo time to move without stopping the entire program
     servoTilt.writeMicroseconds(abs(SERVO_MIDPOINT +  Theta * GEAR_RATIO_XZ * DEGREE_TO_PWM));
     servoTiltLastWrite = millis();
    }   
}

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

//Dot Angle, calculates the angle between two vectors
int DotProductAngle(float *pVecA, float *pVecB, int SizeA, int SizeB){
  long double Dot = DotProduct((float*)pVecA, (float*)pVecB, SizeA, SizeA);
  long double MagnitudeA = sqrt(DotProduct((float*)pVecA, (float*)pVecA, SizeA, SizeA));
  long double MagnitudeB = sqrt(DotProduct((float*)pVecB, (float*)pVecB, SizeA, SizeA));
  int Theta = acos(Dot / (MagnitudeA * MagnitudeB));
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

//Project vector onto 2D plane in Rn
void ProjectVecToPlane(float *pVec, float *pBase1, float *pBase2, int Rn, float *pProj){
  float Base1Scaled[Rn];
  float Base2Scaled[Rn];
  
  float Dot1 = DotProduct((float*)pVec, (float*)pBase1, Rn, Rn);
  float Dot2 = DotProduct((float*)pVec, (float*)pBase2, Rn, Rn);
  ScalarMult((float*)pBase1, Dot1, Rn, 1, (float*)Base1Scaled);
  ScalarMult((float*)pBase2, Dot2, Rn, 1, (float*)Base2Scaled);
  
  Matrix.Add((float*)Base1Scaled, (float*)Base2Scaled, Rn, Rn, (float*)pProj);
}

//Normalize vectors (make length == 1)
void Normalize(float *pVec, int VecSize, float *pNorm){
  int i;

  for(i=0; i<VecSize; i++){
      pNorm[i] = pVec[i] / sqrt(DotProduct((float*)pVec, (float*)pVec, VecSize, VecSize));
  }
}

//--------------------------------------------------------Compass Functions---------------------------------------------------------------

//Function loop for physical antenna setup, using accelerometer data                                                                      SHOULD BE REDUNDANT IF MATRIX FUNCTIONS OPERATE PROPERLY
//void VerticalAlignLoop(){  //Vertically aligned when y and z components are approzimately zero
//  while(accelEvent.acceleration.y < 0.35 | accelEvent.acceleration.z < 0.35){
//    accel.getEvent(&accelEvent);
//
//    //Prints out raw data for test setup
//    Serial.print("X Acceleration: "); Serial.print(accelEvent.acceleration.x); Serial.println(" m/s^2 ");
//    Serial.print("Y Acceleration: "); Serial.print(accelEvent.acceleration.y); Serial.println(" m/s^2 ");
//    Serial.print("Z Acceleration: "); Serial.print(accelEvent.acceleration.z); Serial.println(" m/s^2 ");
//    delay(1000);
//  }
//}

//Create magnetic north and acceleration vectors
float GetVectorData(){
  //Create magnetic north vector
  mag.getEvent(&magEvent);

  MagNorthComp[0] = magEvent.acceleration.x;
  MagNorthComp[1] = magEvent.acceleration.y;
  MagNorthComp[2] = magEvent.acceleration.z;
  
  //Create gravity vector
  accel.getEvent(&accelEvent);

  NegGravity[0] = accelEvent.acceleration.x;
  NegGravity[1] = accelEvent.acceleration.y;
  NegGravity[2] = accelEvent.acceleration.z;

//  Gravity[0] = accelEvent.acceleration.x;   //FOR SOME REASON THE ACCELEROMETER IS GIVING GRAVITY AS POINTING UP
//  Gravity[1] = accelEvent.acceleration.y;
//  Gravity[2] = accelEvent.acceleration.z;

//    NegGravity[0] = -1 * Gravity[0];
//    NegGravity[1] = -1 * Gravity[1];
//    NegGravity[2] = -1 * Gravity[2];
}

//Define inertial axes using projection and cross product, orthonormal basis in R3
//Magnetic compass vector readings should be in InertX-InertZ plane
float DefineInertAxes(){
  //Define vertical axis in inertial frame of reference
  float InertZ[Size];
  float scalarZ = DotProduct((float*)MagNorthComp, (float*)NegGravity, Size, Size);
  ScalarMult((float*)NegGravity, scalarZ, Size, 1, (float*)InertZ);
  Normalize((float*)InertZ, Size, (float*)UnitInertZ);

  //Define forward axis in horizontal plane of inertial frame of reference
  float InertX[Size];
  Matrix.Subtract((float*)MagNorthComp, (float*)InertZ, Size, 1, (float*)InertX);
  Normalize((float*)InertX, Size, (float*)UnitInertX);

  //Define lateral axis in horizontal plane of inertial frame of reference
  float InertY[Size];
  CrossProduct_1x3((float*)UnitInertZ, (float*)UnitInertX, Size, Size, (float*)InertY);
  Normalize((float*)InertY, Size, (float*)UnitInertY);

  //Define compass x-axis in inertial frame of reference and project it into the inertial horizontal plane
  const float CompassX[Size] = {1, 0, 0};
  Matrix.Multiply((float*)ChangeBasis, (float*)CompassX, Size, Size, 1, (float*)CompassXInert);   //Convert compass x-axis to inertial frame of reference
  ProjectVecToPlane((float*)CompassXInert, (float*)UnitInertX, (float*)UnitInertY, Size, (float*)HeadingAnt);   //Project compass x-axis into horizontal plane
}

//Define inertial axes using projection and cross product, orthonormal basis in R3
//Magnetic compass vector readings should be in InertX-InertZ plane

//Define vertical axis in inertial frame of reference                                       X AXIS FOR NOW, WILL CHANGE ONCE MOUNTED --> TRY TO MAKE VERTICAL = Z          CHANGE ALL OTHER FUNCTIONS TO REFLECT THIS ORIENTATION
//float DefineInertZ(){
//  float InertZ[Size];
//  float scalarZ = DotProduct((float*)MagNorthComp, (float*)NegGravity, Size, Size);
//  ScalarMult((float*)NegGravity, scalarZ, Size, 1, (float*)InertZ);
//  Normalize((float*)InertZ, Size, (float*)UnitInertZ);
//}
//
////Define forward axis in horizontal plane of inertial frame of reference
//float DefineInertX(){
//  float InertX[Size];
//  Matrix.Subtract((float*)MagNorthComp, (float*)InertZ, Size, 1, (float*)InertX);
//  Normalize((float*)InertX, Size, (float*)UnitInertX);
//}
//
////Define lateral axis in horizontal plane of inertial frame of reference
//float DefineInertY(){
//  float InertY[Size];
//  CrossProduct_1x3((float*)UnitInertZ, (float*)UnitInertX, Size, Size, (float*)InertY);
//  Normalize((float*)InertY, Size, (float*)UnitInertY);
//}

//Create change of basis matrix
float DefineChangeBasis(){
  int i, j, k;
  for(i=0; i<Size; i++){
    ChangeBasis[i][0] = UnitInertX[i];
  }
  for(j=0; j<Size; j++){
    ChangeBasis[j][1] = UnitInertY[j];
  }
  for(k=0;k<Size; k++){
    ChangeBasis[k][2] = UnitInertZ[k];
  }
  //We know ChangeBasis is invertible because it is a square matrix and it is a basis (linearly independent)
  Matrix.Invert((float*)ChangeBasis, Size);   //Argument becomes result, so ChangeBasis is replaced by its inverted form
}

//Calculate antenna heading relative to magnetic north
//Clockwise (E) is positive
int AntennaHeadingMag(){
  float MagNorthInert[Size];    //Magnetic north in inertial frame of reference
  float HeadingMag[Size];       //Vector in direction of magnetic north, in horizontal inertial frame of reference
  float CheckDir[Size];         //Vector used to check direction of theta
  float CheckDirNorm[Size];     //Normailzed CheckDir
  int ThetaOffset;

  Matrix.Multiply((float*)ChangeBasis, (float*)MagNorthComp, Size, Size, 1, (float*)MagNorthInert);    //MagNorthInert is the converted magnetic (column) vector, in the inertial frame of reference
  ProjectVecToPlane((float*)MagNorthInert, (float*)UnitInertX, (float*)UnitInertY, Size, (float*)HeadingMag);   //Projection of MagNorthInert into horizontal inertial plane
  ThetaOffset = DotProductAngle((float*)HeadingAnt, (float*)HeadingMag, Size, Size);

  //Checks whether the angle is in the positive or negative direction
  CrossProduct_1x3((float*)HeadingAnt, (float*)HeadingMag, Size, Size, (float*)CheckDir);
  Normalize((float*)CheckDir, Size, (float*)CheckDirNorm);
  if(((CheckDirNorm[0] >= UnitInertY[0] + 0.03) || (CheckDirNorm[0] <= UnitInertY[0] - 0.03)) || ((CheckDirNorm[1] >= UnitInertY[1] + 0.03) || (CheckDirNorm[1] <= UnitInertY[1] - 0.03)) || ((CheckDirNorm[2] >= UnitInertY[2] + 0.03) || (CheckDirNorm[2] <= UnitInertY[2] - 0.03))){
//  if(CheckDir[2] < 0){
    ThetaOffset = -1 * ThetaOffset;
  }
  return ThetaOffset;
}

//Calculates antenna heading realtive to true north                                                       //DON'T THINK I NEED THIS FUNCTION
//Clockwise (E) is positive
int AntennaHeadingTrue(){
  //Calculate offset from true north using declination
  float HeadingTrue = AntennaHeadingMag() + declination;

  //Return antenna heading as an angle, CW relative to true north
  return HeadingTrue;
}

//Project MagNorthInert into horizontal plane then use with antenna heading to calculate antenna heading relative to true north
//Clockwise (E) is positive
//int GetAntennaHead(){
//  float HeadingMag[Size];   //Vector in direction of magnetic north, in horizontal inertial frame of reference
//  float CheckDir[Size];
//
//  ProjectVecToPlane((float*)MagNorthInert, (float*)UnitInertX, (float*)UnitInertY, Size, (float*)HeadingMag);
//  int ThetaOffset = DotProductAngle((float*)HeadingAnt, (float*)HeadingMag, Size, Size);
//
//  //Checks whether the angle is in the positive or negative direction
//  CrossProduct_1x3((float*)HeadingAnt, (float*)HeadingMag, Size, Size, (float*)CheckDir);
//  if(CheckDir[2] < 0){
//    ThetaOffset = -1 * ThetaOffset + 360;
//  }
//  //Calculate offset from true north using declination
//  float HeadingTrue = ThetaOffset + declination;
//
//  //Return antenna heading as an angle, CW relative to true north
//  return HeadingTrue;
//}

//Calculates the horizontal (X-Y) direction the antenna is pointing in degrees, relative to true north
//Mount sensor on back of vertical, rotating antenna post with X-axis vertical
//Clockwise (E) is positive
//float GetCompassYZ(){
//  //Retrieve magnetic sensor output
//  mag.getEvent(&magEvent);
//  
//  //Calculate antenna heading counter clockwise (W) in degrees, relative to magnetic north
//  float headingMag = -(atan2(-magEvent.magnetic.y, -magEvent.magnetic.z)*180)/ M_PI;
//
//  //Calculate true north using declination
//  headingTrue = headingMag + declination;
//  
//  if(headingTrue < 0){
//    headingTrue = headingTrue + 360;  //convert to value between 0 - 360 degrees
//  }
//  //Return antenna heading, CW relative to true north
//  return headingTrue;
//}

//-------------------------------------MAIN------------------------------------------------------

void loop(){
      // if there are incoming bytes available 
      // from the server, read them and print them:
      if (client.available()) {
        char c = client.read();
        Data = String(c);
        //Serial.print(c);
        
        if (c != ','){ //as long as the read char is not a ',' do this
          Column = Data; //Assign the Data string to Column
        }
        
        if (c == ','){ //Whenever char that is read is a comma do this
        
                      if (L1 == 1){  //sets the comma count of each important column
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
            
            
                      if (commaCnt == latCnt && L1 == 0){ //when we are at the latitude column, copy number into "latitude"
                         buff = Column.c_str();//comment this line out when using in the field      WHY?
                         //buff = Column.substring(0, 10).c_str();  //comment this line in when using in the field
                         //String tempBuff = Column.substring(2);  //comment this line in when using in the field               MIGHT NEED TO ADD THIS WITH AN IF(commaCnt == 0) IF THERE IS A NEW LINE CHARACTER AT THE BEGINNING OF THE DATA
                         //buff = tempBuff.c_str();
                         //buff[0] = ' ';
                         lat = strtod(buff,NULL);
                         //Serial.println(tempBuff);
                         //Serial.println(buff);
                      }
                       
                      if (commaCnt == longCnt && L1 == 0){ //when we are at the longitude column, copy number into "longitude"
                        //West is negative, East is positive
                         buff = Column.c_str();
                         lon = strtod(buff,NULL);
                         //Serial.println(lon);
                      }
                      if (commaCnt == altitudeCnt && L1 == 0){ //when we are at the altitude column, copy number into "altitude"
                         buff = Column.c_str();
                         altitude = strtod(buff,NULL);
                         //Serial.println(altitude);
                      }
               
         
             Column = "";  //Assign empty string to Column
             commaCnt += 1;
             
         }    //end if comma check
       
         
        
        if (c =='\n'){ //when there is a new line do this

            L1 = 0; //Reset heading line indicator
            commaCnt = 0; //Reset comma count
            lineCnt +=1; //Increment line count
            Column = ""; //Assign empty string to Column    IS THIS REDUNDANT?
          
            if (L1 == 0){
              GetVectorData();      //Retrieves sensor (compass and accelerometer) data
              DefineInertAxes();    //Defines an inertial frame of reference using the magnetic north vector read from the compass
              DefineChangeBasis();    //Defines a change of basis matrix used to convert the magnetic north vector from the compass coordinate system to the inertial frame of reference

              //Gives direction/amount that servo motor should pan this iteration, POSITIVE -> RIGHT, NEGATIVE -> LEFT
              ThetaPan = atan2(getDistance(latORG, lonORG, latORG, lon), getDistance(latORG, lonORG, lat, lonORG)) - ThetaPanRef;         
              //Gives direction/amount that servo motor should tilt this iteration, POSITIVE -> UP, NEGATIVE -> DOWN
              ThetaTilt = GetThetaXZ(getDistance(latORG, lonORG, lat, lon), altitude) - ThetaTiltRef;

              if(DEBUG)
              {
//                Serial.print("testLat: ");
//                Serial.println(lat, 8);
//                Serial.print("testLon: ");
//                Serial.println(lon, 8);
//                Serial.print("testAltitude: ");
//                Serial.println(altitude, 4);
//
//                Serial.print("testLatORG: ");
//                Serial.println(latORG, 8);
//                Serial.print("testLonORG: ");
//                Serial.println(lonORG, 8);
//                Serial.print("testAltORG: ");
//                Serial.println(altORG, 4);
//                
//                Serial.print("testyCoord: ");
//                Serial.println(yCoord, 3);
//                Serial.print("testzCoord: ");
//                Serial.println(zCoord, 3);
//                Serial.print("testHyp: ");
//                Serial.println(Hyp);
//                Serial.print("testThetaXY: ");
//                Serial.println(ThetaXY);
//                Serial.print("testThetaXZ: ");
//                Serial.println(ThetaXZ);
              }
              
              else{
                SetPan(ThetaPan);
                SetTilt(ThetaTilt);
              }
              //Servo::refresh();
            }
         
        }//endif new line check
         
        
        //endif usable char check
        
        
        //Serial.print(c);
    
        
      }//endif client available

  // As long as there are bytes in the serial queue,
  // read them and send them out the socket if it's open:
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    if (client.connected()) {
      client.print(inChar); 
    }
  }

  // if the server's disconnected, stop the client:
  if (!client.connected()) {
    Serial.println();
    Serial.println("disconnecting.");
    client.stop();
    
    //Reset servo pan motors to midpoint before shutting down, useful for quick setup next time
    servoPan.writeMicroseconds(SERVO_INITIALIZE);  
    servoTilt.writeMicroseconds(SERVO_INITIALIZE);
    // do nothing:
    
    while(true);
  }
  
}

