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


#define COMPASS_ID 12345
#define Size 3   //Size of square matrices and length of vectors
#define NorthCalibrate false            //Indicates when north direction must be manually set ("true" if setting manually, "false" if compass is automating process)
#define SouthCalibrate false            //Indicates when compass is calibrated improperly, use manually input south angle to calibrate ("true" if setting manually, "false" if compass is properly calibrated)
//Manual calibration offsets
#define NorthOffset 7                  //Added to "HeadingTrue" to point compass at true north, if calibration isn't working properly (if compass is reading North as West of True North, then NorthOffset > 0)
#define TrueSouth 173                  //Angle given by compass when pointing to True South, if calibration isn't working properly (TrueSouth reading taken should be colinear with NorthOffset reading)


//Define calibration values for magnetometer, Really Thorough
#define MinMag_X -48.36
#define MinMag_Y -55.73
#define MinMag_Z -54.08
#define MaxMag_X 66.36
#define MaxMag_Y 60.73
#define MaxMag_Z 75.20

//-----------------------------------GLOBAL VARIABLES----------------------------

/***      COMPASS & ACCELEROMETER     ***/
//Declare sensor variables
sensors_event_t accelEvent;   //Accelerometer Data
sensors_event_t magEvent;     //Magnetic Compass Data

//Declination values: South Port Manitoba on 04/30/2016 = 4.136deg E, Waterloo Ontario on 03/09/2016 = 9.633deg W, Home on 03/29/2016 = 9.265 W, Flying Dutchmen flight center on 04/10/2016 = -9.624 W
//East declinations are POSITIVE, West declinations are NEGATIVE
float declination = -9.624;

//Declare vectors for sensor calculations
//float Gravity[Size];
float NegGravity[Size];                 //Vertical direction (negative of gravity)
float MagNorthComp[Size];               //Magnetic north in compass frame of reference
float HeadingAnt[Size];                 //Heading of antenna in horizontal plane of inertial frame of reference, colinear with x-axis of compass projected into horizontal inertial plane
float UnitInertX[Size];                 //X-Axis in inertial frame of reference
float UnitInertY[Size];                 //Y-Axis in inertial frame of reference
float UnitInertZ[Size];                 //Z-Axis in inertial frame of reference
float ChangeBasis[Size][Size];          //Change of basis matrix for converting vectors from compass frame of reference to inertial frame of reference, multiply compass vector on left
float RevertBasis[Size][Size];          //Change of basis matrix for converting vectors from inertial frame of reference to compass frame of reference, multiply compass vector on left
float SoftIronTransform[Size][Size];
float BiasVec[Size];
float HeadingTrue = 0;                //Heading relative to true north for compass test

//--------------------------------------------------------Matrix Functions---------------------------------------------------------------

//Multiplication by a scalar
void ScalarMult(float *pMatA, float constant, int m, int n, float *pMatRes) {
  int i, j;

  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) {
      pMatRes[n * i + j] = constant * pMatA[n * i + j];
    }
  }
}

//Dot Product
float DotProduct(float *pVecA, float *pVecB, int SizeA, int SizeB) {
  int i;

  if (SizeA == SizeB) {
    float result = 0;

    for (i = 0; i < SizeA; i++) {
      result += pVecA[i] * pVecB[i];
    }
    return result;
  }
}

//Dot Product Projection, projects vector B onto vector A
float DotProductProject(float *pVecA, float *pVecB, int SizeA, int SizeB, float *pProj) {
  if (SizeA == SizeB) {
    float DotAB = (DotProduct((float*)pVecA, (float*)pVecB, SizeA, SizeA));
    float MagnitudeA_SQRD = (DotProduct((float*)pVecA, (float*)pVecA, SizeA, SizeA));
    float Quotient = DotAB / MagnitudeA_SQRD;
    ScalarMult((float*)pVecA, Quotient, SizeA, 1, (float*)pProj);
  }
}

//Dot Angle, calculates the angle between two vectors
float DotProductAngle(float *pVecA, float *pVecB, int SizeA, int SizeB) {
  float Dot = DotProduct((float*)pVecA, (float*)pVecB, SizeA, SizeA);
  float MagnitudeA = sqrt(DotProduct((float*)pVecA, (float*)pVecA, SizeA, SizeA));
  float MagnitudeB = sqrt(DotProduct((float*)pVecB, (float*)pVecB, SizeA, SizeA));
  float Theta = acos(Dot / (MagnitudeA * MagnitudeB)) * 180 / M_PI;

  return Theta;
}

//Cross Product
void CrossProduct_1x3(float *pVecA, float *pVecB, int SizeA, int SizeB, float *resultVec) {
  int i, j;
  resultVec[SizeA];

  if (SizeA == SizeB && SizeA == 3) {
    j = 1;

    for (i = 0; i < SizeA; i++) {
      if (j == 2 && i == 1) {
        resultVec[i] = pVecA[j] * pVecB[j - 2] - pVecA[j - 2] * pVecB[j];
        j = 0;
      }
      else {
        resultVec[i] = pVecA[j] * pVecB[j + 1] - pVecA[j + 1] * pVecB[j];
        j++;
      }
    }
  }
}

//Project vector onto 2D plane in Rn                                            //METHOD FROM http://people.ucsc.edu/~lewis/Math140/Ortho_projections.pdf
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
void ProjectVecToPlane(float *pVec, float *pBase1, float *pBase2, int Rn, float *pProjVec) {
  int i, j;
  float Matrix_A[Rn][2];
  float Matrix_ATr[2][Rn];
  float Matrix_ATrxA[2][2];
  float Product_AxATrxA[Rn][2];
  float ProjectionMatrix[Rn][Rn];

  //Construct A from two linearly independent basis vectors
  for (i = 0; i < Rn ; i++) {
    Matrix_A[i][0] = pBase1[i];
  }
  for (j = 0; j < Rn ; j++) {
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
void Normalize(float *pVec, int VecSize, float *pNorm) {
  int i;

  for (i = 0; i < VecSize; i++) {
    pNorm[i] = pVec[i] / sqrt(DotProduct((float*)pVec, (float*)pVec, VecSize, VecSize));
  }
}

//-----------------------------------INITIALIZATION VALUES----------------------------

//Initializes the compass library with an arbitrarily set compass id
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(COMPASS_ID); //Create compass object to read magnetic values
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(COMPASS_ID); //Create compass object to read acceleration values

void setup() {
  /* Enable auto-gain */
  mag.enableAutoRange(true);

  //Initialize the compass
  if (!mag.begin()) {
    Serial.println("\nCompass failed to be detected.");
  }
  //Initialize the accelerometer
  if (!accel.begin()) {
    Serial.println("\nAccelerometer failed to be detected.");
  }

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    // wait for serial port to connect. Needed for Leonardo only
  }

  //Set up soft iron matrix and bias vector for calibration of magnetometer data
  //Define soft iron transform matrix
  //Magnetic Field Vector of Earth is 54.041 uT. Calculated at University of Waterloo on 04/18/2016
  SoftIronTransform[0][0] = 1.0859;
  SoftIronTransform[0][1] = -0.0058;
  SoftIronTransform[0][2] = -0.0126;
  SoftIronTransform[1][0] = -0.0058;
  SoftIronTransform[1][1] = 1.0618;
  SoftIronTransform[1][2] = -0.0086;
  SoftIronTransform[2][0] = -0.0126;
  SoftIronTransform[2][1] = -0.0086;
  SoftIronTransform[2][2] = 1.1312;       //Seems to be some discrepancy in the Y direction with these numbers (X and Z are good)
  Matrix.Invert((float*)SoftIronTransform, 3);

  //Define bias vector
  BiasVec[0] = 2.5808;
  BiasVec[1] = -4.0168;
  BiasVec[2] = -32.0504;
}

//--------------------------------------------------------Compass Functions---------------------------------------------------------------

//Create magnetic north and acceleration vectors
//void GetVectorData(float *pMagNorthComp, float *pNegGravity){
void GetVectorData(float *pBiasVec, float *pSoftIronTransform, float *pMagNorthComp, float *pNegGravity) {
  float RawMinusBias[Size];
  //Create magnetic north vector
  mag.getEvent(&magEvent);

  //  //Calibrate magnetic data
  //  //Hard iron calibration difference
  //  long double HardIron_X = (MaxMag_X + MinMag_X) / 2;
  //  long double HardIron_Y = (MaxMag_Y + MinMag_Y) / 2;
  //  long double HardIron_Z = (MaxMag_Z + MinMag_Z) / 2;
  //
  //  //Soft iron calibration factor
  //  long double Avg_X = (MaxMag_X - MinMag_X) / 2;
  //  long double Avg_Y = (MaxMag_Y - MinMag_Y) / 2;
  //  long double Avg_Z = (MaxMag_Z - MinMag_Z) / 2;
  //  long double Avg_Total = (Avg_X + Avg_Y + Avg_Z) / 3;
  //  long double ScaleFactor_X = Avg_Total / Avg_X;
  //  long double ScaleFactor_Y = Avg_Total / Avg_Y;
  //  long double ScaleFactor_Z = Avg_Total / Avg_Z;
  //
  //  pMagNorthComp[0] = (magEvent.magnetic.x - HardIron_X) * ScaleFactor_X;
  //  pMagNorthComp[1] = (magEvent.magnetic.y - HardIron_Y) * ScaleFactor_Y;
  //  pMagNorthComp[2] = (magEvent.magnetic.z - HardIron_Z) * ScaleFactor_Z;

  //  //Assign and calibrate magnetic data
  //  pMagNorthComp[0] = (magEvent.magnetic.x - MinMag_X) / (MaxMag_X - MinMag_X) * 2 - 1;
  //  pMagNorthComp[1] = (magEvent.magnetic.y - MinMag_Y) / (MaxMag_Y - MinMag_Y) * 2 - 1;
  //  pMagNorthComp[2] = (magEvent.magnetic.z - MinMag_Z) / (MaxMag_Z - MinMag_Z) * 2 - 1;

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

  //  Gravity[0] = accelEvent.acceleration.x;   //FOR SOME REASON THE ACCELEROMETER IS GIVING GRAVITY AS POINTING UP
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
void DefineInertAxes(float *pMagNorthComp, float *pNegGravity, float *pUnitInertX, float *pUnitInertY, float *pUnitInertZ) {
  //Define vertical axis in inertial frame of reference
  float InertZ[Size];
  DotProductProject((float*)pNegGravity, (float*)pMagNorthComp, Size, Size, (float*)InertZ);    //Project magnetic compass vector onto vertical gravity vector
  if (DotProductAngle((float*)pNegGravity, (float*)pMagNorthComp, Size, Size) > 90) {           //Check whether magnetic compass vector points above or below horizontal to make sure z-axis is defined as up
    ScalarMult((float*)InertZ, -1, 3, 1, (float*)InertZ);
  }
  Normalize((float*)InertZ, Size, (float*)pUnitInertZ);                                         //Normailze to produce unit vector
  //  if(DEBUG){
  //    Matrix.Print((float*)InertZ, Size, 1, "Inertial Z-Axis");
  //    Matrix.Print((float*)pUnitInertZ, Size, 1, "Normalized Inertial Z-Axis");
  //}

  //Define forward axis in horizontal plane of inertial frame of reference, using Gram-Schmidt process
  float InertX[Size];
  if (DotProductAngle((float*)pNegGravity, (float*)pMagNorthComp, Size, Size) > 90) {
    Matrix.Add((float*)pMagNorthComp, (float*)InertZ, Size, 1, (float*)InertX);
  }
  else {
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
void ProjectCompX(float *pChangeBasis, float *pUnitInertX, float *pUnitInertY, float *pHeadingAnt) {
  const float CompassX[Size] = {1, 0, 0};
  float ProjCompassX[Size];
  ProjectVecToPlane((float*)CompassX, (float*)pUnitInertX, (float*)pUnitInertY, Size, (float*)ProjCompassX);    //Project compass x-axis into horizontal plane
  Matrix.Multiply((float*)pChangeBasis, (float*)ProjCompassX, Size, Size, 1, (float*)pHeadingAnt);              //Convert projected compass x-axis to inertial frame of reference
  //  if(DEBUG){
  //    Matrix.Print((float*)pHeadingAnt, Size, 1, "Compass X-Axis Projected into Horizontal Plane and Converted to Inertial Coordinates");   //Z component should be zero
  //    Matrix.Print((float*)pHeadingAnt, Size, 1, "Antenna Heading in Inertial Coordinates");   //Z component should be zero
  //  }
}

//Create change of basis matrix
void DefineChangeBasis(float *pAxisX, float *pAxisY, float *pAxisZ, float *pMatrix, float *pMatrixRevert) {
  int i, j, k;

  for (i = 0; i < Size ; i++) {
    pMatrixRevert[i * Size] = pAxisX[i];
  }
  for (j = 0; j < Size ; j++) {
    pMatrixRevert[j * Size + 1] = pAxisY[j];
  }
  for (k = 0; k < Size ; k++) {
    pMatrixRevert[k * Size + 2] = pAxisZ[k];
  }
  //Copy matrix to produce change of basis matrix for reverting back to compass coordinate system
  Matrix.Copy((float*)pMatrixRevert, Size, Size, (float*)pMatrix);
  //Invert matrix to produce change of basis matrix for converting from compass to inertial coordinates
  //We know ChangeBasis is invertible because it is a square matrix and it is a basis (linearly independent)
  Matrix.Invert((float*)pMatrix, Size);   //Argument becomes result, so ChangeBasis is replaced by its inverted form
}

//Calculate angular antenna heading, relative to magnetic north
//Clockwise (E) is positive
float AntennaHeadingTrue(float *pChangeBasis, float *pUnitInertX, float *pHeadingAnt) {
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
  if (CheckDir[2] < 0) {
    ThetaOffset = -1 * ThetaOffset;
  }
  //  if(DEBUG){
  //    Matrix.Print((float*)CheckDir, Size, 1, "CheckDir Vector");
  //    Serial.print("\nCompass Heading Relative to Magnetic North (float) : "); Serial.println(ThetaOffset);
  //  }

  //Calculate offset from true north using declination
  float HeadingTrue = ThetaOffset + declination;

  //Manually set North direction
  if (NorthCalibrate) {
    HeadingTrue = HeadingTrue + NorthOffset;
  }

  if (HeadingTrue < -180) {
    HeadingTrue = HeadingTrue + 360;
  }

  if (HeadingTrue > 180) {
    HeadingTrue = HeadingTrue - 360;
  }

  //Calibrate compass so that it reads North and South as colinear
  //Take reading after setting NorthOffset
  if (SouthCalibrate) {
    float SouthRatio = 360 - ((180.0 * 180.0) / abs(TrueSouth));
    if (TrueSouth < 0) {
      if (HeadingTrue < TrueSouth) {
        HeadingTrue = HeadingTrue * (180.0 / abs(TrueSouth)) + 360;
      }
      else if (HeadingTrue > 0) {
        HeadingTrue = HeadingTrue * (SouthRatio / 180.0);
      }
      else {
        HeadingTrue = HeadingTrue * (180.0 / abs(TrueSouth));
      }
    }
    if (TrueSouth > 0) {
      if (HeadingTrue > TrueSouth) {
        HeadingTrue = HeadingTrue * (180.0 / abs(TrueSouth)) - 360;
      }
      else if (HeadingTrue < 0) {
        HeadingTrue = HeadingTrue * (SouthRatio / 180.0);
      }
      else {
        HeadingTrue = HeadingTrue * (180.0 / abs(TrueSouth));
      }
    }
  }

  if (HeadingTrue < -180) {
    HeadingTrue = HeadingTrue + 360;
  }

  if (HeadingTrue > 180) {
    HeadingTrue = HeadingTrue - 360;
  }

  //Return antenna heading as an angle, CW relative to true north
  return HeadingTrue;
}

void loop() {
    //Retrieves sensor (compass and accelerometer) data
    GetVectorData((float*)BiasVec, (float*)SoftIronTransform, (float*)MagNorthComp, (float*)NegGravity);
    //Defines an inertial frame of reference using the magnetic north vector read from the compass
    DefineInertAxes((float*)MagNorthComp, (float*)NegGravity, (float*)UnitInertX, (float*)UnitInertY, (float*)UnitInertZ);
    //Defines a change of basis matrix used to convert the magnetic north vector from the compass coordinate system to the inertial frame of reference
    DefineChangeBasis((float*)UnitInertX, (float*)UnitInertY, (float*)UnitInertZ, (float*)ChangeBasis, (float*)RevertBasis);
  
    //Converts compass x-axis to inertial frame of reference, then projects converted vector into horizontal plane
    ProjectCompX((float*)ChangeBasis, (float*)UnitInertX, (float*)UnitInertY, (float*)HeadingAnt);
    HeadingTrue = AntennaHeadingTrue((float*)ChangeBasis, (float*)UnitInertX, (float*)HeadingAnt);

    Serial.print("\nCompass Heading Relative to True North : "); Serial.println(HeadingTrue,0);                                       //COMPASS HEADING OPERATES ACCEPTABLY, MUST INCLUDE FUNCTION TO MAP ANGLES SO THAT SOUTH IS COLINEAR WITH NORTH

    delay(3000);
}



