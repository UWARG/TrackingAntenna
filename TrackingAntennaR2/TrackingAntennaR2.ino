/*

 This sketch connects to a a telnet server (http://www.google.com)
 using an Arduino Wiznet Ethernet shield.  You'll need a telnet server 
 to test this with.

 Circuit:
 * Ethernet shield attached to pins 10, 11, 12, 13
 
  servoPan is the pan servo, pwm to pin 5 (xy) .
  servoTilt is the tilt servo, pwm6 (z).


Servo Information: 
Model No. HITEC HS-785HB
    
    Information taken from: http://www.robotshop.com/ca/en/hitec-hs785hb-servo-motor.html
    
    PWM 600 -> angle 0
    PWM 2400 -> angle 1260
    
    Center = PWM 1500 -> angle 630
    1 degree = PWM 1.429
    
 */
 
#include <SPI.h>
#include <Ethernet.h>
#include <math.h>
#include <Servo.h> 

#define M_PI 3.14159265358979323
#define SERVO_MIDPOINT 1500
#define EARTH_RADIUS 6371000 //in m
#define DEGREE_TO_PWM 0.7
#define GEAR_RATIO_YZ 7  //how much it is geared down by
#define GEAR_RATIO_XY 1

/****           INITIALIZATION VALUES       *************/

#define INITIAL_LATTITUDE 0
#define INITIAL_LONGITUDE 0
#define INITIAL_ALTITUDE  0
		
// Enter a MAC address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {0x90,0xA2,0xDA,0x0F,0x2C,0x9A};
IPAddress ip(192,168,1,199); //this is for the arduino


// Enter the IP address of the server you're connecting to:
IPAddress server(192,168,1,104);  //pi or computer that is hosting the data

#define INITIALIZATION 0  //0 if you want to track, 1 if you want to go to the home position. home position should be set to point straight east. 

#define DEBUG 1  //turns on the printing to serial

/****        END OF INITIALIZATION      **********/


// Initialize the Ethernet client library
// with the IP address and port of the server 
// that you want to connect to (port 23 is default for telnet;
// if you're using Processing's ChatServer, use  port 10002):
EthernetClient client;

//Servo Objects
Servo servoPan;  // create servo object to control a servo 
Servo servoTilt;  // create servo object to control a servo 

void setup() {
  // attaches the servos
  //usage: servo.attach(pin, minPWM, maxPWM)
  servoPan.attach(5, 600, 2400);
  servoTilt.attach(6, 600, 2400);
  
  // start the Ethernet connection:
  Ethernet.begin(mac,ip);
 // Open serial communications and wait for port to open:
  Serial.begin(9600);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }


  // give the Ethernet shield a second to initialize:
  delay(1000);
  Serial.println("connecting...");

  // if you get a connection, report back via serial:
  if (client.connect(server, 1234)) {
    Serial.println("connected");
  }
  else {
    // if you didn't get a connection to the server:
    Serial.println("connection failed");
  }
 
}


//-----------------------------------VARIABLES FOR MAIN LOOP----------------------------

//counts
int commaCnt = 0;
int lineCnt = 0;

//string of data
String Data = "";
String Column = "";
const char *buff = Column.c_str(); //was const char

//comma positions of important data
int latCnt = 0;
int longCnt = 0;
int altitudeCnt = 0;

//check bit for first line
boolean L1 = 1;//line 1 check to assign commaCNTs to important columns
boolean L2 = 1;//line 2 check to assign the origin


//lat, long
double lat = 0; 
double lon = 0;
double altitude = 0;

//want to change all these to precompiler directives...
double olat = 0; 
double olong = 0;
double oaltitude = 0;

//Angles
int ThetaXY = 0;
int ThetaYZ = 0;
int currentTheta = 50;

//Hypoteneuse of x,y coordinates
double Hyp = 0;

unsigned long servoPanLastWrite = 0;
unsigned long servoTiltLastWrite = 0;


// ---------------------------------FOR GPS CALCULATION FUNCTIONS --------------------------------------
 double xCoord; 
 double yCoord; 

void getXYCoordinates(double longitude, double latitude){
    xCoord = getDistance(olat, olong, olat, longitude);//Longitude relative to (0,0)
    yCoord = getDistance(olat, olong, latitude, olong);
    Hyp = getDistance(olat, olong, latitude, longitude);
    //Hyp = sqrt(sq(xCoord) + sq(yCoord)); //set the hypoteneuse to calculate the angle needed to tilt up/down
}

float getDistance(double lat1, double lon1, double lat2, double lon2){ //in meters

    double dtLat = (lat2 - lat1) * M_PI / 180;
    double dtLon = (lon2 - lon1) * M_PI / 180;

    float a = sin(dtLat / 2) * sin(dtLat / 2) + cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) * sin(dtLon / 2) * sin(dtLon / 2);
    return EARTH_RADIUS * (2 * atan2(sqrt(a),sqrt(1 - a)));
}

//--------------------------------------------------------Servo Functions---------------------------------------------------------------



int GetThetaXY (long double x, long double y)  // this method causes a overflow condtion, needs to change (wont go from 1 to -1, it will go from 1 to 359)
{
  int Theta = 0;
  
    if(lat > olat && lon > olong){
      Theta = atan(y/x) * 180 / M_PI;
    }
    
    if(lat > olat && lon < olong){
      Theta = 180 - atan(y/abs(x)) * 180 / M_PI;
    }
    
    if(lat < olat && lon > olong){
      Theta = 360 - atan(abs(y)/x) * 180 / M_PI;
    }
    
    if(lat < olat && lon < olong){
      Theta = 180 + atan(abs(y)/abs(x)) * 180 / M_PI;
    }
  
  return Theta;
  
}

int GetThetaYZ (long double x, long double y){
  return degrees(atan(y/x)); 
}
              
 
int pos = 0;    // variable to store the servo position 


void SetPan(int ThetaXY)
{
    //
    if(servoPanLastWrite + 15 > millis())    //give the servo time to move without stopping the entire program
    //{
      servoPan.writeMicroseconds(SERVO_MIDPOINT - ThetaXY * DEGREE_TO_PWM);
      //servoPanLastWrite = millis();
    //}
}


void SetTilt(int ThetaYZ)
{    
  //if(servoTiltLastWrite + 15 > millis())
  //{
    servoTilt.writeMicroseconds(abs(SERVO_MIDPOINT -  ThetaYZ * GEAR_RATIO_YZ * DEGREE_TO_PWM));
    //servoTiltLastWrite = millis();
  //}   
}




//-------------------------------------MAIN------------------------------------------------------

void loop()
{
      // if there are incoming bytes available 
      // from the server, read them and print them:
      if (client.available()) {
        char c = client.read();
        Data = String(c);
        //Serial.print(c);
        
        if (c == ','){ //Whenever char that is read is a comma do this
        
                      if (L1 == 1){
                           if (Column == "lat"){  //sets the comma count of each important column
                             latCnt = 0;//commaCnt;
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
                         //buff = Column.c_str();//comment this line out when using in the field
                         buff = Column.substring(0, 10).c_str();//comment this line in when using in the field
                         String tempBuff = Column.substring(2);//comment this line in when using in the field
                         buff = tempBuff.c_str();
                         //buff[0] = ' ';
                         lat = strtod(buff,NULL);
                         //Serial.println(tempBuff);
                         //Serial.println(buff);
                     }
                       
                     if (commaCnt == longCnt && L1 == 0){ //when we are at the longitude column, copy number into "longitude"
                         buff = Column.c_str();
                         lon = strtod(buff,NULL);
                         //Serial.println(lon);
                     }
                     if (commaCnt == altitudeCnt && L1 == 0){ //when we are at the altitude column, copy number into "altitude"
                         buff = Column.c_str();
                         altitude = strtod(buff,NULL);
                         //Serial.println(altitude);
                     }
               
         
             Column = "";  //delete what's in the string
             commaCnt += 1;
             
         }    //end if comma check
       
         
        
        if (c =='\n'){ //when there is a new line do this
        
             
              
              if (L2 == 1 && L1 == 0 ){
                olat = INITIAL_LATTITUDE;
                olong = INITIAL_LONGITUDE;
                oaltitude = INITIAL_ALTITUDE;
                L2 = 0;//check bit for second line line
                         // Serial.println(lat);
                         // Serial.println(lon);
                          //Serial.println(altitude);
              }
              
               if (L1 ==1){
                    L1=0;
               }
          
          commaCnt = 0;
          lineCnt +=1;
          Column = "";
          
              if (L2 == 0 && L1 ==0){
              getXYCoordinates(lon,lat); //set cartesian coordinates
              ThetaXY = GetThetaXY ( xCoord, yCoord);
              ThetaYZ = GetThetaYZ ( Hyp, altitude - oaltitude);        //was missing the offset for initial altitude?
              
              if(INITIALIZATION)
              {
                servoPan.writeMicroseconds(SERVO_MIDPOINT);
                servoTilt.writeMicroseconds(SERVO_MIDPOINT);
              } else {
                SetPan(ThetaXY);
                servoTilt.writeMicroseconds(SERVO_MIDPOINT);
                //SetTilt(ThetaYZ);
              }
              
              if(DEBUG)
              {
                Serial.print("lat: ");
                Serial.print(lat, 8);
                Serial.print("\tlon: ");
                Serial.print(lon, 8);
                Serial.print("\taltitude: ");
                Serial.print(altitude, 4);
                
                Serial.print("\txCoord: ");
                Serial.print(xCoord, 3);
                Serial.print("\tyCoord: ");
                Serial.print(yCoord, 3);
                Serial.print("\tHyp: ");
                Serial.print(Hyp);
                Serial.print("\tThetaXY: ");
                Serial.print(ThetaXY);
                Serial.print("\tThetaYZ: ");
                Serial.println(ThetaYZ);
              }
              //Servo::refresh();
            }
         
        }//endif new line check
         
        if (c != ','){ //as long as the read char is not a ',' do this
         Column += Data;
        }//endif usable char check
        
        
       //Serial.print(c);
    
        
      }//endif client available

  // as long as there are bytes in the serial queue,
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
    // do nothing:
    while(true);
  }
  
}
//


