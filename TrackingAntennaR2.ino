/*
  Telnet client
 
 This sketch connects to a a telnet server (http://www.google.com)
 using an Arduino Wiznet Ethernet shield.  You'll need a telnet server 
 to test this with.
 Processing's ChatServer example (part of the network library) works well, 
 running on port 10002. It can be found as part of the examples
 in the Processing application, available at 
 http://processing.org/
 
 Circuit:
 * Ethernet shield attached to pins 10, 11, 12, 13
 

 */
#include <SPI.h>
#include <Ethernet.h>
#include <math.h>
#include <Servo.h> 


// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {  
  0x90,0xA2,0xDA,0x0F,0x2C,0x9A};
IPAddress ip(169,254,61,56);

// Enter the IP address of the server you're connecting to:
IPAddress server(169,254,61,55); 

// Initialize the Ethernet client library
// with the IP address and port of the server 
// that you want to connect to (port 23 is default for telnet;
// if you're using Processing's ChatServer, use  port 10002):
EthernetClient client;

//Servo Objects
Servo myservo1;  // create servo object to control a servo 
Servo myservo2;  // create servo object to control a servo 


void setup() {
  // attaches the servos
 // myservo1.attach(9);  //change the pins connected
 // myservo2.attach(10);
  
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
const char *buff = Column.c_str();

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

double olat = 0; 
double olong = 0;
double oaltitude = 0;


//Gear Ratios
int GeRxy = 1; //xy pan gear ratio
int GeRyz = 7;// yz tilt gear ratio

//Angles
int ThetaXY = 0;
int ThetaYZ = 0;
int CurrentTheta = 0;

//Hypoteneuse of x,y coordinates
 double Hyp = 0;




// ---------------------------------FOR GPS CALCULATION FUNCTIONS --------------------------------------
 double xCoord; 
 double yCoord; 

void getXYCoordinates(double longitude, double latitude){
    xCoord = getDistance(olat, olong, olat, longitude);//Longitude relative to (0,0)
    yCoord = getDistance(olat, olong, latitude, olong);
    Hyp = sqrt(sq(xCoord) + sq(yCoord)); //set the hypoteneuse to calculate the angle needed to tilt up/down
}

float getDistance(double lat1, double lon1, double lat2, double lon2){ //in meters
    int EARTH_RADIUS = 6371; //km
    double dtLat = radians(lat2 - lat1);
    double dtLon = radians(lon2 - lon1);

    float a = sin(dtLat / 2) * sin(dtLat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dtLon / 2) * sin(dtLon / 2);
    float c = (2 * atan2(sqrt(a),sqrt(1 - a))) * 1000;
    float d = EARTH_RADIUS * c;
    return d;

}

//--------------------------------------------------------Servo Functions---------------------------------------------------------------



int GetTheta (long double x, long double y){
  int Theta = 0;
  
    if (x >0 && y >0){
      Theta = atan(y/x);
    }
    
    if (x <0 && y >0){
      Theta = 180 - atan(y/abs(x));
    }
    
    if (x >0 && y <0){
      Theta = 360 - atan(abs(y)/x);
    }
    
    if (x <0 && y <0){
      Theta = 180 + atan(abs(y)/abs(x));
    }
  
  return Theta;
  
}


              
 
int pos = 0;    // variable to store the servo position 
 

void SetPan(int ThetaXY){
 if (myservo1.read() != ThetaXY){
    myservo1.write(ThetaXY);              // tell servo to go to position in variable 'pos' 
    delay(15);     // waits 15ms for the servo to reach the position
    }
}


void SetTilt(int ThetaYZ){
  if (myservo2.read() != ThetaYZ){
    myservo2.write(ThetaYZ);              // tell servo to go to position in variable 'pos' 
    delay(15);     // waits 15ms for the servo to reach the position
    }                     // waits 15ms for the servo to reach the position
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
                         buff = Column.c_str();
                         lat = strtod(buff,NULL);
                         //Serial.println(lat);
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
                olat = 49.8;
                olong = -98.25;
                oaltitude = 1.4375;
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
              ThetaXY = GetTheta ( xCoord, yCoord);
              ThetaYZ = GetTheta (Hyp, altitude);
              
              //SetPan(ThetaXY);
              //SetTilt(ThetaYZ);
              
             
              Serial.println(ThetaXY);
              Serial.println(ThetaYZ);
              
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



