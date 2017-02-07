#include "Logger.hpp  "
#include "Network.hpp"
<<<<<<< HEAD
#include "GPS.hpp"
=======
#include "Accelerometer.hpp"
#include "Magnetometer.hpp"
>>>>>>> refs/remotes/origin/master

void setup(){
    initDebug();
    info("Starting up...");
<<<<<<< HEAD
    initGPS();
    //initNetwork();
=======

    initNetwork();
    initDebug();
    initMagnetometer();
    initAccelerometer();
>>>>>>> refs/remotes/origin/master
}

void loop(){
  /*
    if(parsePacket()){
      //using serial.println instead of debug() since debug takes a string as parameter
      Serial.println(network_data.alt);
      Serial.println(network_data.lon);
      Serial.println(network_data.lat);
    }
<<<<<<< HEAD
    renewNetwork();
    */
    parseGPS();

    Serial.print(gps_location.lat);
    Serial.print(" ");
    Serial.print(gps_location.lon);
    Serial.print(" ");
    Serial.println(gps_location.alt);
    
=======
   
    renewNetwork(); 
>>>>>>> refs/remotes/origin/master
    delay(100);
}
