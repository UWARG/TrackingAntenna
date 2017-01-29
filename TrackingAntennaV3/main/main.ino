#include "Logger.hpp"
#include "Network.hpp"
#include "GPS.hpp"

void setup(){
    initDebug();
    info("Starting up...");
    initGPS();
    //initNetwork();
}

void loop(){
  /*
    if(parsePacket()){
      //using serial.println instead of debug() since debug takes a string as parameter
      Serial.println(network_data.alt);
      Serial.println(network_data.lon);
      Serial.println(network_data.lat);
    }
    renewNetwork();
    */
    parseGPS();

    Serial.print(gps_location.lat);
    Serial.print(" ");
    Serial.print(gps_location.lon);
    Serial.print(" ");
    Serial.println(gps_location.alt);
    
    delay(100);
}
