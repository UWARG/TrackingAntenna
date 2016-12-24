#include "Logger.hpp"
#include "Network.hpp"

void setup(){
    initDebug();
    info("Starting up...");

    initNetwork();
}

void loop(){
    if(parsePacket()){
      //using serial.println instead of debug() since debug takes a string as parameter
      Serial.println(network_data.alt);
      Serial.println(network_data.lon);
      Serial.println(network_data.lat);
    }
   
    renewNetwork();
    delay(1000);
}
