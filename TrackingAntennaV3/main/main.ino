#include "Logger.hpp"
#include "Network.hpp"

void setup(){
    initDebug();
    info("Starting up...");

    initNetwork();
}

void loop(){
    if(parsePacket()){
      
    }
    Serial.println(network_data.alt);
      Serial.println(network_data.lon);
      Serial.println(network_data.lat);
   
    renewNetwork();
}

