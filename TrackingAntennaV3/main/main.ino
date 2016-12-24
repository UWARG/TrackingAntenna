#include "Logger.hpp"
#include "Network.hpp"

void setup(){
  initDebug();
  info("Starting up...");
  
  initNetwork();
  parseHeaders();
}

void loop(){
  parsePacket();
  Serial.println(network_data.alt*1000);
  Serial.println(network_data.lon*1000);
  Serial.println(network_data.lat*1000);
}

