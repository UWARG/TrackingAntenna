#include "Logger.hpp"
#include "Network.hpp"


void setup(){
  initDebug();

  debug("Starting up");
  
  initNetwork();
  parseHeaders();
}

void loop(){
  parsePacket();
  Serial.println(network_data.alt);
  Serial.println(network_data.lon);
  Serial.println(network_data.lat);
}

