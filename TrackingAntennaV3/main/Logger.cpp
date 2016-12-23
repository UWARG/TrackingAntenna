/**
 * @file   Logger.cpp
 * @Author Serj Babayan (WARG)
 * @date   December 20, 2016
 */

void initDebug(void){
	#if DEBUG_MESSAGES
	Serial.begin(BAUD_RATE);

	// wait for serial port to connect
	while (!Serial);
	
	#endif
}

void debug(const char* message){
	#if DEBUG_MESSAGES
		Serial.println(message);
	#endif
}
