/**
 * @file   Network.cpp
 * @Author Serj Babayan (WARG)
 * @date   December 20, 2016
 */
 
#include "Network.hpp"

//represents the current state of the altitude, latitude, and longitude
//defined globally
volatile NetworkData network_data;

//initialize the Ethernet client library
static EthernetClient client;
static char data_buffer[100]; //we're not going to receive anything greater than 100 bytes

//define index positions of the data we're interested in the packets that we're receiving
static int alt_index = 0;
static int lat_index = 0;
static int lon_index = 0;
static int lines_read = 0; //for keeping track of how many lines/terms/headers in a packet we've parsed
static int buffer_counter = 0; //index position counter for the data buffer

//names of the packet headers
static const char altitude_header[] = ALTITUDE_HEADER_NAME;
static const char latitude_header[] = LATITUDE_HEADER_NAME;
static const char longitude_header[] = LONGITUDE_HEADER_NAME;

//ethernet interface mac address
static byte mac[] = {MAC_ADDR_1,MAC_ADDR_2,MAC_ADDR_3,MAC_ADDR_4,MAC_ADDR_5,MAC_ADDR_6};

static IPAddress data_relay_ip(DATA_RELAY_IP1, DATA_RELAY_IP2, DATA_RELAY_IP3, DATA_RELAY_IP4);

static void printConnectionStatusMessage(int status);

void initNetwork(void){
	int connection_status = 0;

  debug("Attempting to configure ethernet...");
  
	//initialize our network connection. If not successful keep trying
  while(Ethernet.begin(mac) == 0) {
    debug("Failed to configure Ethernet using DHCP, trying again...");
  }
  
  debug("Connected to network with IP: ");
  IPAddress localAddr = Ethernet.localIP();
  byte oct1 = localAddr[0];
  byte oct2 = localAddr[1];
  byte oct3 = localAddr[2];
  byte oct4 = localAddr[3];
  char s[16];  
  sprintf(s, "%d.%d.%d.%d", oct1, oct2, oct3, oct4);
  debug(s);

  //Give the Ethernet shield a second to initialize:
  delay(1000);
  
  //try to connect to data relay, and keep trying if not successful
  while(connection_status != 1){
    debug("Attempting to connect to data relay server...");
    connection_status = client.connect(data_relay_ip, DATA_RELAY_PORT);
    printConnectionStatusMessage(connection_status);
  }
}

void renewNetwork(){
	Ethernet.maintain();
}

void parsePacket(void){
	char c; //character we've just read

	while(client.available()){
		c = client.read();

		if(c == ','){ //if we just read a comma in, means we can interpret the word/header
			data_buffer[buffer_counter] = 0; //terminate our c string with the null character
			
			lines_read++;

			//check if the index of the data we just read corresponds to the altitude, longitude, or altitude
			//if so convert the received string to float and save the corresponding values
			if(lines_read == alt_index){
				network_data.alt = atof(data_buffer);
			} else if(lines_read == lat_index){
				network_data.lat = atof(data_buffer);
			} else if(lines_read == lon_index){
				network_data.lon = atof(data_buffer);
			}

			buffer_counter = 0;
		} else if (c == '\n'){ //we've reached the end of the packet, break out of the loop
			buffer_counter = 0;
			lines_read = 0;
			return;
		} else {
			data_buffer[buffer_counter] = c;
			buffer_counter++;
		}
	}
}

void parseHeaders(void){
	char c; //character we've just read

	debug("Waiting for headers");
	while(!client.available());

	debug("Receiving and parsing headers");

	while(client.available()){
		c = client.read();

		if(c == ','){ //if we just read a comma in, means we can interpret the word/header
			data_buffer[buffer_counter] = 0; //terminate our c string with the null character
			
			if(strcmp(data_buffer, altitude_header) == 0){ //if the header we just parsed was for altitude
				alt_index = lines_read;
			}

			if(strcmp(data_buffer, latitude_header) == 0){ //if the header we just parsed was for latittude
				lat_index = lines_read;
			}

			if(strcmp(data_buffer, longitude_header) == 0){ //if the header we just parsed was for longitude
				lon_index = lines_read;
			}

			buffer_counter = 0;
			lines_read++;
		} else if (c == '\n'){ //we've reached the end of the packet, break out of the loop
			buffer_counter = 0;
			lines_read = 0;
			return;
		} else {
			data_buffer[buffer_counter] = c;
			buffer_counter++;
		}
	}
}

//Prints out a status message depending on the connection status
static void printConnectionStatusMessage(int status){
	// if you get a connection, report back via serial:
  if (status == 1) {
    debug("Sucessfully connected to Data Relay");
  }
  else {  // if you didn't get a connection to the server:
    debug("Connection Failed! Reason:");
    if(status == -1){
      debug("Connection Timed Out");
    }
    else if(status == -2){
      debug("Invalid Server");
    }
    else if(status == -3){
      debug("Truncated");
    }
    else if(status == -4){
      debug("Invalid Response");
    } else {
      debug("Unknown Reason");
    }
  }
}
