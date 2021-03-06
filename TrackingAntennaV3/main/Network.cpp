/**
 * @file   Network.cpp
 * @Author Serj Babayan (WARG)
 * @date   December 20, 2016
 */

#include "Network.hpp"
#include "Logger.hpp"
#include <string.h>
#include <stdlib.h>
#include <math.h>

//represents the current state of the altitude, latitude, and longitude
//defined globally
GPSLocation network_data;

//initialize the Ethernet client library
static EthernetClient client;

//define index positions of the packet that we're interested in.
//initialize to -1 so we can later see if the appropriate headers were detected
static int alt_index = -1;
static int lat_index = -1;
static int lon_index = -1;

//ethernet interface mac address
static byte mac[] = {MAC_ADDR_1,MAC_ADDR_2,MAC_ADDR_3,MAC_ADDR_4,MAC_ADDR_5,MAC_ADDR_6};

static IPAddress data_relay_ip(DATA_RELAY_IP1, DATA_RELAY_IP2, DATA_RELAY_IP3, DATA_RELAY_IP4);

static void printConnectionStatusMessage(int status);
static bool allHeadersReceived(void);
static void connectDataRelay(void);
static void parseHeaders(void);

void initNetwork(void){
    int connection_status = 0;

    info("Attempting to configure ethernet...");

    //initialize our network connection. If not successful keep trying
    while(Ethernet.begin(mac) == 0) {
        error("Failed to configure Ethernet using DHCP, trying again...");
    }

    debug("Connected to network with IP: ");
    char string[16];
    IPAddress ip = Ethernet.localIP();
    ipToString(&ip, string);
    debug(string);

    //Give the Ethernet shield a second to initialize:
    delay(1000);
    connectDataRelay();
    parseHeaders();

}

bool parsePacket(void){
    char c; //character we've just read
    int buffer_counter = 0;
    int lines_read = 0;
    char data_buffer[64]; //nothing we read will be larger than 64 bytes

    if(client.available() == 0){
        return false;
    }

    while(client.available()) {
        c = client.read();

        if (c == ',') { //if we just read a comma in, means we can interpret the word/header
            data_buffer[buffer_counter] = 0; //terminate our c string with the null character

            //check if the index of the data we just read corresponds to the altitude, longitude, or altitude
            //if so convert the received string to float and save the corresponding values
            if(lines_read == alt_index){
                double tmp = atof(data_buffer);
                if (tmp != 0.0) {
                    network_data.alt = tmp;
                }
            } else if(lines_read == lat_index){
                double tmp = atof(data_buffer);
                if (tmp != 0.0) {
                    network_data.lat = tmp;
                }
            } else if(lines_read == lon_index){
                double tmp = atof(data_buffer);
                if (tmp != 0.0) {
                    network_data.lon = tmp;
                }
            }
            buffer_counter = 0;
            lines_read++;
        } else if (c == '\n') { //we've reached the end of the packet, break out of the loop
            buffer_counter = 0;
            lines_read = 0;
            break;
        } else if (c == 0) { //if we read the null character, ignore it
            buffer_counter++;
        } else {
            data_buffer[buffer_counter] = c;
            buffer_counter++;
        }
    }

    return true;
}

void renewNetwork(){
    //renew DHCP lease if necessary
    Ethernet.maintain();

    //check if we're still connected to the data relay
    if(!client.connected()){
        client.stop();
        connectDataRelay();
        parseHeaders();
    }
}

/**
 * Will parse the headers received from the data relay. Will parse the entire
 * received packet, and save the indices of the headers corresponding to altitude,
 * longitude, and latitude. Should be called right after network initialization.
 * Is required for correct packet parsing to occur. Will wait until headers are received.
 */
static void parseHeaders(void){
    char c; //character we've just read
    int buffer_counter = 0;
    int lines_read = 0;
    char data_buffer[64]; //nothing we read will be larger than 64 bytes

    debug("Waiting for headers");
    while(!client.available());

    info("Receiving and parsing headers");

    while(client.available()){
        c = client.read();

        if(c == ','){ //if we just read a comma in, means we can interpret the word/header
            data_buffer[buffer_counter] = 0; //terminate our c string with the null character

            if(strcmp(data_buffer, ALTITUDE_HEADER_NAME) == 0){ //if the header we just parsed was for altitude
                alt_index = lines_read;
            } else if(strcmp(data_buffer, LATITUDE_HEADER_NAME) == 0){ //if the header we just parsed was for latittude
                lat_index = lines_read;
            } else if(strcmp(data_buffer, LONGITUDE_HEADER_NAME) == 0){ //if the header we just parsed was for longitude
                lon_index = lines_read;
            }

            buffer_counter = 0;
            lines_read++;
        } else if (c == '\n'){ //we've reached the end of the packet, break out of the loop
            buffer_counter = 0;
            lines_read = 0;

            debug("Headers finished parsing.");

            //stop execution if we werent able to parse all the headers, since any incoming data will be useless
            if(!allHeadersReceived()){
                while(1);
            }
            return;
        } else {
            data_buffer[buffer_counter] = c;
            buffer_counter++;
        }
    }
}

/**
 * Initializes a connection with the data relay station.
 * Will continually attempt to do so if unsuccessful
 */
static void connectDataRelay(void){
    int connection_status = 0;

    //try to connect to data relay, and keep trying if not successful
    while(connection_status != 1){
        info("Attempting to connect to data relay server...");
        connection_status = client.connect(data_relay_ip, DATA_RELAY_PORT);
        printConnectionStatusMessage(connection_status);

        //wait a second before trying to connect again
        if(connection_status != 1){
            delay(1000);
        }
    }
}

/**
 * For warning the user if we were unable to parse the positions of the altitude,
 * latitude, and longitude from the first header packet.
 * Returns a boolean indicating if all 3 headers were successfully parsed.
 */
static bool allHeadersReceived(void){
    bool success = true;
    if(alt_index == -1){
        error("Altitude header position could not be parsed from data relay packet!!!");
        success = false;
    }

    if(lat_index == -1){
        error("Latitude header position could not be parsed from data relay packet!!!");
        success = false;

    }

    if(lon_index == -1){
        error("Longitude header position could not be parsed from data relay packet!!!");
        success = false;
    }

    return success;
}

//Prints out a status message depending on the connection status
static void printConnectionStatusMessage(int status){
    // if you get a connection, report back via serial:
    if (status == 1) {
        info("Sucessfully connected to Data Relay");
    }
    else {  // if you didn't get a connection to the server:
        error("Connection Failed! Reason:");
        if(status == -1){
            error("Connection Timed Out");
        }
        else if(status == -2){
            error("Invalid Server");
        }
        else if(status == -3){
            error("Truncated");
        }
        else if(status == -4){
            error("Invalid Response");
        } else {
            error("Unknown Reason");
        }
    }
}
