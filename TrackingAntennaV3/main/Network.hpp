/**
 * @file   Network.hpp
 * @Author Serj Babayan (WARG)
 * @date   December 20, 2016
 * @brief  Network initialization and parsing functions.
 *         Connection based off of the Arduino ethernet library.
 *		     Uses DHCP so no need to know the IP of the shield once connected.
 * @see https://www.arduino.cc/en/Reference/Ethernet
 */
#ifndef NETWORK
#define NETWORK

/**
 *IP adress of the data relay station and port
 */
#define DATA_RELAY_PORT 1234

#define DATA_RELAY_IP1 192
#define DATA_RELAY_IP2 168
#define DATA_RELAY_IP3 1
#define DATA_RELAY_IP4 102

/**
 * Data relay header names for altitude, latitude, and longitude
 */
#define ALTITUDE_HEADER_NAME "altitude"
#define LATITUDE_HEADER_NAME "lat"
#define LONGITUDE_HEADER_NAME "lon"

/**
 * Mac address of the ethernet shield/interface
 */
#define MAC_ADDR_1 0x90
#define MAC_ADDR_2 0xA2
#define MAC_ADDR_3 0xDA
#define MAC_ADDR_4 0x0F
#define MAC_ADDR_5 0x2C
#define MAC_ADDR_6 0x9A

typedef struct {
    float alt;
    float lat;
    float lon;
} NetworkData;

/**
 * Global variable representing the current state of the network data parsed.
 * Includes altitude, longitude, and longitude, which is all we really care about
 * for the tracking antenna.
 */
extern NetworkData network_data;

/**
 * Initializes the ethernet connection, attempts to connect to the data relay,
 * and parses headers
 */
void initNetwork(void);

/**
 * Attempts to renew the DHCP lease with the router, only when necessary.
 * Can be called as many times as possible, it will only perform the re-request
 * if one is required.
 * 
 * In addition checks if the connection to the data relay server is still active.
 * If not, will attempt to re-initialize it.
 */
void renewNetwork(void);

/**
 * Same as parseHeaders, only will parse an incoming packet stream representing the data.
 * Will update network_data global variable with the newly parsed altitude, latitude, and
 * longitude information. Will parse 1 whole entire packet synchronously. Requires that
 * parseHeaders be called before.
 * 
 * Returns true if values for altitude, longitude, and latitude were parsed from a packet.
 */
bool parsePacket(void);

#endif
