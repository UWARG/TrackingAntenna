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

/**
 * Data relay header names for altitude, latitude, and longitude
 */
#define ALTITUDE_HEADER_NAME "altitude"
#define LATITUDE_HEADER_NAME "latitude"
#define LONGITUDE_HEADER_NAME "longitude"

/**
 * Mac address of the ethernet shield/interface
 */
#define MAC_ADDR_1 0x90
#define MAC_ADDR_2 0xA2
#define MAC_ADDR_3 0xDA
#define MAC_ADDR_4 0x0F
#define MAC_ADDR_5 0x2C
#define MAC_ADDR_6 0x9A

/**
 * IP of the data relay
 */
#define DATA_RELAY_IP1 192
#define DATA_RELAY_IP2 168
#define DATA_RELAY_IP3 1
#define DATA_RELAY_IP4 150


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
extern volatile NetworkData network_data;

/**
* Initializes the network connection
*/
void initNetwork(void);

/**
 * Attempts to renew the DHCP lease with the router, only when necessary.
 * Can be called as many times as possible, it will only perform the re-request
 * if one is required
*/
void renewNetwork(void);

/**
 * Will parse the headers received from the data relay. Will parse the entire
 * received packet, and save the indices of the headers corresponding to altitude,
 * longitude, and latitude. Should be called right after network initialization.
 * Is required for correct packet parsing to occur. Will wait until headers are received.
 */
void parseHeaders(void);

/**
 * Same as parseHeaders, only will parse an incoming packet stream representing the data.
 * Will update network_data global variable with the newly parsed altitude, latitude, and
 * longitude information. Will parse 1 whole entire packet synchronously. Requires that
 * parseHeaders be called before.
 */
void parsePacket(void);

#endif
