/**
 * @file   Util.cpp
 * @Authors Serj Babayan (WARG)
 * @date   December 22, 2016
 */

/**
 * @brief Converts an IPAdress object into a string and stores it in the
 * second parameter. Useful for displaying IPAddresses in the Serial console
 *
 * @param[in]  ip      { The ip to convert into a string }
 * @param      string  Pointer to a character array of AT LEAST size 16!!
 */

#include "Util.hpp"

void ipToString(const IPAddress* ip, char* string){
    byte oct1 = (*ip)[0];
    byte oct2 = (*ip)[1];
    byte oct3 = (*ip)[2];
    byte oct4 = (*ip)[3];
    sprintf(string, "%d.%d.%d.%d", oct1, oct2, oct3, oct4);
}

float dotProduct(int a1, int a2, int b1, int b2){
	return (a1*b1 + a2*b2);
}

float magnitude(int a1, int a2){
	return (sqrt((a1*a1) + (a2*a2)));
}

float mapf(float input, float in_min, float in_max, float out_min, float out_max) {
   return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

