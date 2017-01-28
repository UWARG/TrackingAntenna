/**
 * @file   Util.cpp
 * @Authors Serj Babayan, Michael Lenover (WARG)
 * @date   December 22, 2016
 */

/**
 * @brief Converts an IPAdress object into a string and stores it in the
 * second parameter. Useful for displaying IPAddresses in the Serial console
 * 
 *        Converts x and y values to angle with respect to x axis
 *
 * @param[in]  ip      { The ip to convert into a string }
 * @param      string  Pointer to a character array of AT LEAST size 16!!
 */

#include "Util.hpp"
#include <Math.h>

void ipToString(const IPAddress* ip, char* string){
    byte oct1 = (*ip)[0];
    byte oct2 = (*ip)[1];
    byte oct3 = (*ip)[2];
    byte oct4 = (*ip)[3];
    sprintf(string, "%d.%d.%d.%d", oct1, oct2, oct3, oct4);
}

float vectorToAngle(float x, float y){
  return tan(y / x);
}

