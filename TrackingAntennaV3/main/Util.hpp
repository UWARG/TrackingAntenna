/**
 * @file   Util.hpp
 * @Author Serj Babayan (WARG)
 * @date   December 22, 2016
 * @brief A variety of utility functions that dont belong anywhere else.
 */
#ifndef UTIL
#define UTIL

#include <Ethernet.h>

void ipToString(const IPAddress* ip, char* string);

float dotProduct(int a1, int a2, int b1, int b2);

float magnitude(int a1, int a2);

float mapf(float input, float inMin, float inMax, float outMin, float outMax);

#endif
