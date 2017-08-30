//
//  utility.cpp
//  Path_Planning
//
//  Created by Steve Horton on 8/29/17.
//
//
#include <math.h>  // Contains M_PI



#include "utility.hpp"



// In-line functions for converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double meters2miles(double x) { return x * 0.000621371; }
double miles2meters(double x) { return x * 1609.3440; }
double mps2mph(double x) { return x * 2.236936292; }
