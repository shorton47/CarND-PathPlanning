//
//  utils.cpp
//  Path_Planning
//
//  Created by Steve Horton on 8/29/17.
//
//
#include <math.h>

#include "utils.hpp"


// In-line utility functions for converting back and forth between radians/degrees, miles/meters.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double meters2miles(double x) { return x * 0.000621371; }
double miles2meters(double x) { return x * 1609.3440; }
double mps2mph(double x) { return x * 2.236936292; }
double mph2mps(double x) { return(x * 0.447040); }  // Exact conversion mph to m/s
