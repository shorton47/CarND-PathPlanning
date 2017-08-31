//
//  utils.hpp
//  Path_Planning
//
//  Created by Steve Horton on 8/29/17.
//
//

#ifndef utils_hpp
#define utils_hpp

//#include <stdio.h>

#include <math.h>
// In-line utility functions for converting back and forth between radians/degrees, miles/meters.
//constexpr double pi();

/*
double pi();
double deg2rad(double x);
double rad2deg(double x);
double meters2miles(double x);
double miles2meters(double x);
double mps2mph(double x);
double mph2mps(double x);
*/

constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double meters2miles(double x) { return x * 0.000621371; }
double miles2meters(double x) { return x * 1609.3440; }
double mps2mph(double x) { return x * 2.236936292; }
double mph2mps(double x) { return(x * 0.447040); }  // Exact conversion mph to m/s



#endif /* utils_hpp */


