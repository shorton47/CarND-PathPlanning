//
//  utils.hpp
//  Path_Planning
//
//  Created by Steve Horton on 8/29/17.
//
//

#ifndef utils_hpp
#define utils_hpp

#include <stdio.h>

#endif /* utils_hpp */


// In-line utility functions for converting back and forth between radians/degrees, miles/meters.
//constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);
double meters2miles(double x);
double miles2meters(double x);
double mps2mph(double x);
double mph2mps(double x); 
