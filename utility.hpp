//
//  utility.hpp
//  Path_Planning
//
//  Created by Steve Horton on 8/29/17.
//
//

#ifndef utility_hpp
#define utility_hpp

#include <stdio.h>

// In-line functions for converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);
double meters2miles(double x);
double miles2meters(double x);
double mps2mph(double x);



#endif /* utility_hpp */
