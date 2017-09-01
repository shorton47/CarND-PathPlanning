//
//  pathplanningutils.hpp
//  Path_Planning
//
//  Created by Steve Horton on 9/1/17.
//
//

#ifndef pathplanningutils_hpp
#define pathplanningutils_hpp

//#include <stdio.h>
#include <cmath>   // for abs of double!
#include <array>
#include <vector>

#include "trackedcar.hpp"


// Finds & returns the closest car in a lane just ahead of sdc from vector of Tracked Cars using Frenet delta s
void get_min_ahead_cars(vector<TrackedCar> &tracked_cars, TrackedCar &min_delta_s_tracked_car);


// Finds and returns the closest car in a lane just behind sdc from vector of Tracked Cars using Frenet delta s
void get_min_behind_cars(vector<TrackedCar> &tracked_cars, TrackedCar &min_delta_s_tracked_car);



#endif /* pathplanningutils_hpp */
