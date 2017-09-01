//
//  map.hpp
//  Path_Planning
//
//  Created by Steve Horton on 8/30/17.
//
//
#ifndef map_hpp
#define map_hpp


#include <stdio.h>
#include <math.h>
#include <vector>

using namespace std;


double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);


#endif /* map_hpp */

