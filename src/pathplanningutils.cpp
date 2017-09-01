//
//  pathplanningutils.cpp
//  Path_Planning
//
//  Created by Steve Horton on 9/1/17.
//
//
#include <cmath>   // for abs of double!
#include <array>
#include <vector>

#include "pathplanningutils.hpp"
#include "trackedcar.hpp"


// Finds & returns the closest car in a lane just ahead of sdc from vector of Tracked Cars using Frenet delta s
void get_min_ahead_cars(vector<TrackedCar> &tracked_cars, TrackedCar &min_delta_s_tracked_car) {
    
    double min_delta_s = __DBL_MAX__;         // Start w/ large #
    vector<TrackedCar>::iterator it_save;  // Save pointer to min object (de-activate)
    
    if (tracked_cars.empty()) {
        // min_delta_s_tracked_car.set_delta_s(__DBL_MAX__); // This is flag that no delta_s (maybr skip_
        min_delta_s_tracked_car = {}; // <TODO> THIS IS DIFFERENT THAN COUNTER PART ABOVE
        //cout << "tracked cars AHEAD for this lane is empty!" << endl;
        return;
    }
    
    
    for (vector<TrackedCar>::iterator it = tracked_cars.begin(); it!=tracked_cars.end(); ++it) {
        
        if (it->delta_s <= min_delta_s) {
            min_delta_s = it->delta_s;  // Compare delta_s in TrackedCar object to find min
            it_save = it;               // Iterator of min object by delta_s
            
            min_delta_s_tracked_car.id = it->id;
            min_delta_s_tracked_car.x = it->x;
            min_delta_s_tracked_car.y = it->y;
            min_delta_s_tracked_car.vx = it->vx;
            min_delta_s_tracked_car.vy = it->vy;
            min_delta_s_tracked_car.s = it->s;
            min_delta_s_tracked_car.d = it->d;
            
            // Calculated
            min_delta_s_tracked_car.v = it->v;
            min_delta_s_tracked_car.delta_s = it->delta_s;
            
            // Speed
            min_delta_s_tracked_car.speed_mph = it->speed_mph;
            min_delta_s_tracked_car.future_speed_mph = it->future_speed_mph;
            
        }
        //min_delta_s_tracked_car = (*it);
    } // for
    
    //cout << "min AHEAD  delta_s=" << min_delta_s << "," << min_delta_s_tracked_car.delta_s << endl;
}


// Finds and returns the closest car in a lane just behind sdc from vector of Tracked Cars using Frenet delta s
void get_min_behind_cars(vector<TrackedCar> &tracked_cars, TrackedCar &min_delta_s_tracked_car) {
    
    //double min_delta_s = __DBL_MAX__;         // Start w/ large #
    double min_delta_s = -__DBL_MAX__;         // Start w/ large #
    vector<TrackedCar>::iterator it_save;  // Save pointer to min object
    
    
    if (tracked_cars.empty()) {
        min_delta_s_tracked_car = {}; // <TODO> THIS IS DIFFERENT THAN COUNTER PART ABOVE
        // cout << "tracked cars BEHIND for this lane is empty!" << endl;
        return;
    }
    
    
    
    for (vector<TrackedCar>::iterator it = tracked_cars.begin(); it!=tracked_cars.end(); ++it) {
        
        //if (abs(it->delta_s) <= min_delta_s) { /// THIS IS DIFFERENCE TO ABOVE !!!
        if (it->delta_s >= min_delta_s) { /// THIS IS DIFFERENCE TO ABOVE !!!
            min_delta_s = it->delta_s;  // Compare delta_s in TrackedCar object
            it_save = it;               // Iterator of min object by delta_s
            
            min_delta_s_tracked_car.id = it->id;
            min_delta_s_tracked_car.x = it->x;
            min_delta_s_tracked_car.y = it->y;
            min_delta_s_tracked_car.vx = it->vx;
            min_delta_s_tracked_car.vy = it->vy;
            min_delta_s_tracked_car.s = it->s;
            min_delta_s_tracked_car.d = it->d;
            
            // Calculated
            min_delta_s_tracked_car.v = it->v;
            min_delta_s_tracked_car.delta_s = it->delta_s;
            
            // Speed
            min_delta_s_tracked_car.speed_mph = it->speed_mph;
            min_delta_s_tracked_car.future_speed_mph = it->future_speed_mph;
        }
        
        //min_delta_s = min(min_delta_s, it->delta_s);  // Compare delta_s in TrackedCar object
        //it_save = it;
        //cout << "it,it_save=" << &it << "," << &it_save << endl;
        //cout << "min behind delta_s=" << it->delta_s << "," << min_delta_s_tracked_car.delta_s << endl;
        
    } // for
    
    //cout << "min BEHIND delta_s=" << min_delta_s << "," << min_delta_s_tracked_car.delta_s << endl;
    
    //min_delta_s_tracked_car = *it_save;
}



