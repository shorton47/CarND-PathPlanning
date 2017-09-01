//
//  costfunctions.cpp
//  Path_Planning
//
//  Created by Steve Horton on 8/30/17.
//
//
#include <cmath>   // for abs of double!
#include <array>
#include <vector>

// My libraries & classes
#include "costfunctions.hpp"

#include "selfdrivingcar.hpp"
#include "trackedcar.hpp"
#include "constants.hpp"
#include "pathplanningutils.hpp"


using namespace std;


//---
// Individual Cost Functions
//---

// Cost for potential collision ahead & behind. 0.0=ok, 1.0 or 2.0=collision risk based on safety distances
double cost_Collision_Risk(SelfDrivingCar &sdc, array<vector<TrackedCar>,NUM_LANES> &cars_ahead,  \
                                                array<vector<TrackedCar>,NUM_LANES> &cars_behind, \
                                                TrackedCar &min_ahead_car, TrackedCar &min_behind_car, int &proposed_lane) {
    
#define AHEAD_COLLISION_RISK   10.0 // (meters) 15.0 worked
#define BEHIND_COLLISION_RISK  10.0 // (meters)
    
    // Default
    double cost = 0.0;
    
    
    // Look ahead
    if (cars_ahead[proposed_lane].size() > 0) {
        double future_delta_s =  min_ahead_car.get_future_s() - sdc.get_future_s();
        if (abs(future_delta_s) <= AHEAD_COLLISION_RISK) {
            cost = 1.0;
            cout << "1. cost: collision ahead=" << min_ahead_car.get_future_s() << "," << sdc.get_future_s() << "," \
            << future_delta_s << "," << cost << endl;
            //return cost;
        }
    }
    
    // Look behind
    if (cars_behind[proposed_lane].size() > 0) {
        double future_delta_s =  min_behind_car.get_future_s() - sdc.get_future_s();
        if (abs(future_delta_s) <= BEHIND_COLLISION_RISK) {
            cost = 1.0;
            cout << "1. cost: collision behind=" << min_behind_car.get_future_s() << "," << sdc.get_future_s() << "," \
            << future_delta_s << "," << cost << endl;
            //return cost;
        }
    }
    
    cout << "1. cost: collision total=" << cost << endl;
    return cost;
}


// Cost for potential lane change. 0.0=ok, 1.0 or 2.0=no or right lane
double cost_Lane_Movement(SelfDrivingCar &sdc, array<vector<TrackedCar>,NUM_LANES> &cars_ahead, \
                          array<vector<TrackedCar>,NUM_LANES> &cars_behind, TrackedCar &min_ahead_car, \
                          TrackedCar &min_behind_car, int &proposed_lane) {
    
#define AHEAD_SEPERATION  20.0  // 30 working (meters) Helps box-in recover (Tuned at 50 mph - scales inversely with speed down
#define BEHIND_SEPERATION 20.0 // 10 was working, 20 now with scale code 12.5 good, 15 was working (meters)  SHOULD BE LARGER?
    
    
    double cost = 0.0;
    
    //double cost = 0.0; // <TODO> update return w/ costs
    int lane_movement = proposed_lane - sdc.get_lane();
    
    
    // Same lane
    if (lane_movement == 0) {
        cout << "2. cost: lane m collision ahead cost=" << cost << endl;
        return cost;
    }
    
    // Move right or left
    if ((lane_movement == 1) || (lane_movement == -1))  {
        
        // Check if not clear ahead
        if (cars_ahead[proposed_lane].size() > 0) {
            double future_ahead_delta_s = min_ahead_car.get_future_s() - sdc.get_future_s();
            double velocity_scale = min_behind_car.get_speed_mph()/sdc.get_car_speed();
            double scaled_seperation = AHEAD_SEPERATION*velocity_scale;
            if (abs(future_ahead_delta_s) <= scaled_seperation) {
                cost += 1.0;
                cout << "2. cost: lane m collision ahead=" << min_ahead_car.get_future_s() << "," << sdc.get_future_s() << "," \
                << future_ahead_delta_s << "," << cost << endl;
                //return 1.0;
            }
        }
        
        // Check if not clear behind
        if (cars_behind[proposed_lane].size() > 0) {
            double future_behind_delta_s = sdc.get_future_s() - min_behind_car.get_future_s();
            double velocity_scale = min_behind_car.get_speed_mph()/sdc.get_car_speed();
            double scaled_seperation = BEHIND_SEPERATION*velocity_scale;
            cout << "debug: " << proposed_lane << "," << min_behind_car.get_future_s() << "," << sdc.get_future_s() << "," \
            << future_behind_delta_s << endl;
            //if (abs(future_behind_delta_s) <= BEHIND_SEPERATION) {
            if (abs(future_behind_delta_s) <= scaled_seperation) {
                cost += 1.0;
                cout << "2. cost: lane m collision behind=" << min_behind_car.get_future_s() << "," << sdc.get_future_s() \
                << "," <<future_behind_delta_s << "," << scaled_seperation << "," << cost << endl;
                return 1.0;
            }
        }
        
    } else {
        cout << "Error: invalid lane movement=" << lane_movement << endl;
    }
    
    // Sum of Both checks (also covers if tracked car moves from behind to forward & visa versa in projection!
    cout << "2. cost: lane m collision risk=" << cost << endl;
    return cost;
}

// Cost is [0-1] based on 1.0-% of speed.  Higher # is worse
double cost_Closest_Vehicle_Ahead(SelfDrivingCar &sdc, array<vector<TrackedCar>,NUM_LANES> &cars_ahead, \
                                  TrackedCar &min_ahead_car, int &proposed_lane) {
    
    // TrackedCar min_ahead_car;
    double cost = 0.0;
    
    // Need to add size check
    if (cars_ahead[proposed_lane].size() > 0) {
        double delta_s = min_ahead_car.get_future_s() - sdc.get_future_s();
        cost = min(delta_s,HORIZON);   // Clip at Horizon if above
        cost = max(cost,0.0);          // Clip to zero if negative
        cost = (HORIZON-cost)/HORIZON; // Normalize [0,1]
        
        cout << "3. cost: closest ahead=" << min_ahead_car.get_future_s() << "," << sdc.get_future_s() <<  "," \
        << delta_s << "," << cost << endl;
        return cost;
    }
    
    cout << "3. cost: closest ahead=0.0" << endl;
    return cost;
}


// Cost is [0-1] based on 1.0-% of speed.  Higher # is worse
double cost_Speed(SelfDrivingCar &sdc, array<vector<TrackedCar>,NUM_LANES> &cars_ahead, \
                  TrackedCar &min_ahead_car, int &proposed_lane) {
    
#define FULL_THROTTLE_DISTANCE 75.0  // (meters)
    
    
    // Start w/ no cost
    double cost = 0.0;
    
    // Completely free lane. Full throttle possible - no cost
    if (cars_ahead[proposed_lane].size() == 0) {
        cout << "4. cost: speed (all clear)=" << cost << endl;
        return cost;
    }
    
    // Plenty of room to go full throttle - no cost
    double delta_s = min_ahead_car.get_future_s() - sdc.get_future_s();
    if (delta_s >= FULL_THROTTLE_DISTANCE) {
        cout << "4. cost: speed (over 75m)=" << cost << endl;
        return cost;
        
        // Use speed of min car ahead - normalized cost to that speed
    } else {
        cost = (MAX_SPEED - min_ahead_car.get_speed_mph())/MAX_SPEED;
        cout << "4. cost: speed (under 75m)=" << cost << endl;
        return cost;
    }
    
    
    // Old Speed
    /*
     double cost = (MAX_SPEED - sdc.get_future_speed())/MAX_SPEED;
     cout << "4. cost: speed=" << cost << endl;
     */
    
    return cost;
}


// Cost is # of cars in lane [0,1,2,3,4...]. Higher # is worse
double cost_Number_Of_Vehicles_In_Lane_Ahead(SelfDrivingCar &sdc, array<vector<TrackedCar>,NUM_LANES> &cars_ahead, \
                                             int &proposed_lane) {
    
    // Start w/ no cost
    double cost = 0.0;
    
    if (cars_ahead[proposed_lane].size() > 0) {
        for (vector<TrackedCar>::iterator it=cars_ahead[proposed_lane].begin(); it!=cars_ahead[proposed_lane].end(); ++it) {
            double delta_s = it->get_future_s()  - sdc.get_future_s();
            if (delta_s <= HORIZON) cost += 1.0;  // Add one to the cost, if deteted car within Horizon
        }
    }
    
    cout << "5. cost: # of cars in lane=" << proposed_lane << " cost=" << cost << endl;
    return cost;
}


// Cost is 0.0=Second from Left, 1.0=left or right lane
double cost_Preferred_Lane(int &proposed_lane) {
    
    double cost = (double) abs(1 - proposed_lane);
    cout << "6. cost: preferred lane cost=" << cost << endl;
    
    return cost;
}

// In case of ties, prefer far left "fast" lane or in general most left lane
double cost_Fast_Lane(int &proposed_lane) {
    
    double cost = (double) proposed_lane;
    cout << "7. cost: fast lane cost=" << cost << endl;
    
    return cost;
}


//---
// Total Cost Function
//---

// Calculate total cost for a proposed trajectory.  Note: the weights are in front of each function
double cost_For_Proposed_Trajectory(SelfDrivingCar &sdc, array<vector<TrackedCar>,NUM_LANES> &cars_ahead, \
                                    array<vector<TrackedCar>,NUM_LANES> &cars_behind, int proposed_lane) {
    
    TrackedCar min_ahead_car;
    TrackedCar min_behind_car;
    double total_cost;
    
    
    // Init
    min_ahead_car = {};
    min_behind_car = {};
    total_cost = 0.0;
    
    // Get closest cars to SDC Project closest surronding cars into future
    get_min_ahead_cars(cars_ahead[proposed_lane], min_ahead_car);
    get_min_behind_cars(cars_behind[proposed_lane], min_behind_car);
    
    // Project into future
    min_ahead_car.project_future_self(TIME_AHEAD, proposed_lane);
    min_behind_car.project_future_self(TIME_AHEAD, proposed_lane);
    cout << "lane " << proposed_lane << " min car ahead : now, future: " << min_ahead_car.get_s() << "  " \
    << min_ahead_car.get_future_s() << " " << endl;
    cout << "lane " << proposed_lane << " min car behind: now, future: " << min_behind_car.get_s() << "  " \
    << min_behind_car.get_future_s() << " " << endl;
    //cout << "min cars in future: " << min_ahead_car.get_future_s() << " " << min_behind_car.get_future_s() << endl;
    
    // Calculate total cost. Coefficients are weights of each cost type
    total_cost +=  1000.0 * cost_Collision_Risk(sdc, cars_ahead, cars_behind, min_ahead_car, min_behind_car, proposed_lane);
    total_cost +=  1000.0 * cost_Lane_Movement(sdc, cars_ahead, cars_behind, min_ahead_car, min_behind_car, proposed_lane);
    total_cost +=   150.0 * cost_Closest_Vehicle_Ahead(sdc, cars_ahead, min_ahead_car, proposed_lane); // (like 1.5 cars) 50 100
    total_cost +=    20.0 * cost_Speed(sdc, cars_ahead, min_ahead_car, proposed_lane);                 // New!
    total_cost +=    10.0 * cost_Number_Of_Vehicles_In_Lane_Ahead(sdc, cars_ahead, proposed_lane);     // 5
    total_cost +=     5.0 * cost_Preferred_Lane(proposed_lane);                                        // 10
    total_cost +=     1.0 * cost_Fast_Lane(proposed_lane);
    
    cout << "Total cost for lane,cost=" << proposed_lane << "," << total_cost << endl;
    return total_cost;
}


