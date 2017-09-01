//
//  costfunctions.cpp
//  Path_Planning
//
//  Created by Steve Horton on 8/30/17.
//
//

//#include "costfunctions.hpp"



//
// More Helper Functions - Going into COST.H!!!
//

/*

double _cost_Collision_Ahead(SelfDrivingCar &sdc, array<vector<Tracked_Vehicle>,NUM_LANES> &cars_ahead, \
                            Tracked_Vehicle &min_ahead_car, int &proposed_lane) {
    
    //Tracked_Vehicle min_ahead_car = {};
    double cost = 0.0;
    
    
    // Cost = 1 if collide, 0 otherwise. Will have large weight factor
    if (cars_ahead[proposed_lane].size() > 0) {
        double delta_car_s =  min_ahead_car.get_future_s() - sdc.get_future_s();
        if (abs(delta_car_s) <= 5.0) {
            cost = 1.0;
            cout << "1. cost: collision ahead=" << min_ahead_car.get_future_s() << "," << sdc.get_future_s() << "," << delta_car_s << "," << 1.0 << endl;
            return cost;
        } else {
            cout << "1. cost: collision ahead=0.0" << endl;
            return 0.0;
        }
    }
    cout << "1. cost: no cars, collision=0.0" << endl;
    return cost;
    
   }


double _cost_Lane_Movement(SelfDrivingCar &sdc, array<vector<Tracked_Vehicle>,NUM_LANES> &cars_ahead, \
                          array<vector<Tracked_Vehicle>,NUM_LANES> &cars_behind, Tracked_Vehicle &min_ahead_car, \
                          Tracked_Vehicle &min_behind_car, int &proposed_lane) {
    
    // Tracked_Vehicle min_ahead_car = {};
    //Tracked_Vehicle min_behind_car = {};
    
    // Set cost to zero and update accordingly. language.. <TODO>
    //double cost = 0.0;
    int lane_movement = proposed_lane - sdc.get_lane();
    
    // Same lane
    if (lane_movement == 0) {
        cout << "2. cost: lane m collision ahead cost=0.0" << endl;
        return 0.0;
    }
    
    // Move right or left
    if ((lane_movement == 1) || (lane_movement == -1))  {
        
        // Check if not clear ahead
        if (cars_ahead[proposed_lane].size() > 0) {
            //get_min_ahead_cars(cars_ahead[proposed_lane], min_ahead_car);
            //min_ahead_car.project_future_self(TIME_AHEAD, proposed_lane);
            double sdc_car_future_ahead_delta_s = min_ahead_car.get_future_s() - sdc.get_future_s();
            if (sdc_car_future_ahead_delta_s <= 30.0) {
                cout << "2. cost: lane m collision ahead=" << min_ahead_car.get_future_s() << "," << sdc.get_future_s() << "," <<sdc_car_future_ahead_delta_s << "," << 1.0 << endl;
                return 1.0;
            }
        }
        
        // Check if not clear behind
        if (cars_behind[proposed_lane].size() > 0) {
            //get_min_behind_cars(cars_behind[proposed_lane], min_behind_car); // Were cars behind updated with future s??
            //min_behind_car.project_future_self(TIME_AHEAD, proposed_lane);
            
            double sdc_car_future_behind_delta_s = sdc.get_future_s() - min_behind_car.get_future_s();
            cout << "debug: " << proposed_lane << "," << min_behind_car.get_future_s() << "," << sdc.get_future_s() << "," \
            << sdc_car_future_behind_delta_s << endl;
            
            if (abs(sdc_car_future_behind_delta_s) <= 15.0) {
                cout << "2. cost: lane m collision behind=" << min_behind_car.get_future_s() << "," << sdc.get_future_s() << "," <<sdc_car_future_behind_delta_s << "," << 1.0 << endl;
                return 1.0;
            }
        }
        
    } else {
        cout << "Error: invalid lane movement=" << lane_movement << endl;
    }
    
    //cout << "cost: move lane collision=" << sdc.get_future_s() << "," << min_ahead_car.get_future_s() << "," << delta_car_s << endl;
    // Clear!
    cout << "2. cost: lane m collision clear=0.0" << endl;
    return 0.0;
}


double _cost_Closest_Vehicle_Ahead(SelfDrivingCar &sdc, array<vector<Tracked_Vehicle>,NUM_LANES> &cars_ahead, \
                                  Tracked_Vehicle &min_ahead_car, int &proposed_lane) {
    
    // Tracked_Vehicle min_ahead_car;
    double cost = 0.0;
    
    // Need to add size check
    if (cars_ahead[proposed_lane].size() > 0) {
        // Cost = 1 for each meter lower than horizon (200m)
        //get_min_ahead_cars(cars_ahead[proposed_lane],min_ahead_car);  //Change function to get_min_car_ahead_from_cars
        //min_ahead_car.project_future_self(TIME_AHEAD, proposed_lane);
        
        double delta_s = min_ahead_car.get_future_s() - sdc.get_future_s();
        cost = min(delta_s,HORIZON);  // Clip at Horizon if above
        cost = max(cost,0.0);         // Clip to zero if negative
        cost = (HORIZON-cost)/HORIZON;          // Normalize [0,1]
        
        cout << "3. cost: closest ahead=" << min_ahead_car.get_future_s() << "," << sdc.get_future_s() <<  "," << delta_s << "," << cost << endl;
        return cost;
    }
    
    cout << "3. cost: closest ahead=0.0" << endl;
    return cost;
}


double _cost_Speed(SelfDrivingCar &sdc, array<vector<Tracked_Vehicle>,NUM_LANES> &cars_ahead, \
                  Tracked_Vehicle &min_ahead_car, int &proposed_lane) {
    
    // For this  to be even more effitive, need to calculate future speed to include either 50 mph if lane free
    // or velocity of min car ahead...
    double cost = 0.0;
    
    // Completely free lane. Full throttle possible - no cost
    if (cars_ahead[proposed_lane].size() == 0) {
        cout << "4. cost: speed (all clear)=" << cost << endl;
        return cost;
    }
    
    // Plenty of room to go full throttle - no cost
    double delta_s = min_ahead_car.get_future_s() - sdc.get_future_s();
    if (delta_s >= 75.0) {
        cout << "4. cost: speed (over 75m out)=" << cost << endl;
        return cost;
    } else {
        cost = (MAX_SPEED - min_ahead_car.get_speed_mph())/MAX_SPEED;
        cout << "4. cost: speed (under 75m out)=" << cost << endl;
        return cost;
    }
    
    // Likely will have to adopt speed of min car ahead - normalized cost to that speed
    
    
    
    
    return cost;
}


double _cost_Number_Of_Vehicles_In_Lane_Ahead(SelfDrivingCar &sdc, array<vector<Tracked_Vehicle>,NUM_LANES> &cars_ahead, int &proposed_lane) {
    
    // NOTE: MAY NEED TO ADD Sophisitication to do out to horizon only
    // Cost is # of cars in lane [0,1,2,3,4...]. Higher # is worse
    double cost = 0.0;
    
    if (cars_ahead[proposed_lane].size() > 0) {
        for (vector<Tracked_Vehicle>::iterator it=cars_ahead[proposed_lane].begin(); it!=cars_ahead[proposed_lane].end(); ++it) {
            double delta_s = it->get_future_s()  - sdc.get_future_s();
            if (delta_s <= HORIZON) cost += 1.0;  // Add one to the cost, if deteted car within Horizon
        }
    }
    
    cout << "5. cost: # of cars in lane=" << proposed_lane << " cost=" << cost << endl;
    return cost;
}


double _cost_Preferred_Lane(int &proposed_lane) {
    
    // Cost is 0=middle, 1=left or right lane (probably should change to more left (fast) lane than right lane??
    double cost = (double) abs(1 - proposed_lane);
    cout << "6. cost: preferred lane=" << cost << endl;
    
    return cost;
}


// NEED TO PASS ARRAYS av1 ,cars_ahead, cars_behind


double _cost_For_Proposed_Trajectory(SelfDrivingCar &sdc, array<vector<Tracked_Vehicle>,NUM_LANES> &cars_ahead, \
                                    array<vector<Tracked_Vehicle>,NUM_LANES> &cars_behind, int proposed_lane) {
    
    Tracked_Vehicle min_ahead_car;
    Tracked_Vehicle min_behind_car;
    double total_cost;
    
    
    // Init
    min_ahead_car = {};
    min_behind_car = {};
    total_cost = 0.0;
    
    // Project closest surronding cars into future
    get_min_ahead_cars(cars_ahead[proposed_lane], min_ahead_car);
    get_min_behind_cars(cars_behind[proposed_lane], min_behind_car);
    
    min_ahead_car.project_future_self(TIME_AHEAD, proposed_lane);
    min_behind_car.project_future_self(TIME_AHEAD, proposed_lane);
    cout << "min cars in future: " << min_ahead_car.get_future_s() << " " << min_behind_car.get_future_s() << endl;
    
    // Calculate total cost. Coefficients are weights of each cost type
    total_cost +=  2000.0 * cost_Collision_Ahead(sdc, cars_ahead, min_ahead_car, proposed_lane);
    total_cost +=  1000.0 * cost_Lane_Movement(sdc, cars_ahead, cars_behind, min_ahead_car, min_behind_car, proposed_lane);
    total_cost +=   150.0 * cost_Closest_Vehicle_Ahead(sdc, cars_ahead, min_ahead_car, proposed_lane); // (like 1.5 cars) 50 100
    total_cost +=    20.0 * cost_Speed(sdc, cars_ahead, min_ahead_car, proposed_lane);                 // New!
    total_cost +=    10.0 * cost_Number_Of_Vehicles_In_Lane_Ahead(sdc, cars_ahead, proposed_lane);     // 5
    total_cost +=     5.0 * cost_Preferred_Lane(proposed_lane);                                        // 10
    
    
    cout << "Total cost for lane,cost=" << proposed_lane << "," << total_cost << endl;
    return total_cost;
}

 */

