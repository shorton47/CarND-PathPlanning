//
//  selfdrivingcar.hpp
//  Path_Planning
//
//  Created by Steve Horton on 8/31/17.
//
//



#ifndef selfdrivingcar_hpp
#define selfdrivingcar_hpp

//#ifndef selfdrivingcar_H
//#define selfdrivingcar_H


#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include "spline.h"  // Copyright (C) 2011, 2014 Tino Kluge (ttk448@gmail.com)

// My libraries
//#include "selfdrivingcar.hpp"
#include "map.hpp"
#include "utils.hpp"


using namespace std;


class SelfDrivingCar{
    
#define FULL_SPEED_TRIGGER_POINT     30.0  // (meters)
#define MAINTAIN_SPEED_TRIGGER_POINT 10.0  // (meters)
    
#define MAX_SPEED  49.87125       // !49.87 49.86 works 49.80 49.88 was too high, (49.875,49.8725) too high for max accel speed variations
#define LANE_WIDTH   4.0       // Width of each lane (meters)
    
    
    
    
public:
    enum class State {Emergency, LaneChangeLeft, KeepLane, LaneChangeRight};
    const char State_Name[4][20] = {"Emergency" , "LaneChangeLeft", "KeepLane", "LaneChangeRight"};
    
private:
    // Canonical
    double sdc_x;   // add units!!
    double sdc_y;
    double sdc_s;
    double sdc_d;
    double sdc_yaw;
    double sdc_speed;
    double sdc_endpath_s;
    double sdc_endpath_d;
    
    // Determined
    SelfDrivingCar::State sdc_state;
    SelfDrivingCar::State sdc_next_state;
    
    // External control
    int sdc_lane;
    double sdc_ref_vel; // This is DIFFERENT than speed. This is velocity setting sent to the Simulator Server
    
    // Future self for FSM Behavior analysis
    double sdc_future_s;
    double sdc_future_speed;
    double sdc_future_d;
    int    sdc_future_lane;
    
    // SDC status
    bool sdc_lane_change_in_process;
    
public:
    // Default Constructor
    SelfDrivingCar();
    
    // Destructor
    virtual ~SelfDrivingCar();
    
    
    //---
    // Helper Methods
    //---
    
    // Feasible future States to switch to for a given lane from defined Finite State Machine (FSM)
    vector<SelfDrivingCar::State> get_feasible_next_States();
    
    
    // Feasible future lanes to switch to for a given SDC State from defined Finite State Machine (FSM)
    vector<int> get_feasible_next_Lanes(SelfDrivingCar::State &next_state);
    
    // Project self driving car into a future self to check future behaviors.  Rough trajectory - lane independent for now
    void project_future_self(double elapsed_time);
    
    
    //---
    // Data, Behavior, Prediction & Trajectory Methods for Finite State Machine
    //---
    
    // 1A. Data
    // Update car w/ localization data returned from Simulator.  Could eliminate extra copy but done for readability.
    void update_Localization_Data(double x, double y, double s, double d, double yaw, double speed, \
                                  double endpath_s, double endpath_d);
    
    
    
    
    
    
    
    // 2. Behavior
    // Per Path Planning approach, update the car's behavior based on telemetry data, car state, and sensor data.
    // This is a predicition because Trajectory update can always
    // Input: Car data & Car State data (previous path & sensor data)
    // Output: target lane (sdc_lane) and velocity (sdc_velocity). Availablity internally & by Get
    void update_Behavior(const vector<double> &previous_path_x, const vector<double> &previous_path_y, \
                         const vector<vector<double>> &sensor_fusion);
   
    
    // 3. Predicition
    
    
    
    // 4. Trajectory
    // Update the car's previous full trajectory path sensors, all the of detected cars in range
    //
    // Input: Car data & Car State data: previous path & sensor data, AND Target Lane , Target Velocity
    // Output: target lane (sdc_lane) and target velocity (sdc_velocity). Availablity internally & by Get
    //
    // Method: Using Spline library for inter-point smooth interpolation
    //
    // Anchor points 30,60,90 (change this to seconds ahead of a 50 mph car in meters
    //
    
    void update_Trajectory(const vector<double> &previous_path_x, const vector<double> &previous_path_y, \
                           const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y, \
                           const vector<double> &map_waypoints_s, vector<double> &x_points, vector<double> &y_points);
    
    //---
    // Getters
    //---
    double get_s();
    
    SelfDrivingCar::State get_State();
    
    SelfDrivingCar::State get_next_State();
    
    int get_lane();
    
    double get_car_speed();
    
    double get_future_s();
    
    double get_future_lane();
    
    double get_future_speed();
    
    double get_future_d();
    
    //---
    // Setters
    //---
    void set_State(SelfDrivingCar::State state);
    
    void set_lane(int updated_lane);
   
    void set_next_State(SelfDrivingCar::State state);
    
    void set_lane_change_in_process(bool lane_change_status);
    
    //---
    // Status
    //---
    bool lane_change_in_process();
    
}; // SelfDrivingCar Class


#endif /* selfdrivingcar_hpp */
