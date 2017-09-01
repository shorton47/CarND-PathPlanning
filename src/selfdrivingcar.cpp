//------------------
// Self Driving Car (SDC) class (selfdrivingcar.cpp)
//   This is the main data class for the Path_Planner program. This class enstantiates a self driving car object and
//   encapsulates the data incoming from telemetry, it performs behavior and prediction functions based on a
//   Finite State Machine (FSM) implementation and other various calculations.  SDC makes it's own decisions using FSM & cost
//   functions and keeps it's State data internal. I have provided a few override set functions that would be used in an
//   emergency situation like getting an SDC off the road, law enforcement,etc.
//
// Features: the FSM has 4 states currently: KeepLane, LaneChangeLeft, LaneChageRight as well as I added "Emergency" State for
//   rapid de-acceleration and recovery for being cut-off or unforseen behaviors
//
// Structure of selfdrivingcar.CPP:
//    1. Init
//    2. Constructors
//    3. Helper Methods
//    4. Telemetry Data, Behavior & Predicition methods (4 total)
//    5. Getter/Setters
//
// Key Data Struture:
//    Array of lanes that contains vector of tracked vehicles for that lane per frame
//
// Note1: Traffic rules and sdc speed settings to Server are in MPH but tracked vehicles from sensor fusion are in
//        meters/second. I have decided for readibility (maybe incorrectly) to do everything INTERNALLY in MPH.
//        The function mps2mph does the conversion.
//
// Created by Steve Horton on 8/30/17.
//----------
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include "spline.h"  // Copyright (C) 2011, 2014 Tino Kluge (ttk448@gmail.com)

// My libraries
#include "selfdrivingcar.hpp"
#include "map.hpp"
#include "constants.hpp"

using namespace std;

constexpr double pi() { return M_PI; }
constexpr double deg2rad() { return (0.017453293); }
constexpr double rad2deg() { return (57.95777937); }
constexpr double mps2mph() { return (2.236936292); }
constexpr double mph2mps() { return (0.447040);    }  // This is exact conversion of mph to m/s

/*
//constexpr double meters2miles(double x) { return x * 0.000621371; }
//constexpr double miles2meters(double x) { return x * 1609.3440; }
*/



/*
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
 
 */
 
// Default Constructor
SelfDrivingCar::SelfDrivingCar() {
        
    sdc_x = 0.0; sdc_y = 0.0; sdc_s = 0.0; sdc_d = 0.0;
    sdc_yaw = 0.0; sdc_speed = 0.0;
    sdc_endpath_s = 0.0; sdc_endpath_d = 0.0;
        
    sdc_state      = SelfDrivingCar::State::KeepLane;
    sdc_next_state = SelfDrivingCar::State::KeepLane;
        
    // Match Simulator initializes sdc in 2nd from the left lane at zero speed
    sdc_lane    = 1;    // 2nd from left lane. Lane convention=[0,1,2]
    sdc_ref_vel = 0.0;  // 1st velocity point (MPH)
        
    sdc_future_s = 0.0; sdc_future_speed = 0.0;
    sdc_future_d =0.0; sdc_future_lane = 1;
        
    sdc_lane_change_in_process = false;
        
    cout << "SDC : init w/ lane=" << sdc_lane << ", speed=" << sdc_speed << ", max speed="  << MAX_SPEED \
         << " current State=" << State_Name[(int)sdc_state] << endl;
}
    
// Destructor
SelfDrivingCar::~SelfDrivingCar() {}
    
    
//---
// Helper Methods
//---
    
// Feasible future States to switch to for a given lane from defined Finite State Machine (FSM)
vector<SelfDrivingCar::State> SelfDrivingCar::get_feasible_next_States() {
        
        vector<SelfDrivingCar::State> valid_states;
        
        
        valid_states = {};
        // Left lane
        if (sdc_lane == 0) {
            valid_states = {SelfDrivingCar::State::KeepLane, SelfDrivingCar::State::LaneChangeRight};
            
            // Middle lane
        } else if (sdc_lane == 1) {
            valid_states = {SelfDrivingCar::State::LaneChangeLeft, SelfDrivingCar::State::KeepLane, SelfDrivingCar::State::LaneChangeRight,};
            
            // Right lane
        } else if (sdc_lane == 2) {
            valid_states = {SelfDrivingCar::State::LaneChangeLeft, SelfDrivingCar::State::KeepLane};
            
        } else {
            cout << "SDC: Error, invalid lane=" << sdc_lane << endl;
        }
        
        return valid_states;
};
    
    
// Feasible future lanes to switch to for a given SDC State from defined Finite State Machine (FSM)
vector<int> SelfDrivingCar::get_feasible_next_Lanes(SelfDrivingCar::State &next_state) {
        
        vector<int> valid_Lanes;
        
        
        valid_Lanes = {};
        if (next_state == SelfDrivingCar::State::KeepLane) {
            
            valid_Lanes = {sdc_lane};
            
        } else if (next_state == SelfDrivingCar::State::LaneChangeLeft) {
            
            valid_Lanes = {sdc_lane-1};
            
        } else if (next_state == SelfDrivingCar::State::LaneChangeRight) {
            
            valid_Lanes = {sdc_lane+1};
        }
        
        return valid_Lanes;
};
    
    
// Project self driving car into a future self to check future behaviors.  Rough trajectory - lane independent for now
void SelfDrivingCar::project_future_self(double elapsed_time) {
    
    cout << "av1 pfs: time,speed" << elapsed_time << " " << sdc_speed << endl;
        sdc_future_s = sdc_s +  (sdc_speed*elapsed_time)*.447038889;  // Convert speed in mph to delta s in meters
        //sdc_future_s = sdc_s +  (sdc_speed*elapsed_time);  // Convert speed in mph to delta s in meters
        sdc_future_d = sdc_lane*LANE_WIDTH + 2;
        sdc_future_speed = sdc_speed;
};


//---
// Data, Behavior, Prediction & Trajectory Methods for Finite State Machine
//---
    
// 1A. Data - Update car w/ localization data returned from Simulator.  Could eliminate extra copy but done for readability.
void SelfDrivingCar::update_Localization_Data(double x, double y, double s, double d, double yaw, double speed, \
                                              double endpath_s, double endpath_d) {
        
        sdc_x = x;                  // car x (meters) in map
        sdc_y = y;                  // car y (meters) in map
        sdc_s = s;                  // car s (meters) in Frenet coord (along track)
        sdc_d = d;                  // car d (meters) in Frenet coord (displacement from center lane)
        sdc_yaw = yaw;             // car angle (degrees) in map. Simulation begins with car at 0 degrees pointing straight ahead
        sdc_speed=speed;            // car speed (MPH)
        sdc_endpath_s = endpath_s;  // last s point from prior path from Telemetry
        sdc_endpath_d = endpath_d;  // last d point from prior path from Telemetry
        
};
    
    
    
    
    
    
    
    // 2. Behavior
    // Per Path Planning approach, update the car's behavior based on telemetry data, car state, and sensor data.
    // This is a predicition because Trajectory update can always
    // Input: Car data & Car State data (previous path & sensor data)
    // Output: target lane (sdc_lane) and velocity (sdc_velocity). Availablity internally & by Get
void SelfDrivingCar::update_Behavior(const vector<double> &previous_path_x, const vector<double> &previous_path_y, \
                         const vector<vector<double>> &sensor_fusion) {
        
        
        //   A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in
        //  map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d
        //  position in frenet coordinates.
        //
        
        //
        // The key final loop to this is a setting of car velocity and lane selection based on The FSM selection of the best state
        // with minimum cost
        
    };
    
    
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
    
void SelfDrivingCar::update_Trajectory(const vector<double> &previous_path_x, const vector<double> &previous_path_y, \
                           const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y, \
                           const vector<double> &map_waypoints_s, vector<double> &x_points, vector<double> &y_points) {
        
        
        // The key input from Behavior module is the lane selection & velocity. These 2 things determine how the points
        // are generated
        
        
        //
        // I think this starts the trajectory generation section
        //
        // Using 5 anchor points to send to spline 1 point behind car, car and 3 ahead at 30, 60 & 90 m
        
        
        // Widely spaced waypoints, evenly spaced at 30m (anchor points)
        vector<double> ptsx = {};  // Anchor points for trajectory spline
        vector<double> ptsy = {};
        
        double ref_x = sdc_x;
        double ref_y = sdc_y; // car y (meters) in map
        double ref_yaw = deg2rad()*sdc_yaw; // car yaw (degrees)?? units but used later
        
        //cout << ref_x << " " << ref_y << " " << car_yaw << " " << ref_yaw << endl;
        double car_yaw_r = deg2rad()*sdc_yaw;
        
        
        //
        // 1st 2 anchor points - 1 point behind car, 1 point is car
        //
        int prev_size = previous_path_x.size();
        if (prev_size < 2) {
            
            // Path tangent to current angle of the car
            //double prev_car_x = sdc_x - cos(sdc_yaw);  // Need to convert to radians?
            //double prev_car_y = sdc_y - sin(sdc_yaw);
            
            double prev_car_x = sdc_x - cos(car_yaw_r);  // Need to convert to radians?
            double prev_car_y = sdc_y - sin(car_yaw_r);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(sdc_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(sdc_y);
            
            
        } else {
            
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
            
            // Use 2 points that make path tangent to the previous path's end points
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
            
        }
        
        //
        // Create 3 more ahead Anchor Points to Frenet Space & create more points
        vector<double> next_wp0 = getXY(sdc_s+30, (2+4*sdc_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> next_wp1 = getXY(sdc_s+60, (2+4*sdc_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> next_wp2 = getXY(sdc_s+90, (2+4*sdc_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        
        ptsx.push_back(next_wp0[0]);
        ptsx.push_back(next_wp1[0]);
        ptsx.push_back(next_wp2[0]);
        
        ptsy.push_back(next_wp0[1]);
        ptsy.push_back(next_wp1[1]);
        ptsy.push_back(next_wp2[1]);
        
        
        
        // Transfer points into car reference frame (keeps spline well behaved, easier math. Class hint)
        for (int i=0; i<ptsx.size(); i++ ) {
            
            // Translate, then rotate to 0 degrees. Will reverse at end: rotate and translate...
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
        }
        
        tk::spline s;
        
        //cout << "ptsx size=" << ptsx.size() << endl;
        //cout << "ptsx before spline=" << ptsx[0] << endl;
        //cout << "ptsy before spline=" << ptsy[0] << endl;
        
        
        s.set_points(ptsx,ptsy);  // Create spline from anchor points
        
        
        
        
        //---
        // Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
        // With Anchor points & car's previous path, build a
        vector<double> next_x_vals = {};
        vector<double> next_y_vals = {};
        
        for (int i=0; i<previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
        }
        
        // Generate points ahead of car along spline (in local/car coordinates) (xpoint, ypoint)
        double target_x = 30.0;  // horizon meters ahead
        double target_y = s(target_x);
        double target_dist = sqrt((target_x*target_x) + (target_y*target_y));
        
        double x_add_on = 0;  // Start at vehicle x=0
        for (int i=1; i<=50-previous_path_x.size(); i++) {
            
            double N = (target_dist/(.02*sdc_ref_vel/2.24)); // convert from mph to m/s
            double x_point = x_add_on + (target_x/N);
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Rotate back to "normal" e earlier rotation
            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);
            
            x_point += ref_x;
            y_point += ref_y;
            
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
        } // for
        
    }; // update_Trajectory
    
    
    //---
    // Getters
    //---
double SelfDrivingCar::get_s() {
        return sdc_s;
    }
    
    SelfDrivingCar::State SelfDrivingCar::get_State(){
        return sdc_state;
    }
    
    SelfDrivingCar::State SelfDrivingCar::get_next_State(){
        return sdc_next_state;
    }
    
    int SelfDrivingCar::get_lane(){
        return sdc_lane;
    }
    
    double SelfDrivingCar::get_car_speed() {
        return sdc_speed;
    }
    
    double SelfDrivingCar::get_future_s() {
        return sdc_future_s;
    }
    
    double SelfDrivingCar::get_future_lane() {
        return sdc_future_lane;
    }
    
    double SelfDrivingCar::get_future_speed() {
        return sdc_future_speed;
    }
    
    double SelfDrivingCar::get_future_d() {
        return sdc_future_d;
    }
    
    
    //---
    // Setters
    //---
    void SelfDrivingCar::set_State(SelfDrivingCar::State state) {
        sdc_state = state;
        return;
    }
    
    void SelfDrivingCar::set_lane(int updated_lane) {
        sdc_lane = updated_lane;
        return;
    }
    
    void SelfDrivingCar::set_next_State(SelfDrivingCar::State state) {
        sdc_next_state = state;
        return;
    }
    
    void SelfDrivingCar::set_lane_change_in_process(bool lane_change_status) {
        sdc_lane_change_in_process = lane_change_status;
        return;
    }
    
    
    //---
    // Status
    //---
    bool SelfDrivingCar::lane_change_in_process() {
        return sdc_lane_change_in_process;
    }
    
    
//}; // SelfDrivingCar Class
