//----------
// This is the Main method for the Path-Planning Projet (main.cpp). This method is the overall framework controller that
// communicates with the Udacity Simulator and Server through a WebSockets messaging protocol. This framework utilizes a
// finite state machine (FSM) and behavior cost functions to decide how to manage self driving car (SDC) path control
// (lane & velocity).
//
// My FSM current States are: KeepLane, LaneChangeLeft, LaneChageRight as well as I added an Emergency State for rapid
// de-accleration and recovery for being cut-off or unforseen behaviors
//
// Structure of main.CPP:
//    Init
//    In-line Functions
//    Helper Methods - General & Geometry
//    Main Method
//        Read Waypoints for Simulator Track
//        WebSocket Message Handler Functions including onMessage (which is key)
//        Start Messaging handling
//
// Main Data Structure Architecture:
//    Self Driving Vehicle Class (selfdrivingcar.cpp) object
//    Tracked Cars object (trackedcar.cpp) & array by lane of ahead and behind (ex. cars_ahead[lane], cars_behind[lane])
//    Matrix of projected costs (projectedcosts[State][lane])
//
// Note1: See (constants.h) for important constants for this project
// Note2: Self Driving Car makes it's own decisions and keeps it's data internal. I have provided a few override
//        set functions that would be used in an emergency situation like getting an SDC off the road, law enforcement, etc..
// Note3: Traffic rules and sdc speed settings to Server are in MPH but tracked vehicles from sensor fusion are in
//        meters/second. I have decided for readibility (maybe incorrectly) to do everything INTERNALLY in MPH.
//        The function mps2mph does the conversions.
//
// Created by Steve Horton on 8/15/17. (Derived from starter code supplied by Udacity SD Nanodegree program)
//----------
#include <fstream>
#include <iostream>
#include <math.h>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <list>

// 3rd party
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include <uWS/uWS.h> // Websockets
#include "spline.h"  // Copyright (C) 2011, 2014 Tino Kluge (ttk448@gmail.com)

// My support libraries
#include "selfdrivingcar.hpp"
#include "trackedcar.hpp"
#include "costfunctions.hpp"
#include "map.hpp"
#include "pathplanningutils.hpp"
#include "constants.hpp"

// For convenience
using json = nlohmann::json;
using namespace std;


// In-line functions for converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
constexpr double deg2rad() { return (0.017453293) ; }
constexpr double mps2mph() { return (2.236936292) ; }
//double meters2miles(double x) { return x * 0.000621371; }
//double miles2meters(double x) { return x * 1609.3440; }
//double mph2mps(double x) { return(x * 0.447040); }  // Exact conversion mph to m/s


//---
// Main Helper Methods
//---

// Checks if the SocketIO event has JSON data. If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
  return "";
}


//---
// Main method for Path Planning & Message Handling with Simulator Server
//---
int main() {
     cout << "Main: Path Planning project code start..." << endl;
    
    //
    // Key project variables & init values for main
    //
    
    string map_file_ = "../data/highway_map.csv";      // Waypoint map to read from
    //string map_file_ = "../data/highway_map_bosch1.csv"; // Waypoint map to read from
   
    long frame_cnt = 0L;                  // Telemetry frame count
    int lane_change_in_progress_cnt = 0;  // Telemetry frame count for lane change progress
    double elapsed_time = 0.0;            // Lane change progress time
    
    int lane = 1;                         // Lane value for trajectory generator (0=Far Left, 1=Center, 2=Far Right)
    double ref_vel = 0.0;                 // Self driving car speed (mph!) (starts in Simulator at 0)
    
    uWS::Hub h;         // Create Websocket message object
    SelfDrivingCar av1; // Create Self Driving Car (sdc) object
    
    
    //---
    // Section to load values for Waypoint's x,y,s and d normalized normal vectors
    //---
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Open & load Waypoint map file for Simulator track
    ifstream in_map_(map_file_.c_str(), ifstream::in);
    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;  // x Normal component of d
        float d_y;  // y Normal component of d
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    } // end Waypoint map load
    cout << "Main: Waypoint file=" << map_file_ << ", No. of Waypoints read=" << map_waypoints_x.size() << endl;

    
    //---
    // Message Handling Section
    //--
    
    // Main receiver of Websocket message event from Simulator Server
    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, \
                 &av1, &frame_cnt, &elapsed_time, &lane_change_in_progress_cnt] \
                 (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
       
    // 4 signifies a websocket message, 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
 
            frame_cnt += 1;
            elapsed_time = frame_cnt*DT;
            cout << "----- count=" << frame_cnt << " elapsed time=" << elapsed_time << " sec -----" << endl;
            
            //
            // Step #0: Get all data returned from Simulator
            // Car's localization data for this frame. j[1] is the data JSON object from Simulation Server
          	//
            double car_x = j[1]["x"];          // car x (meters) in map coord
          	double car_y = j[1]["y"];          // car y (meters) in map coord
          	double car_s = j[1]["s"];          // car s (meters) in Frenet coord (along track)
          	double car_d = j[1]["d"];          // car d (meters) in Frenet coord (displacement from center)
          	double car_yaw = j[1]["yaw"];      // car angle in map coord (degrees!)
          	double car_speed = j[1]["speed"];  // car speed in map coord (in MPH!)

            // Previous path's end s and d values
            double end_path_s = j[1]["end_path_s"];  // car path far end s (meters) in Frenet coord (along track)
            double end_path_d = j[1]["end_path_d"];  // car path far end d (meters) in Frenet coord (displacement from center)
            
          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];  // car's previous frame full trajectory path x (meters) in map coord
          	auto previous_path_y = j[1]["previous_path_y"];  // car's previous frame full trajectory path y (meters) in map coord
          	
          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"]; // Array of detected other cars from car's sensors
            
            //---
            // Step #1A: Update SD Car's (localization) data
            //---
            av1.update_Localization_Data(car_x,car_y,car_s,car_d,car_yaw,car_speed,end_path_s,end_path_d);
          
            //
            // Step #1B. Update SD Car's sensor localization data (i.e. other nearby cars called "sensor fusion")
            // This will become av1.update_TrackedCars after I have more time for refactor
            array<vector<TrackedCar>,NUM_LANES> cars_ahead = {};
            array<vector<TrackedCar>,NUM_LANES> cars_behind = {};
            TrackedCar new_tracked_car = {};
            
            for (int i=0; i<sensor_fusion.size(); i++) {
                
                float d = sensor_fusion[i][6];
                float vehicle_s = (float)sensor_fusion[i][5];
                float delta_s   = vehicle_s - car_s;
                
                // Further Debug
                float vx = sensor_fusion[i][3];
                float vy = sensor_fusion[i][4];
                float vel = sqrt(vx*vx + vy*vy);
                

                // On road
                if (d >= 0.0 && d <= 12.0) {
                    //printf( "%7s %9s %9s %9s %9s %9s \n" , "i","d","vehicle_s","car_s","delta_s","vehicle_vel");

                    // Lane 0
                    if (d >= 0.0 && d < 4.0) {
                        printf("l0: %3d  %8.2f  %8.2f  %8.2f  %8.2f  %8.2f \n",i ,d ,vehicle_s, car_s, delta_s, vel);
                        
                        
                        new_tracked_car.add_sensor_fusion_data(sensor_fusion[i], delta_s);
                    
                        if ((delta_s >= 0.0) && (delta_s <= HORIZON)) {
                            cars_ahead[0].push_back(new_tracked_car);
                        } else if ((delta_s < 0.0) && (delta_s >= -HORIZON)) {
                            cars_behind[0].push_back(new_tracked_car);
                        }
                        
                    // Lane 1
                    } else if (d >= 4.0 && d < 8.0) {
                        
                        //cout << "l1: id,d,sf,cars,delta_s=" << i     << " " << d << " " << (float)sensor_fusion[i][5] << " " \
                        //                                    << car_s << " " << delta_s << endl;
                        printf("l1: %3d  %8.2f  %8.2f  %8.2f  %8.2f  %8.2f \n",i ,d ,vehicle_s, car_s, delta_s, vel);

                        
                        new_tracked_car.add_sensor_fusion_data(sensor_fusion[i], delta_s);
                        
                        if ((delta_s >= 0.0) && (delta_s <= HORIZON)) {
                            cars_ahead[1].push_back(new_tracked_car);
                        } else if ((delta_s < 0.0) && (delta_s >= -HORIZON)) {
                            cars_behind[1].push_back(new_tracked_car);
                        }

                    // Lane 2
                    } else if (d >= 8.0 && d <= 12.0) {

                        //cout << "l2: id,d,sf,cars,delta_s=" << i     << " " << d << " " << (float)sensor_fusion[i][5] << " " \
                        //                                    << car_s << " " << delta_s << endl;
                        printf("l2: %3d  %8.2f  %8.2f  %8.2f  %8.2f  %8.2f \n",i ,d ,vehicle_s, car_s, delta_s, vel);


                        
                        new_tracked_car.add_sensor_fusion_data(sensor_fusion[i], delta_s);
                        //tracked_car.setDelta_S(delta_s);
                        
                        if ((delta_s >= 0.0) && (delta_s <= HORIZON)) {
                            cars_ahead[2].push_back(new_tracked_car);
                        } else if ((delta_s < 0.0) && (delta_s >= -HORIZON)) {
                            cars_behind[2].push_back(new_tracked_car);
                        }
                    }
                }
            } // for
            
            // Store tracked min car ahead & behind for each lane
            array<TrackedCar,NUM_LANES> min_ahead_cars = {};
            array<TrackedCar,NUM_LANES> min_behind_cars = {};
            for (int i=0; i<NUM_LANES; i++) {
                get_min_ahead_cars(cars_ahead[i],min_ahead_cars[i]);
                get_min_behind_cars(cars_behind[i],min_behind_cars[i]);
            }
            
            // Debug
            cout << "Step #1 SDC current lane=" << lane << " speed=" << ref_vel << " mph" << endl;
            cout << "Cars ahead:" << endl;
            for (int i=0; i<NUM_LANES; i++) {
               cout << "L" << i << " # of cars=" << cars_ahead[i].size() << " min s=" << min_ahead_cars[i].get_delta_s() << endl;
            }
            cout << "Cars behind:" << endl;
            for (int i=0; i<NUM_LANES; i++) {
                 cout << "L" << i << " # of cars=" << cars_behind[i].size() << " min s=" << min_behind_cars[i].get_delta_s() << endl;
            }
         
            
            //
            // New - Emergency Safety Check before Step #2
            // <TODO> Need to work on speed back up after clear
            //
            if  (min_ahead_cars[lane].get_delta_s() <= SAFETY_DISTANCE) {
                av1.set_State(SelfDrivingCar::State::Emergency);
                cout << "SAFETY engaged! Lane=" << lane << " min ahead detected=" << min_ahead_cars[lane].get_delta_s() << endl;
            }
            
            
            //-----
            // Step #2. - Behavior
            //
            // This section generates POTENTIAL behaviors w/ associated cost functions as part of an FSM. The result output is
            // next proposed state for predicition module
            // Note: This will become av1.update_Behavior after I have more time for refactor
            //---
            cout << "Step #2 - Behavior estimates:" << endl;
            //av1.update_Behavior(previous_path_x, previous_path_y, sensor_fusion);
            
            vector<SelfDrivingCar::State>  feasible_states = {};
            SelfDrivingCar::State best_next_state = {};
            double cost;
    
            // Cost matrix by: row = state  , col=lane
            double projected_costs[4][3] = {
                                           {__DBL_MAX__,__DBL_MAX__,__DBL_MAX__},
                                           {__DBL_MAX__,__DBL_MAX__,__DBL_MAX__},
                                           {__DBL_MAX__,__DBL_MAX__,__DBL_MAX__},
                                           {__DBL_MAX__,__DBL_MAX__,__DBL_MAX__}, };
            
            double min_cost = __DBL_MAX__;
            vector<int> feasible_lanes = {};
            
            // Piviot on current self driving car State
            switch(av1.get_State()) {
                
                // Current State = Emergency State.  Same new State until ahead_vehicle clears enough
                case (SelfDrivingCar::State::Emergency):
                    
                    if (min_ahead_cars[lane].get_delta_s() < SAFETY_DISTANCE) {
                        
                        // Fall through - Keep slowing down
                        av1.set_next_State(SelfDrivingCar::State::Emergency);
                        cout << "Current State=ES, Next State=ES" << endl;
                    
                    } else {
                        
                        // Can come out of it
                        av1.set_next_State(SelfDrivingCar::State::KeepLane);
                         cout << "Current State=ES, Next State=KL" << endl;
                    }
                break;
                    
                    
                // Current State = KeepLane.
                // This is where FSM calculates cost of 3 options and decides which one to do
                // Calc cost of staying in lane or switching left or right...
                case (SelfDrivingCar::State::KeepLane):
                    
                    // 1. Project sdc out x seconds, project tracked cars out 3 seconds
                    // 2. Calc costs
                    // 3. Choose min cost state
                    
                    // Step #1A. Project av1 forward in time & add to their objects
                    av1.project_future_self(TIME_AHEAD);
                
                    // Step #1B. Project tracked vehicles forward in time & add to their objects
                    // Debug check before
                    //cout << "car ahead lane1: s future=";
                    //for (vector<TrackedCar>::iterator it=cars_ahead[1].begin(); it!=cars_ahead[1].end(); ++it) {
                    //    it->project_future_self(TIME_AHEAD,lane);  // <TODO> SHOULD BE ABLE TO TAKE OUT LANE PART
                    //    cout << it->get_s() << ",";
                    //}
                    //cout << endl;
                    
                    // Project all vehicles, all lanes ahead for up-coming lane movement check
                    for (int lane=0; lane<=NUM_LANES-1; lane++) {
                        
                        // Detected cars ahead
                        for (vector<TrackedCar>::iterator it=cars_ahead[lane].begin(); it!=cars_ahead[lane].end(); ++it) {
                            it->project_future_self(TIME_AHEAD,lane);
                        }
                        
                        // Detected cars behind
                        for (vector<TrackedCar>::iterator it=cars_behind[lane].begin(); it!=cars_behind[lane].end(); ++it) {
                            it->project_future_self(TIME_AHEAD,lane);
                        }
                    } // for
                    
                    // Debug check after projection
                    cout << "tracked cars ahead future=\n";
                    for (int i=0; i<NUM_LANES; i++) {
                        for (vector<TrackedCar>::iterator it=cars_ahead[i].begin(); it!=cars_ahead[i].end(); ++it) {
                            it->project_future_self(TIME_AHEAD,i);
                            //cout << it->get_future_s() << ",";
                            double s = it->get_s();
                            double future_s = it->get_future_s();
                            double delta_s = future_s - s;
                            double vehicle_speed = it->get_speed_mph();
                            printf("%3d  %8.2f  %8.2f  %8.2f  %8.2f \n",i ,s ,future_s, delta_s, vehicle_speed);
                        }
                    }
                    //cout << endl;
                    
                    
                    // Step #2. Calculate forecasted cost
                    feasible_states = av1.get_feasible_next_States(); // For av1 current lane... lane is current lane of av1
                    for (auto next_state : feasible_states) {
                        vector<int> feasible_lanes = {};
                        feasible_lanes = av1.get_feasible_next_Lanes(next_state);
                        for (auto proposed_lane : feasible_lanes) {
                            projected_costs[(int)next_state][(int)proposed_lane] = \
                                    cost_For_Proposed_Trajectory(av1, cars_ahead, cars_behind, proposed_lane);
                        } // for
                    } // for
                    
                    // Debug
                    cout << "Projected Cost Matrix:" << endl;
                    for (int i=0; i<4; i++) {
                        for (int j=0; j<3; j++) {
                            printf("%E ",projected_costs[i][j]);
                            //cout << projected_costs[i][j] <<  " ";

                        }
                        cout << endl;
                    }
                    
                    // Step #3. Then, find min cost state and make it proposed_next_State
                    for (auto next_state : feasible_states) {
                        feasible_lanes = av1.get_feasible_next_Lanes(next_state);
                        for (auto proposed_lane : feasible_lanes) {
                            cost = projected_costs[(int)next_state][(int)proposed_lane];
                            if (cost <= min_cost) {  // <TODO> ISSUE ON BREAKING TIE
                                min_cost = cost;
                                best_next_state = next_state;
                            }
                        }
                    }
                    cout << "OK! current,best next,cost=" << (int)av1.get_State() << " , " << (int)best_next_state << " , " \
                        << min_cost << endl;
                    
                    // <TODO> Formalize this: if risk level costs are too high stay put
                    if (min_cost > 2000.0) best_next_state = SelfDrivingCar::State::KeepLane;
                    
                    // This is output
                    av1.set_next_State(best_next_state);
                break;
                
                    
                // Current State = LaneChangeLeft
                case (SelfDrivingCar::State::LaneChangeLeft):
                    
                    lane_change_in_progress_cnt += 1;
                    if (lane_change_in_progress_cnt <= (int)TIME_AHEAD*FPS) {   // ~2 <TODO> Formalize hold until complete
                        av1.set_next_State(SelfDrivingCar::State::LaneChangeLeft);
                    } else if (lane_change_in_progress_cnt >= (int)TIME_AHEAD*FPS) {
                        lane_change_in_progress_cnt = 0;
                        av1.set_next_State(SelfDrivingCar::State::KeepLane);
                        av1.set_State(SelfDrivingCar::State::KeepLane);
                        av1.set_lane(lane);
                        av1.set_lane_change_in_process(false);
                    }
                    break;
                
                
                // Current State = Lane Change to Right
                case (SelfDrivingCar::State::LaneChangeRight):
                    
                    lane_change_in_progress_cnt += 1;
                    if (lane_change_in_progress_cnt < (int)TIME_AHEAD*FPS) {   // ~2 <TODO> Formalize hold until complete
                        av1.set_next_State(SelfDrivingCar::State::LaneChangeRight);
                    } else if (lane_change_in_progress_cnt >= (int)LANE_CHANGE_TIMER*FPS) {
                        lane_change_in_progress_cnt = 0;
                        av1.set_next_State(SelfDrivingCar::State::KeepLane);
                        av1.set_State(SelfDrivingCar::State::KeepLane);
                        av1.set_lane(lane);
                        av1.set_lane_change_in_process(false);
                    }
                    break;
                    
            }; // switch
            
            
            // Handle starting from zero. <TODO> Formalize this as constant
            double MAX_DELTA_SPEED_MPH;
            if (frame_cnt < 200) {
                av1.set_next_State(SelfDrivingCar::State::KeepLane);
                MAX_DELTA_SPEED_MPH = 0.44450;  // Maximum 10ms2 accleration limit in mph/.02 sec for startup (0.447872584)
            } else {
                MAX_DELTA_SPEED_MPH = 0.4441850;
            }
            
            
            //----------
            // Step #3: Prediction Step
            //
            // Predict/generate (lane & velocity) from "current State" & "next State" from finite state machine (FSM)
            // from last Behavior module.
            //----------
            cout << "Step #3 - Prediction (decide lane & velocity w/ current & next state:" << endl;
            
            TrackedCar min_car_ahead = {};
            SelfDrivingCar::State curr_state,next_state;  // SDC states
            
    
            double min_delta = min_ahead_cars[lane].get_delta_s();   // Needs to be outside case statement
        
            // Get States
            curr_state = av1.get_State();
            if (av1.lane_change_in_process()) {
                next_state = av1.get_State();               // Hold current State while lane change in process
            } else {
                next_state = av1.get_next_State(); // Change this naming to just "next_State"
            }
            cout << "Curr+Next States =" << (int)curr_state << "  " << (int)next_state << endl;
            
            // Piviot by "next" State
            switch(next_state) {
                    
                // Next state = "Emergency".  Rapid decel until ahead_vehicle clears enough
                case (SelfDrivingCar::State::Emergency):
                    
                    if (curr_state == SelfDrivingCar::State::Emergency) {
                        
                        ref_vel -= MAX_ACCEL; // Emergency de-celeration (at project limits)
                        if (DEBUG) cout << "E+E: decel ref_vel=" << ref_vel << endl;
                        
                    } else if (curr_state == SelfDrivingCar::State::KeepLane) {
                          
                        ref_vel += .05*ref_vel; // Come out of it slowly
                        if (DEBUG) cout << "E+KL: come out of it slowly ref_vel=" << ref_vel << endl;
                      
                    } else if (curr_state == SelfDrivingCar::State::LaneChangeRight) {
                          
                        lane -= 1;              // Abort lane change, return at same speed
                        av1.set_lane(lane);
                        if (DEBUG) cout << "E+LCR: aborted Right Lane Change. Back to lane=" << lane << endl;
                          
                    } else if (curr_state == SelfDrivingCar::State::LaneChangeLeft) {
                          
                        lane += 1;             // Abort lane change, return at same speed
                        av1.set_lane(lane);
                        if (DEBUG) cout << "E+LCL: aborted Left Lane Change. Back to lane=" << lane << endl;
                      }
                break;
                    
                    
                // Next state = "KeepLane".  Stay in same lane Main State of driving in current lane
                case (SelfDrivingCar::State::KeepLane):
                    
                    if (curr_state == SelfDrivingCar::State::Emergency) {
                        
                        ref_vel = 0.02;  // 1.0 (mph) Start-up low speed to slowly come out of ES in same lane
                    
                    // All other current States actions the same - max speed if clear, slow down & match if car ahead
                    } else {
                        
                        // Full speed limit
                        if (min_delta > 32.5) {   // meters
                            
                            ref_vel += MAX_DELTA_SPEED_MPH; // (.44,.4425,.44375) worked,  (.444375, .4441875,.445,.45 ) too much  22 mph sq ~ x m/s
                            ref_vel = min(ref_vel, MAX_SPEED);  // Cap just under speed limit so dont violate
                        
                        // Maintain speed of ahead_car
                        } else if ((min_delta > 10.0) && (min_delta <= 32.5)) {
                            get_min_ahead_cars(cars_ahead[lane], min_car_ahead);
                            double delta_speed_mph = min_car_ahead.v*2.236936292 - av1.get_car_speed();
                            if (DEBUG) cout << "KL: ADJUST SPEED to car ahead(mph)=" << min_car_ahead.get_speed_mph() << "," \
                            << av1.get_car_speed() << "," << delta_speed_mph << "," << ref_vel << endl;;
                            ref_vel += .02*delta_speed_mph;  // .0175 .015 .00875 ok close delta speed in 1 second (1.0/(1.0*50))  1=100%, 1.0=1 second
                        
                        // Back-off if too close
                        } else if ((min_delta > 5.0) && (min_delta <= 10.0)) {
                            ref_vel -= .01*ref_vel; //  1mph/1sec(1*50)
                            cout << "BACKOFF: delta mph=" << .005*ref_vel << "  " << min_delta << endl;
                        }
                    }
                break;
                
                
                // Next State = "Lane Change Left"
                case (SelfDrivingCar::State::LaneChangeLeft):
                    
                    if (curr_state == SelfDrivingCar::State::KeepLane) {
                        lane -= 1;  // This triggers lane change start (no speed change)
                        av1.set_State(SelfDrivingCar::State::LaneChangeLeft);
                        av1.set_lane_change_in_process(true);
                        if (DEBUG) cout << "Switched from KL to LCL. New lane=" << lane <<  endl;
                        break;
                    }
                    if (DEBUG) cout << "KLLCL: Dont change 'lane' or 'ref_vel' for LCL until complete..." << endl;
                break;
   
                    
                // Next State = "Lane Change to Right"
                case (SelfDrivingCar::State::LaneChangeRight):
                    
                    if (curr_state == SelfDrivingCar::State::KeepLane) {
                        lane += 1; // This triggers lane change start (no speed change)
                        av1.set_State(SelfDrivingCar::State::LaneChangeRight);
                        av1.set_lane_change_in_process(true);
                        if (DEBUG) cout << "Switched from KL to LCR. New lane=" << lane << endl;
                        break;
                    }
                    if (DEBUG)cout << "KLLCR:Dont change 'lane' or 'ref_vel' for LCR until complete" << endl;
                break;
            
            };  // switch
            
            
            //-----
            // Section #4 - Trajectory Generation for the Simulator
            //-----
            //cout << "Step #4 - Trajectory Generation:" << endl;
            cout << "lane=" << lane << "  ref_vel=" << ref_vel << " path_size=" << previous_path_x.size() << endl;

            // Simulator returns previous car path at each timestep
            //int detected_cars = sensor_fusion.size();
            //cout << " # of cars detected=" << detected_cars << " Previous car path points=" << prev_size << endl;
            //cout << "previous path points=" << prev_size << endl;
            // 1st thing is to check if car in front (this might go in "prediction" or behavior section. Yes predicition section
            // as predicting
            // if going to run into car or need to brake
            //
            // This seems to have both "predicition" and  "behavior" in it
            //
            //cout << "old logic..." << endl;
         
            int prev_path_size = previous_path_x.size();
            if (prev_path_size > 0) {
                car_s = end_path_s; // This means out at the far end of the trajectory
            }
            
          
            //
            // Trajectory generation uses anchor points (from Map Waypoints & Previous Car Path) and splines to generate a high
            // frequency car trajectory that is fed to the Simulator Server that sets BOTH the position and speed of the car.
            // This is done in 2 sections by generating the anchor points & then creating a high frequency spline fit using the
            // anchor points.
            //
            // Note: For numerical stability, the points are translated to vehicle space and fitted around the horizontal axis
            // (instead of the vertical) and then translated back.
            // Using 5 anchor points to send to spline 1 point behind car, car and 3 ahead at 30, 60 & 90 m
            
            //
            // Section 1: Create 5 trajectory anchor points (spread out over full distance of interest)
            //
            
            // Widely spaced anchor points from waypoints & previous path points ultimately for trajectory spline
            vector<double> ptsx = {};
            vector<double> ptsy = {};
            
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad()*car_yaw; // Need to convert to radians from degrees
            
            // 1st 2 anchor points are one point behind car and one point is car (makes transition smooth)
            if (prev_path_size < 2) {
            
                // Path tangent to current angle of the car
                double ref_yaw = deg2rad()*car_yaw; // Need to convert degrees to radians
                double prev_car_x = car_x - cos(ref_yaw);
                double prev_car_y = car_y - sin(ref_yaw);
                
                // P1
                ptsx.push_back(prev_car_x);
                ptsy.push_back(prev_car_y);
                
                // P2
                ptsx.push_back(car_x);
                ptsy.push_back(car_y);
        
            } else {
            
                // Use 2 points that make path tangent to the previous path's end points
                ref_x = previous_path_x[prev_path_size-1];
                ref_y = previous_path_y[prev_path_size-1];
                double ref_x_prev = previous_path_x[prev_path_size-2];
                double ref_y_prev = previous_path_y[prev_path_size-2];
                // Where is translation????
                ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev); //  Being used later
                
                // P1
                ptsx.push_back(ref_x_prev);
                ptsy.push_back(ref_y_prev);
                
                // P2
                ptsx.push_back(ref_x);
                ptsy.push_back(ref_y);
            
            }
            
            //
            // For next 3 (look-ahead) points,  Translate to Frenet Space & create more points  evenly spaced at 30m (anchor points)
            //
            //cout << "car_s=" << car_s << " " << 2+4*lane << endl;
            vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            //cout << "wp0,1,2=" << next_wp0[0] << " " << next_wp1[0] << " " << next_wp2[0] << endl;
            //cout << "wp0,1,2=" << next_wp0[1] << " " << next_wp1[1] << " " << next_wp2[1] << endl;
            
            /*
            vector<double> next_wp0 = getXY(car_s+25, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+50, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+75, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp3 = getXY(car_s+100, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            */
            
            
            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);
            //ptsx.push_back(next_wp3[0]);
            
            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);
            //ptsy.push_back(next_wp3[1]);
            
            // Transfer points into car reference frame (keeps spline well behaved, easier math. Class hint)
            for (int i=0; i<ptsx.size(); i++ ) {
            
                // Translate, then rotate to 0 degrees. Will reverse at end: rotate and translate...
                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;
                
                ptsx[i] = (shift_x*cos(0.0-ref_yaw) - shift_y*sin(0.0-ref_yaw));
                ptsy[i] = (shift_x*sin(0.0-ref_yaw) + shift_y*cos(0.0-ref_yaw));
            }

            
            /* Useful Debug when point gen a problem
            cout << "ptsx,ptsy size=" << ptsx.size() << endl;
            cout << "ptsx before spline=" << ptsx[0] << " " << ptsx[1] << " " << ptsx[2] << " " << ptsx[3] << " " << ptsx[4] << endl;
            cout << "ptsy before spline=" << ptsy[0] << " " << ptsy[1] << " " << ptsy[2] << " " << ptsy[3] << " " << ptsy[4] << endl;
            */
            
            //
            // Section 2: Create full, high frequency trajectory with anchor points (spread out over full distance of interest)
            //
            
            // Create fitted spline coefficients from anchor points
            tk::spline s;
            s.set_points(ptsx,ptsy);
            
            //---
            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            vector<double> next_x_vals = {};
            vector<double> next_y_vals = {};

            for (int i=0; i<previous_path_x.size(); i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }
            
            // Generate points ahead of car along spline (in local/car coordinates) TODO Change x,y to car_, car_y
            double target_x = 75.0;  // 100, 30 horizon meters ahead
            double target_y = s(target_x);
            double target_dist = sqrt((target_x*target_x) + (target_y*target_y));
            
            double x_add_on = 0;  // Start at vehicle x=0
            for (int i=1; i<=50-previous_path_x.size(); i++) {
            
                //double N = (target_dist/(.02*ref_vel/2.24)); // (NOTE: 2.24 is not precise enough!!) convert from mph to
                // m/s This is where point spread is made & velocity!!!
                double N = (target_dist/(.02*ref_vel/2.236936292)); // (NOTE: 2.24 is not precise enough!!) convert from mph to m/s This is where point spread is made & velocity!!!
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
            }
          
            
            // Do through Object structure
            vector<double> updated_x_points = {};
            vector<double> updated_y_points = {};
            //av1.update_Trajectory(previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s, \
            //                      updated_x_points, updated_y_points);
            
            
            //
          	//  Section #5. Compose response message & send to Simulator
            //
            json msgJson;
            msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;
            //msgJson["next_x"] = updated_x_points;
            //msgJson["next_y"] = updated_y_points;
          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

            // <TODO> DO I NEED TO COMMENT THIS BACK IN TO SIMULATE LATENCY???
          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  }); // h.onMessage

    
    // (Udacity) We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    
    // Connect to Server
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Main: Connected!" << std::endl;
    });

    // Disconnect from Server
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Main: Disconnected!" << std::endl;
    });

    
    //---
    // Start of Main message handling section
    //---
    int port = 4567;
    if (h.listen(port)) {
        cout << "Main: Path Planning message handler listening on port=" << port << "..." << endl;
    } else {
        cout << "Main: Path Planning message handler failed listening on port=" << port << " Exiting!" << endl;
    return -1;
    }
    
    h.run();   // run Websocket message handler
    return 0;
} // Main


//
// Old Code
//

/*
double ahead_car_speed;
bool too_close = false; // Assume ok until
for (int i=0; i<sensor_fusion.size(); i++) {
    
    float d = sensor_fusion[i][6];  // displacement (d) of a detected car (lane it's in)
    //cout << "d=" << d << endl;
    
    // If ith car is in my lane
    if (d > (2+4*lane-2) && d < (2+4*lane+2)) {
        
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        ahead_car_speed = sqrt(vx*vx + vy*vy);  // MPH??
        double ahead_car_s = sensor_fusion[i][5];
        
        ahead_car_s += ((double)prev_path_size*.02*ahead_car_speed); // if using previous points can project s value out in time because using previous points
        //cout << "ahead_car_s=" << ahead_car_s << endl;
        // If in front and less than gap, slow down relative to car in front
        if ((ahead_car_s > car_s) && ((ahead_car_s - car_s) < 50)) {
            
            //cout << "Car in my lane! Car#=" << i << " v=" << ahead_car_speed << " at " << ahead_car_s << endl;
            
            //ref_vel = .90*ahead_car_speed;
            //ref_vel = 40.0;
            too_close = true;
            
            if (lane == 1) {
                //        lane = 0; // Move to left lane
                //        cout << "Switch from center to left" << endl;
                
            } else if (lane == 0) {
                //         lane = 1;
                //        cout << "Switch from left to center" << endl;
            } else if (lane == 2) {
                //        lane = 1;
                //        cout << "Switch from right to center" << endl;
            }
            
        } // if
    } // if
    
    
    
} // for
*/

/* THis is better in that it looks from center lane if anyone to left OR right!!!
 // Slow down or
 // change lane
 //ref_vel = 29.5;
 too_close = true;
 if((lane == 0) && (lane_1.size() == 0) )
 {
 lane = 1;
 }
 else if((lane == 1) && (lane_0.size() == 0) )
 {
 lane = 0;
 }
 else if((lane == 1) && (lane_2.size() == 0) )
 {
 lane = 2;
 }
 else if((lane == 2) && (lane_1.size() == 0) )
 {
 lane = 1;
 }
 }
*/



/*
//
// Maybe this is supposed to be behavior section??
//

if (too_close) {
    
    //ref_vel -= .224; // ~ 5m/s**2 ??
    //ref_vel -= .250; // ~ 5m/s**2 ??
    //ref_vel -= .1250; // ~ 5m/s**2 ??
    //ref_vel -= .1750; // ~ 5m/s**2 ??
    
    // Last one
    //  ref_vel -= .01*ahead_car_speed; // ~ 5m/s**2 ??
    
    
} else if (ref_vel < 49.575) {
    
    //ref_vel += .50;
    //ref_vel += .45;
    
    // working piece!
    // ref_vel += .425;
    // ref_vel = min(ref_vel, 49.9);  // It isnt just this!! N points must be rounding cause it isnt going above 49.65...

    
}
*/



