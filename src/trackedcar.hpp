//--------------------
// TrackedCar is a custom class that encapsulate all of the data for a surrounding vehicle that are tracked by the
// self driving car and passed through telemetry via sensor_fusion. I thought it was a good idea to make this a standalone
// data object instead of making the SelfDrivingCar class even bigger and more unwieldly
//
//  trackedcar.hpp
//  Path_Planning
//
//  Created by Steve Horton on 8/30/17.
//
//

#ifndef trackedcar_hpp
#define trackedcar_hpp

#include <stdio.h>
#include <math.h>
#include <vector>

using namespace std;

class TrackedCar {
    
    // Clean everything else then go private and experiment
    
    //private:
public:
    
    // Canonical
    double id;   // Car unique ID
    double x;    // x position in map coord
    double y;    // y position in map coord
    double vx;   // x velocity in m/s
    double vy;   // y velocity in m/s
    double s;    // s position in Frenet (meters)
    double d;    // d position in Frenet (meters)
    
    // Calculated
    double v;          // speed in m/s
    double speed_mph;  // speed in mph
    
    // External
    int lane;
    double delta_s;
    
    // Future self for FSM Behavioral analysis
    double future_s;
    double future_d;
    double future_v;
    double future_speed_mph;
    int future_lane;
    
    //public:
    
    // Default Constructor
    TrackedCar();
    
    // Alternate Constructor w/ data elements
    TrackedCar(double id, double x, double y, double vx, double vy, double s, double d);
    
    // Alternate Constructor #2 w/ Sensor Fusion data
    TrackedCar(vector<double> one_car_sensor_fusion);
    
    // Trivial destructor
    ~TrackedCar();
    
    
    //---
    // Data Handling & Methods
    //---
    
    // COPY telemetry data into object
    void add_sensor_fusion_data(vector<double> one_car_sensor_fusion, double delta_s);
    
    // Project the tracked car into future to compare future behaviors
    void project_future_self(double elapsed_time, int next_lane);
    
    // Overloaded "=" copy operator to copy object
    bool operator=(const TrackedCar &car);
    
    
    //---
    // Getter Section
    //---
    double get_ID();
    
    double get_s();
    
    double get_delta_s();
    
    double get_future_s();
    
    double get_speed_mph();
    
    //---
    // Getter Section
    //---
    // Set delta_s if calculated relative sdc's s position
    void set_delta_s(double in_delta_s);
    
}; // Class TrackedCar


#endif /* trackedcar_hpp */
