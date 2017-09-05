//--------------------
// TrackedCar is a custom class that encapsulate all of the data for a surrounding vehicle that are tracked by the
// self driving car and passed through telemetry via sensor_fusion. I thought it was a good idea to make this a standalone
// data object instead of making the SelfDrivingCar class even bigger and more unwieldly
//
// The key data returned from sensor fusion for this class as per the documentation:
//    A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map
//    coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position
//    in frenet coordinates.
//
// Structure of trackedvehicle.CPP:
//    Init
//    Constructors
//    Data Handling & Methods
//    Getter/Setters
//
//  Created by Steve Horton on 8/30/17.
//----------
#include <stdio.h>
#include <math.h>
#include <vector>

// My libraries
#include "trackedcar.hpp"
#include "constants.hpp"

using namespace std;

constexpr double mps2mph() { return (2.236936292); }
constexpr double mph2mps() { return (0.447040);    }  // This is exact conversion of mph to m/s

/*
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
 */

// Default Constructor
TrackedCar::TrackedCar() {
        id = 0.0; x  = 0.0; y = 0.0;
        vx = 0.0; vy = 0.0;
        s  = 0.0; d  = 0.0;
        
        v = 0.0; speed_mph = 0.0;
        
        lane = 1;
        delta_s = __DBL_MAX__;  // Important default convention that no delta_s exists yet (cant use zero)
        
        future_s     = 0.0;
        future_d     = 0.0;
        future_v     = 0.0;
        future_speed_mph = 0.0;
        future_lane = 1;
}
    
// Alternate Constructor w/ data elements
TrackedCar::TrackedCar(double id, double x, double y, double vx, double vy, double s, double d) {
        this->id = id;
        this->x  = x;
        this->y  = y;
        this->vx = vx;
        this->vy = vy;
        this->s  = s;
        this->d  = d;
        
        this->v = sqrt(vx*vx + vy*vy); // in m/s!
        this->speed_mph = v*mps2mph();
}
    
// Alternate Constructor #2 w/ Sensor Fusion data
TrackedCar::TrackedCar(vector<double> one_car_sensor_fusion) {
        
        id = one_car_sensor_fusion[0];  // Car unique ID
        x  = one_car_sensor_fusion[1];  // x position in map coord
        y  = one_car_sensor_fusion[2];  // y position in map coord
        vx = one_car_sensor_fusion[3];  // x velocity in m/s
        vy = one_car_sensor_fusion[4];  // y velocity in m/s
        s  = one_car_sensor_fusion[5];  // s position in Frenet (meters)
        d  = one_car_sensor_fusion[6];  // d position in Frenet (meters)
        
        v = sqrt(vx*vx + vy*vy);
        speed_mph = v*mps2mph();
    }
    
// Trivial destructor
TrackedCar::~TrackedCar() = default;
    
    
//---
// Data Handling & Methods
//---
    
// COPY telemetry data into object
void TrackedCar::add_sensor_fusion_data(vector<double> one_car_sensor_fusion, double delta_s) {
        
        this->id = one_car_sensor_fusion[0];  // Car unique ID
        this->x  = one_car_sensor_fusion[1];  // x position in map coord
        this->y  = one_car_sensor_fusion[2];  // y position in map coord
        this->vx = one_car_sensor_fusion[3];  // x velocity in m/s
        this->vy = one_car_sensor_fusion[4];  // y velocity in m/s
        this->s  = one_car_sensor_fusion[5];  // s position in Frenet (meters)
        this->d  = one_car_sensor_fusion[6];  // d position in Frenet (meters)
        
        // Calculate
        this->v = sqrt(vx*vx + vy*vy); // m/s
        this->speed_mph = v*mps2mph();      // mph
        
        // Adding from external
        this->delta_s = delta_s;  // referenced to sdc_s for this frame
    }
    
    
// Project the tracked car into future to compare future behaviors
void TrackedCar::project_future_self(double time, int lane) {
        
        this->future_s = s + v*time;
        this->future_d = lane*LANE_WIDTH + LANE_WIDTH/2;  // middle of lane
        this->future_v = v;  // m/s
        this->future_speed_mph = speed_mph;
        this->future_lane = lane;
        //cout << "tracked car: cur s,future s=" << s << "," << future_s << endl;
        //cout << "tracked car: cur d,future d=" << d << "," << future_d << " speed=" << future_speed_mph << endl;
    }
    
    
// Overloaded "=" copy operator to copy object
bool TrackedCar::operator=(const TrackedCar &car)
    {
        this->id = car.id;
        this->x  = car.x;
        this->y  = car.y;
        this->vx = car.vx;
        this->vy = car.vy;
        this->s =  car.s;
        this->d =  car.d;
        
        this->v = sqrt(car.vx*car.vx + car.vy*car.vy);
        this->speed_mph = car.v*mps2mph();
        
        this->delta_s = car.delta_s;  // referenced to sdc_s for this frame
        
        this->future_s = car.future_s;
        this->future_d = car.future_d;
        this->future_lane = car.future_lane;
        this->future_v = car.future_v;
        this->future_speed_mph = car.future_speed_mph;
        
        return (true);
    }
    
    
//---
// Getter Section
//---
double TrackedCar::get_ID() {
        // assuming s", d' and d" = 0
        return id;
    }
    
double TrackedCar::get_s() {
        // assuming s", d' and d" = 0
        return s;
    }
    
double TrackedCar::get_delta_s(){
        // assuming s", d' and d" = 0
        return delta_s;
    }
    
    
double TrackedCar::get_future_s() {
        // assuming s", d' and d" = 0
        return future_s;
    }
    
double TrackedCar::get_speed_mph() {
        return speed_mph;
    }
    
    
//---
// Setter Section
//---

// Set delta_s if calculated relative sdc's s position
void TrackedCar::set_delta_s(double in_delta_s){
        this->delta_s = in_delta_s;
    }
    
    
//}; // Class TrackedCar


