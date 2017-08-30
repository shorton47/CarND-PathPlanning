//----------
//
//
// Structure of Main.CPP:
//    Init
//    In-line Functions
//    Helper Methods - General & Geometry
//    Main Method
//        Read Waypoints for Simulator Track
//        WebSocket Message Handler Functions including onMessage (which is key)
//        Start Messaging handling
//
// Conventions:
//   Lanes from center median
//
//
// Note: Normally would split this up into several files for easier management. However for both the project submission and the
// BOSCH challenge, request is for main.cpp only so all is in this file for now.
//
//
// Self Driving Car makes it's own decisions and keeps it's data internal. I have provided a few override set functions that
// would be used in an emergency situation like getting an SDC off the road, law enforcement, etc..
//
// Architecture:
//    Self Driving Vehicle Class & Object
//    Tracked Cars Object
//    FSM to calculate costs of lane choice and car speed
//
// Key Data Struture:
//    Array of lanes that contains vector of tracked vehicles for that lane per frame
//
//
// Note1: Traffic rules and sdc speed settings to Server are in MPH but tracked vehicles from sensor fusion are in
//        meters/second. I have decided for readibility (maybe incorrectly) to do everything INTERNALLY in MPH.
//        The function mps2mph does the conversions.
//
//----------
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"  // Copyright (C) 2011, 2014 Tino Kluge (ttk448@gmail.com)
#include <algorithm> // Needed for sort
#include <list>

// My support libraries
//#include "utility"
#include "utils.hpp"  // My utility library

// For convenience
using json = nlohmann::json;
//using State = SelfDrivingCar::State;
using namespace std;

//
// Important Project LevelConstants: Simulator, Track, etc...
//
#define FULL_TRACK_S 6945.554  // One loop of Simulator track (meters)
#define DT           0.02      // Delta time of Simulator telemetry. NOTE: Fixed for this project (real-world it usually isnt)
#define FPS          50        // Associated frames/second of telemtry from Simulator
#define NUM_LANES  3           // Number of highway lanes. Fixed in this Simulator
#define LANE_WIDTH 4.0         // Width of each lane (meters)
//#define LARGE_NUM_DBL 1.0e47   // Machine independent large number used for min/max and testing


#define HORIZON    150.0       // 200.0 300.0 (meters)  Horizon over which to track vehicles and sensor data
#define TIME_AHEAD   2.0       // 1.0 1.25 1.5 1.0 2.0 (seconds)  Projection ahead for rough trajectory (less time means safer distance)

#define MAX_SPEED 49.80         // 49.88 was too high for speed variations

//---
// Main Helper Methods
//---

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
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
// General Utility Methods Section
//---

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}



//---
// General Map Utility Methods Section
//---
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

///--- ANOTHER CLASS FOR TRACKING CARS
class Tracked_Vehicle {

public:
    
//private:
    // Public Data
    double id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
    
    // Calculated
    double v;
    double speed_mph;
    
    // Added from external
    double delta_s = {};
    int lane = {};
    
    // Future self for predicted behavior analysis
    int future_lane = {};
    double future_s = 0.0;
    double future_d = {};
    double future_v = 0.0;
    
    
    
    
    // Project self driving car into a future self to check future behaviors
    void project_future_self(double elapsed_time, int next_lane) {
        
        
        //double current_speed = sqrt(vx*vx + vy*vy);
        
        this->future_lane = next_lane;
        //this->future_s    = s + (current_speed*elapsed_time)*.447038889;
        this->future_s    = s + v*elapsed_time;
        this->future_d    = future_lane*LANE_WIDTH + 2;
        this->future_v = v;  // m/s
        //cout << "tracked car: cur s,future s=" << s << "," << future_s << endl;
        //cout << "tracked car: cur d,future d=" << d << "," << future_d << endl;
    }

    
    
//public:
    
    
    Tracked_Vehicle() {
        id = 0.0;
        x = 0.0;
        y = 0.0;
        vx = 0.0;
        vy = 0.0;
        s = 0.0;
        d = 0.0;
        
        v = 0.0;
        speed_mph = 0.0;
        
        delta_s = __DBL_MAX__;
    }
    
    
    
    /* A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d pos
     */
    Tracked_Vehicle(vector<double> one_car_sensor_fusion) {
        
        id = one_car_sensor_fusion[0];  // Car unique ID
        x  = one_car_sensor_fusion[1];  // x position in map coord
        y  = one_car_sensor_fusion[2];  // y position in map coord
        vx = one_car_sensor_fusion[3];  // x velocity in m/s
        vy = one_car_sensor_fusion[4];  // y velocity in m/s
        s  = one_car_sensor_fusion[5];  // s position in Frenet
        d  = one_car_sensor_fusion[6];  // d position in Frenet
        
        v = std::sqrt(vx*vx + vy*vy);
        speed_mph = v*2.236936292;
    }

    
    
    
    
    
    
    Tracked_Vehicle(double id, double x, double y, double vx, double vy, double s, double d){
        this->id = id;
        this->x = x;
        this->y = y;
        this->vx = vx;
        this->vy = vy;
        this->s = s;
        this->d = d;
        this->v = std::sqrt(vx*vx + vy*vy); // m/s!  CHANGE THIS TO SPEED IN MPH?? LIKE SDC??
        this->speed_mph = v*2.236936292;
    }
    
    // Trivial destructor
    ~Tracked_Vehicle() = default;
    
    
    // Update car w/ localization data returned from Simulator.
    // Note: this is extra copy. Could be eliminated for speed but done for readability.
    void setData(double id, double x, double y, double vx, double vy, double s, double d) {
        
        this->id = id;
        this->x = x;
        this->y = y;
        this->vx = vx;
        this->vy = vy;
        this->s = s;
        this->d = d;
        
        this->v = std::sqrt(vx*vx + vy*vy);
        this->speed_mph = v*2.236936292;
    };
    
    
    void add_Sensor_Fusion_Data(vector<double> one_car_sensor_fusion) {
        
        id = one_car_sensor_fusion[0];  // Car unique ID
        x  = one_car_sensor_fusion[1];  // x position in map coord
        y  = one_car_sensor_fusion[2];  // y position in map coord
        vx = one_car_sensor_fusion[3];  // x velocity in m/s
        vy = one_car_sensor_fusion[4];  // y velocity in m/s
        s  = one_car_sensor_fusion[5];  // s position in Frenet (meters)
        d  = one_car_sensor_fusion[6];  // d position in Frenet (meters)
        
        // Calculated
        v = std::sqrt(vx*vx + vy*vy); // m/s
        speed_mph = v*2.236936292;
        
    }

    
    
    
    
    
    // Update car w/ localization data returned from Simulator.
    // Note: this is extra copy. Could be eliminated for speed but done for readability.
    void getData(Tracked_Vehicle tracked_vehicle) {
        
        id = tracked_vehicle.id;
        x  = tracked_vehicle.x;
        y  = tracked_vehicle.y;
        vx = tracked_vehicle.vx;
        vy = tracked_vehicle.vy;
        s  = tracked_vehicle.s;
        d  = tracked_vehicle.d;
        
        v = std::sqrt(vx*vx + vy*vy);
        speed_mph = v*2.236936292;
        
    };

    
 
    
    
    
    //bool operator < (Tracked_Vehicle &rhs) {return s < rhs.s;}  // Overloaded < operator for object comparison

    //bool SortByAscendingS(Tracked_Vehicle const &lhs, Tracked_Vehicle const &rhs) {return lhs.s < rhs.s;}
    
    
    bool operator<(const Tracked_Vehicle &b)
    {
        return (s < b.s);
    }
    

    // This one is being used!!!!!!!
    bool operator=(const Tracked_Vehicle &a)
    {
        this->id = a.id;
        this->x = a.x;
        this->y = a.y;
        this->vx = a.vx;
        this->vy = a.vy;
        this->s = a.s;
        this->d = a.d;
        
        this->v = std::sqrt(a.vx*a.vx + a.vy*a.vy);
        this->speed_mph = v*2.236936292;
        //cout << "overload copy called! s=" << a.s << " " << s << endl;
        
        return (true);
    }

    // Overload to copy Tracked_Vehicle object
/*
public: Tracked_Vehicle& operator=(const Tracked_Vehicle& rhs) {};
*/
    
    
    
    
    double predict(double t){
        // assuming s", d' and d" = 0
        return s + v*t;
    }
    
    // Getters
    double getID(){
        // assuming s", d' and d" = 0
        return id;
    }
    
    double getS(){
        // assuming s", d' and d" = 0
        return s;
    }
    
    double getDelta_S(){
        // assuming s", d' and d" = 0
        return delta_s;
    }
    
    
    double get_future_s() {
        // assuming s", d' and d" = 0
        return future_s;
    }
    
    double get_speed_mph() {
        return speed_mph;
    }
    
   
    
    // Set delta_S if calculated relative to a sdc s position
    void setDelta_S(double in_delta_s){
        this->delta_s = in_delta_s;
    }
    
    
}; // Class Tracked_Vehicle

//
// Helper/Utility Functions
//

/*
struct SortByAscendingS
{
    bool operator() (const Tracked_Vehicle &lhs, const Tracked_Vehicle &rhs)
    {
        return (lhs.s < rhs.s);
    }
};
*/

// Note: need to pass the Object to get it's order changed
struct SortByAscendingS
{
    bool operator() (const Tracked_Vehicle &lhs, const Tracked_Vehicle &rhs)
    {
        return (lhs.s < rhs.s);
    }
};



struct SortByAscendingDeltaS
{
    bool operator() (const Tracked_Vehicle &lhs, const Tracked_Vehicle &rhs)
    {
        cout << "lhs,rhs=" << lhs.delta_s << " " << rhs.delta_s << endl;
        return (lhs.delta_s < rhs.delta_s);
    }
};






/*
struct SortByAscendingDeltaS
{
    bool operator() (const Tracked_Vehicle &lhs, const Tracked_Vehicle &rhs)
    {
        cout << "lhs,rhs=" << lhs.delta_s << " " << rhs.delta_s << endl;
        return (lhs.delta_s < rhs.delta_s);
    }
};
*/
 
struct SortByDescendingDeltaS
{
    bool operator() (const Tracked_Vehicle &lhs, const Tracked_Vehicle &rhs)
    {
        return (lhs.delta_s > rhs.delta_s);
    }
};

double min_Delta_S(vector<Tracked_Vehicle> &tracked_cars) {

    //double min_delta_s = LARGE_NUM_DBL; // Start w/ large #
    double min_delta_s = __DBL_MAX__;  // Start w/ large #

    if (tracked_cars.size() > 0) {
        for (vector<Tracked_Vehicle>::iterator it = tracked_cars.begin(); it!=tracked_cars.end(); ++it) {
            min_delta_s = min(min_delta_s, it->delta_s);  // Compare delta_s in Tracked_Vehicle object
        }
    }
    
    return min_delta_s;
}

double max_Delta_S(vector<Tracked_Vehicle> &tracked_cars) {
    
    //double max_delta_s = -LARGE_NUM_DBL; // Start w/ large negative #
    double max_delta_s = -__DBL_MAX__; // Start w/ large negative #
    
    if (tracked_cars.size() > 0) {
        for (vector<Tracked_Vehicle>::iterator it = tracked_cars.begin(); it!=tracked_cars.end(); ++it) {
            //cout << "-" << max_delta_s << "," << it->delta_s << endl;
            max_delta_s = max(max_delta_s, it->delta_s);  // Compare delta_s in Tracked_Vehicle object
        }
    }
    
    return max_delta_s;
}

// !!!!!
// What happend if the vector of tracked cars is empty?
// min_delta_s_tracked cars should come in empty and if there are non, should go out empty
// tracked_cars has to be initialized to
void get_min_ahead_cars(vector<Tracked_Vehicle> &tracked_cars, Tracked_Vehicle &min_delta_s_tracked_car) {
    
    double min_delta_s = __DBL_MAX__;         // Start w/ large #
    vector<Tracked_Vehicle>::iterator it_save;  // Save pointer to min object (de-activate)
    
    if (tracked_cars.empty()) {
        min_delta_s_tracked_car.setDelta_S(__DBL_MAX__); // This is flag that no delta_s (maybr skip_
        //cout << "tracked cars AHEAD for this lane is empty!" << endl;
        return;
    }
    
    
    for (vector<Tracked_Vehicle>::iterator it = tracked_cars.begin(); it!=tracked_cars.end(); ++it) {
        
        if (it->delta_s <= min_delta_s) {
            min_delta_s = it->delta_s;  // Compare delta_s in Tracked_Vehicle object to find min
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
        }
        //min_delta_s_tracked_car = (*it);
    } // for
    
    //cout << "min AHEAD  delta_s=" << min_delta_s << "," << min_delta_s_tracked_car.delta_s << endl;
}


void get_min_behind_cars(vector<Tracked_Vehicle> &tracked_cars, Tracked_Vehicle &min_delta_s_tracked_car) {
    
    double min_delta_s = __DBL_MAX__;         // Start w/ large #
    vector<Tracked_Vehicle>::iterator it_save;  // Save pointer to min object
    
    
    if (tracked_cars.empty()) {
        min_delta_s_tracked_car = {};
       // cout << "tracked cars BEHIND for this lane is empty!" << endl;
        return;
    }
    
    
    
    for (vector<Tracked_Vehicle>::iterator it = tracked_cars.begin(); it!=tracked_cars.end(); ++it) {
        
        if (abs(it->delta_s) <= min_delta_s) {
            min_delta_s = it->delta_s;  // Compare delta_s in Tracked_Vehicle object
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
        }
        
        //min_delta_s = min(min_delta_s, it->delta_s);  // Compare delta_s in Tracked_Vehicle object
        //it_save = it;
        //cout << "it,it_save=" << &it << "," << &it_save << endl;
        //cout << "min behind delta_s=" << it->delta_s << "," << min_delta_s_tracked_car.delta_s << endl;
        
    } // for
    
    //cout << "min BEHIND delta_s=" << min_delta_s << "," << min_delta_s_tracked_car.delta_s << endl;
    
    //min_delta_s_tracked_car = *it_save;
}


//
// More Helper Functions
//



///------------------
// Self Driving Car Class
//


class SelfDrivingCar{
    
//
// Important Constants for Self Driving Car
//
//#define HIWAY_SPEED_LIMIT_MPH 49.45  // = 79.58 km/h or 22.xx m/s
    
    
public:
    //enum class State {EmergencyStop, KeepLane, LaneChangeLeft, LaneChangeRight, LaneChangeInProcess};
    //enum class State {EmergencyStop, KeepLane, LaneChangeLeft, LaneChangeRight};
    enum class State {EmergencyStop, LaneChangeLeft, KeepLane, LaneChangeRight};

    
    
    
    
private:
    
    double sdc_x; double sdc_y;
    double sdc_s; double sdc_d;
    double sdc_yaw; double sdc_speed;
    double sdc_endpath_s; double sdc_endpath_d;
    
    int sdc_lane;
    SelfDrivingCar::State sdc_state, sdc_proposed_next_state;
    
    // Future self for behavior analysis
    double sdc_future_s;
    double sdc_future_speed;
    int sdc_future_lane;
    double sdc_future_d;
    
    bool sdc_lane_change_in_process;
    
    
    
    double SAFETY_DISTANCE; //meters
    double target_vel;  // Target velocity used to set sdc speed through trajectory point spacing
    //double SPEED_LIMIT;// 49.5mph = 22.098m/s
    
    //string sdc_state; // state includes KL - Keep Lane / LCL - Lane Change Left / LCR - Lane Change Right
    vector<double> lane_speed;
    vector<double> lane_frontcar_s;
    vector<double> lane_backcar_s;
    
public:
    
    
   
    
    
    
    // Default Constructor
    SelfDrivingCar(){
        
        // Simulator initializes in middle lane at zero speed
        sdc_lane = 1; // middle lane
        sdc_state = SelfDrivingCar::State::KeepLane;
        sdc_proposed_next_state = SelfDrivingCar::State::KeepLane;
        target_vel = 0.0; // 1st velocity point (x - 0)/.02
        
        SAFETY_DISTANCE = 25.0;
       
        //SPEED_LIMIT_MPH = 49.5;
        //lane_speed = {HIWAY_SPEED_LIMIT_MPH, HIWAY_SPEED_LIMIT_MPH, HIWAY_SPEED_LIMIT_MPH};
        lane_frontcar_s = {numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max()};
        lane_backcar_s = {numeric_limits<double>::min(), numeric_limits<double>::min(), numeric_limits<double>::min()};
        //sdc_state = "KL";
        
        sdc_x = 0.0; sdc_y = 0.0; sdc_s = 0.0; sdc_d = 0.0;
        sdc_yaw = 0.0; sdc_speed = 0.0;
        sdc_endpath_s = 0.0; sdc_endpath_d = 0.0;
        
        sdc_lane_change_in_process = false;
        sdc_future_speed = 0.0;
        
        cout << "SDC: Self Driving Car initialized w/ lane=" << sdc_lane << ", speed=" << target_vel << ", max speed=" << \
                MAX_SPEED << endl;
    }
    
    // Destructor
    virtual ~SelfDrivingCar() {}
    
    
    // Update car w/ localization data returned from Simulator.
    // Note: this is extra copy. Could be eliminated for speed but done for readability.
    void update_LocalizationData(double x, double y, double s, double d, double yaw, double speed, \
                                 double endpath_s, double endpath_d) {
       
        sdc_x = x;       // car x (meters) in map
        sdc_y = y;       // car y (meters) in map
        sdc_s = s;       // car s (meters) in Frenet coord (along track)
        sdc_d = d;       // car d (meters) in Frenet coord (displacement from center lane)
        sdc_yaw = yaw;   // car angle (degrees) in map. Note: Simulation begins with car at 0 degrees pointing straight ahead
        sdc_speed=speed; // car speed (MPH)
        sdc_endpath_s = endpath_s;
        sdc_endpath_d = endpath_d;
    
    };
    
    
    // Project self driving car into a future self to check future behaviors
    // For now, rought trajectory - lane independent
    void project_future_self(double elapsed_time) {
    
        //sdc_future_lane = future_lane;
        sdc_future_s = sdc_s +  (sdc_speed*elapsed_time)*.447038889;  // Convert speed in mph to delta s in meters
        //sdc_future_d = future_lane*LANE_WIDTH + 2;
        sdc_future_speed = sdc_speed;
        //cout << "sdc: cur s,future s=" << sdc_s << "," << sdc_future_s << endl;
        //cout << "sdc: cur d,future d=" << sdc_d << "," << sdc_future_d << endl;

        
    }
    
    
    
    
    
    //
    // Per Path Planning approach, update the car's behavior based on telemetry data, car state, and sensor data.
    // This is a predicition because Trajectory update can always
    // Input: Car data & Car State data (previous path & sensor data)
    // Output: target lane (sdc_lane) and velocity (sdc_velocity). Availablity internally & by Get
    void update_Behavior(const vector<double> &previous_path_x, const vector<double> &previous_path_y, \
                         const vector<vector<double>> &sensor_fusion) {
        
    
        /* A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.
         */
        
        
        
        
        
   

    //
    // The key final loop to this is a setting of car velocity and lane selection based on The FSM selection of the best state
    // with minimum cost
    
    
    
    };
    
  
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
        double ref_yaw = deg2rad(sdc_yaw); // car yaw (degrees)?? units but used later
        
        //cout << ref_x << " " << ref_y << " " << car_yaw << " " << ref_yaw << endl;
        double car_yaw_r = deg2rad(sdc_yaw);
       
        
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
            
            double N = (target_dist/(.02*target_vel/2.24)); // convert from mph to m/s
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
    
    
    //
    // Getters
    //
    double get_s() {
        return sdc_s;
    }
    
    
    SelfDrivingCar::State get_State(){
        return sdc_state;
    }
    
    int get_lane(){
        return sdc_lane;
    }
    
    
    
    
    SelfDrivingCar::State get_proposed_next_State(){
        return sdc_proposed_next_state;
    }
    
    double get_car_speed() {
        return sdc_speed;
    }
    
    
    double get_future_s() {
        return sdc_future_s;
    }
    
    double get_future_lane() {
        return sdc_future_lane;
    }
    
    double get_future_speed() {
        return sdc_future_speed;
    }
    
    double get_future_d() {
        return sdc_future_d;
    }
    
    
    // Status
    bool lane_change_in_process() {
        return sdc_lane_change_in_process;
    }
    
    
    
    
    
    //
    // Setters
    //
    void set_State(SelfDrivingCar::State state) {
        sdc_state = state;
    return;
    }
    
    void update_lane(int updated_lane) {
        sdc_lane = updated_lane;
        return;
    }
    
    
    
    void set_proposed_next_State(SelfDrivingCar::State state) {
        sdc_proposed_next_state = state;
        return;
    }
    
    
    void set_lane_change_in_process(bool lane_change_status) {
        sdc_lane_change_in_process = lane_change_status;
    return;
    }
    
    
    
    
    //sdc_proposed_next_state = SelfDrivingCar::State::KeepLane;

}; // SelfDrivingCar Class


//
// General Helper Functions!!
//


vector<SelfDrivingCar::State> get_feasible_next_States(int &current_lane) {

    vector<SelfDrivingCar::State> valid_states;
  
    
    valid_states = {};
    
    // Left lane
    if (current_lane == 0) {
        valid_states = {SelfDrivingCar::State::KeepLane, SelfDrivingCar::State::LaneChangeRight};
    
    // Middle lane
    } else if (current_lane == 1) {
        valid_states = {SelfDrivingCar::State::LaneChangeLeft, SelfDrivingCar::State::KeepLane, SelfDrivingCar::State::LaneChangeRight,};
    
    // Right lane
    } else if (current_lane == 2) {
        valid_states = {SelfDrivingCar::State::LaneChangeLeft, SelfDrivingCar::State::KeepLane};
    
    } else {
        cout << "Error: invalid lane=" << current_lane << endl;
    }
    
    return valid_states;
}


//
// Road Architecture - Define feasible future lanes for a given Vehicle State from Finite State Machine (FSM)
//

vector<int> get_feasible_next_Lanes(SelfDrivingCar::State &next_state, int &current_lane) {
    
    vector<int> valid_Lanes;
    
    valid_Lanes = {};
    
    
    if (next_state == SelfDrivingCar::State::KeepLane) {
        valid_Lanes = {current_lane};
    }  else if (next_state == SelfDrivingCar::State::LaneChangeLeft) {
        valid_Lanes = {current_lane-1};
    }  else if (next_state == SelfDrivingCar::State::LaneChangeRight) {
        valid_Lanes = {current_lane+1};
    }
    
    
    /*
    if (state == SelfDrivingCar::State::LaneChangeRight) {
        valid_Lanes = {0,1};
    } else if (state == SelfDrivingCar::State::KeepLane) {
        valid_Lanes = {0,1,2};
    } else if (state == SelfDrivingCar::State::LaneChangeLeft) {
        valid_Lanes = {1,2};
    }
    */
     
    return valid_Lanes;
}







//
// More Helper Functions - Going into COST.H!!!
//



double cost_Collision_Ahead(SelfDrivingCar &sdc, array<vector<Tracked_Vehicle>,NUM_LANES> &cars_ahead, \
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

    /*
    // Cost = 1 if collide, 0 otherwise. Will have large weight factor
    if (cars_ahead[proposed_lane].size() > 0) {
    
        get_min_ahead_cars(cars_ahead[proposed_lane], min_ahead_car);
        min_ahead_car.project_future_self(TIME_AHEAD, proposed_lane);
        double delta_car_s =  min_ahead_car.get_future_s() - sdc.get_future_s();
        if (abs(delta_car_s) <= 10.0) {
            cost = 1.0;
            cout << "1. cost: collision ahead=" << min_ahead_car.get_future_s() << "," << sdc.get_future_s() << "," << delta_car_s << "," << 1.0 << endl;
            return cost;
        } else {
             cout << "1. cost: collision ahead=0.0" << endl;
            return 0.0;
        }
        
        //---
        get_min_ahead_cars(cars_ahead[proposed_lane],min_ahead_car);  // <TODO> Change function name to get_min_car_ahead_from_cars
        min_ahead_car.project_future_self(TIME_AHEAD, proposed_lane);
    
    
        //double delta_car_s =  min_ahead_car.get_future_s() - sdc.get_future_s();
        double delta_car_s =  min_ahead_car.get_future_s() - sdc.get_future_s();
        if (delta_car_s <= 10.0) {   // 10m buffer for car length
            cost = 1.0;
        } else {
            cost = 0.0;
        }
     //---
     
    }
    cout << "1. cost: collision=0.0" << endl;
    return cost;
     */
}


double cost_Lane_Movement(SelfDrivingCar &sdc, array<vector<Tracked_Vehicle>,NUM_LANES> &cars_ahead, \
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


double cost_Closest_Vehicle_Ahead(SelfDrivingCar &sdc, array<vector<Tracked_Vehicle>,NUM_LANES> &cars_ahead, \
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


double cost_Speed(SelfDrivingCar &sdc, array<vector<Tracked_Vehicle>,NUM_LANES> &cars_ahead, \
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
    
    
    
    
    // Old Speed
    /*
    double cost = (MAX_SPEED - sdc.get_future_speed())/MAX_SPEED;
    cout << "4. cost: speed=" << cost << endl;
    */
    
    return cost;
}


double cost_Number_Of_Vehicles_In_Lane_Ahead(SelfDrivingCar &sdc, array<vector<Tracked_Vehicle>,NUM_LANES> &cars_ahead, int &proposed_lane) {
    
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


double cost_Preferred_Lane(int &proposed_lane) {
    
    // Cost is 0=middle, 1=left or right lane (probably should change to more left (fast) lane than right lane??
    double cost = (double) abs(1 - proposed_lane);
    cout << "6. cost: preferred lane=" << cost << endl;
    
    return cost;
}


// NEED TO PASS ARRAYS av1 ,cars_ahead, cars_behind


double cost_For_Proposed_Trajectory(SelfDrivingCar &sdc, array<vector<Tracked_Vehicle>,NUM_LANES> &cars_ahead, \
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



    
//---
// Main method for Path Planning & Message Handling with Simulator Server
//---
int main() {
     cout << "Main: Path Planning project code start..." << endl;
    
    //---
    // NOTE: Key constants & init values for main
    //---
    //string map_file_ = "../data/highway_map.csv";  // Waypoint map to read from
    string map_file_ = "../data/highway_map_bosch1.csv";
    //string map_file_ = "../data/highway_map_bosch1.csv";  // Waypoint map to read from
    //double const FULL_TRACK_S = 6945.554;          // s value before wrapping around the track back to 0 (meters)
    int lane = 1;                                  // Define lane (0=Far Left, 1=Center, 2=Far Right)
    //double ref_v = 49.25;                      // Car speed (start at max)
    double ref_vel = 0.0;                      // Car speed (start at 10 mph)
    //double dt = .02;                        // This project is a perfect simulator and delta time is fixed per frame (sec)
    //double fps = 1.0/dt;
    //---
    int lane_change_in_progress_cnt = 0;

    long frame_cnt = 0L;
    double elapsed_time = 0.0;
    
    
    //int cars_ahead[NUM_LANES][12];
    //array<vector<float>,NUM_LANES> cars_ahead = {};
    
    /*
    cars_ahead[0].push_back(3.5);
    cars_ahead[0].push_back(4.5);
    cars_ahead[0].push_back(2.5);

    cars_ahead[1].push_back(13.5);
    cars_ahead[1].push_back(14.5);
    
    cars_ahead[2].push_back(.5);
    cars_ahead[2].push_back(45.5);
    cars_ahead[2].push_back(3.5);
    cars_ahead[2].push_back(2.5);
    
    
    cout << "lane0=" << cars_ahead[0].size() << " lane1=" << cars_ahead[1].size() << " lane2=" << cars_ahead[2].size() << endl;
    
    sort(cars_ahead[0].begin(), cars_ahead[0].end());
    sort(cars_ahead[1].begin(), cars_ahead[1].end());
    sort(cars_ahead[2].begin(), cars_ahead[2].end());

    
    cout << cars_ahead[0][0] << " " <<  cars_ahead[0][1] << " " <<  cars_ahead[0][2] << endl;
    cout << cars_ahead[1][0] << " " <<  cars_ahead[1][1] << endl;
    cout << cars_ahead[2][0] << " " <<  cars_ahead[2][1] << " " <<  cars_ahead[2][2] << endl;
    cars_ahead = {};
    cout << "lane0=" << cars_ahead[0].size() << " lane1=" << cars_ahead[1].size() << " lane2=" << cars_ahead[2].size() << endl;
     */
    
    // More complicated Array of vectors
    array<vector<Tracked_Vehicle>,NUM_LANES> cars_ahead2 = {};
    
    Tracked_Vehicle tracked_vehicle(1.0,1.0,1.0,1.0,1.0,1.0,1.0);
    
    tracked_vehicle.setData(1.0,1.0,1.0,1.0,1.0,0.75,1.0);
    cars_ahead2[0].push_back(tracked_vehicle);
    tracked_vehicle.setData(1.0,1.0,1.0,1.0,1.0,0.35,1.0);
    cars_ahead2[0].push_back(tracked_vehicle);
    tracked_vehicle.setData(1.0,1.0,1.0,1.0,1.0,0.5,1.0);
    cars_ahead2[0].push_back(tracked_vehicle);
    
    tracked_vehicle.setData(1.0,1.0,1.0,1.0,1.0,0.65,1.0);
    cars_ahead2[1].push_back(tracked_vehicle);
    
    tracked_vehicle.setData(1.0,1.0,1.0,1.0,1.0,0.25,1.0);
    cars_ahead2[1].push_back(tracked_vehicle);
    
    /*
    cout << "l0=" << cars_ahead2[0].size() << " l1=" << cars_ahead2[1].size() << " l2=" << cars_ahead2[2].size() << endl;
    */
    
    //sort(cars_ahead2[0].begin(), cars_ahead[0].end(),SortByAscending);
    //sort(cars_ahead2[0].begin(), cars_ahead2[0].end());  // Sorts by ascending S based on class definition
    //sort(cars_ahead2[0].begin(), cars_ahead2[0].end(), SortByAscendingS());  // Sorts by ascending S based on class definition
    
    /*
    for (vector<Tracked_Vehicle>::iterator it=cars_ahead2[0].begin(); it!=cars_ahead2[0].end(); ++it) {
        cout << it->s << ",";
    }
    cout << endl;
    */
     
    sort(cars_ahead2[0].begin(), cars_ahead2[0].end(), SortByAscendingS());  // Sorts by ascending S based on class definition
    
    /*
    for (vector<Tracked_Vehicle>::iterator it=cars_ahead2[0].begin(); it!=cars_ahead2[0].end(); ++it) {
        cout << it->s << ",";
    }
    cout << endl;
    */
     
   // Tracked_Vehicle first_vehicle(cars_ahead2[0].pop_back());
    Tracked_Vehicle first_vehicle = {};
    first_vehicle = cars_ahead2[0].front(); // Right now this is a copy!!!!
    
    /*
    //double first_vehicle_s = first_vehicle.getS;
    cout << "first vehicle s=" << first_vehicle.getS() << endl;
    cout << "first vehicle s=" << cars_ahead2[0].front().getS() << endl;
    cout << "# of cars in lane 0=" << cars_ahead2[0].size() << endl;
     */
     
    /*
    for (vector<int>::iterator it = cars_ahead2[0].begin(); it != cars_ahead2[0].end(); ++it) {
        cout << "\t" << *it;
    }
    cout << endl;
    */
    
    /*
    for (const auto& sval: cars_ahead2[0].getS)
        std::cout << sval << std::endl;
    */
    
    /*
    for (vector<Tracked_Vehicle>::iterator it = cars_ahead2[0].begin(); it!=cars_ahead2[0].end(); ++it) {
        cout << it->s << ",";
    }
    cout << endl;
    */
     
    // Next experiment is an array to hold objects because can then guarantee sort will work
    // More complicated Array of vectors
    vector<Tracked_Vehicle> l0_cars_ahead3 = {};
    vector<Tracked_Vehicle> l1_cars_ahead3 = {};
    
    
    tracked_vehicle.setData(1.0,1.0,1.0,1.0,1.0,0.75,1.0);
    l0_cars_ahead3.push_back(tracked_vehicle);
    
    tracked_vehicle.setData(1.0,1.0,1.0,1.0,1.0,0.35,1.0);
    l0_cars_ahead3.push_back(tracked_vehicle);
    
    tracked_vehicle.setData(1.0,1.0,1.0,1.0,1.0,0.65,1.0);
    l1_cars_ahead3.push_back(tracked_vehicle);
    
    tracked_vehicle.setData(1.0,1.0,1.0,1.0,1.0,0.25,1.0);
    l1_cars_ahead3.push_back(tracked_vehicle);
    
    tracked_vehicle.setData(1.0,1.0,1.0,1.0,1.0,0.5,1.0);
    l0_cars_ahead3.push_back(tracked_vehicle);
    
    /*
    for (vector<Tracked_Vehicle>::iterator it=l0_cars_ahead3.begin(); it!=l0_cars_ahead3.end(); ++it) {
        cout << it->s << ",";
    }
    cout << endl;
    */
     
    sort(l0_cars_ahead3.begin(), l0_cars_ahead3.end(), SortByAscendingS());  // Sorts by ascending S based on class definition
    sort(l1_cars_ahead3.begin(), l1_cars_ahead3.end(), SortByAscendingS());  // Sorts by ascending S based on class definition
    
    /*
    for (vector<Tracked_Vehicle>::iterator it=l0_cars_ahead3.begin(); it!=l0_cars_ahead3.end(); ++it) {
        cout << it->s << ",";
    }
    cout << endl;
    
    for (vector<Tracked_Vehicle>::iterator it=l1_cars_ahead3.begin(); it!=l1_cars_ahead3.end(); ++it) {
        cout << it->s << ",";
    }
    cout << endl;
     */
    
    
    
    
    
    
    // Create Websocket message object
    uWS::Hub h;
   

    //
    // Section to load values for Waypoint's x,y,s and d normalized normal vectors
    //
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
    cout << "Main: Waypoint file=" << map_file_ << ". # of Waypoints read=" << map_waypoints_x.size() << endl;

    // Init Self Driving Car object
    SelfDrivingCar av1;
    
    
    //---
    // Message Handling Section
    //there's a websocket message event.
    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, \
                 &av1, &frame_cnt, &elapsed_time, &lane_change_in_progress_cnt] \
                 (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    
    
    
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
          	double car_speed = j[1]["speed"];  // car speed in map coord (MPH!)

            // Previous path's end s and d values
            double end_path_s = j[1]["end_path_s"];  // car path far end s (meters) in Frenet coord (along track)
            double end_path_d = j[1]["end_path_d"];  // car path far end d (meters) in Frenet coord (displacement from center)
            
          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];  // car's previous frame full trajectory path x (meters) in map coord
          	auto previous_path_y = j[1]["previous_path_y"];  // car's previous frame full trajectory path y (meters) in map coord
          	
          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"]; // Array of detected other cars from car's sensors
            
            //---
            //
            // Step #1: Update Car's Localization data
            //
            av1.update_LocalizationData(car_x,car_y,car_s,car_d,car_yaw,car_speed,end_path_s,end_path_d);
            //cout << "after Localization store" << endl;
            
            //
            // Step #2. Update Car's Sensor Localization Data (i.e. other nearby cars called "sensor fusion")
            // THIS WILL BECOME av1.update_Tracked_Vehicles
            // Experiment
            array<vector<Tracked_Vehicle>,NUM_LANES> cars = {};
            array<vector<Tracked_Vehicle>,NUM_LANES> cars_ahead3 = {};
            array<vector<Tracked_Vehicle>,NUM_LANES> cars_behind3 = {};
           
            array<vector<Tracked_Vehicle>,NUM_LANES> cars_ahead = {};
            array<vector<Tracked_Vehicle>,NUM_LANES> cars_behind = {};
            //array<double,NUM_LANES> min_ahead_delta_s = {__DBL_MAX__, __DBL_MAX__, __DBL_MAX__};
            array<double,NUM_LANES> min_ahead_delta_s = {};
            array<double,NUM_LANES> min_behind_delta_s = {};

            
            vector<Tracked_Vehicle> l0_cars_ahead = {};
            vector<Tracked_Vehicle> l0_cars_behind = {};
            vector<Tracked_Vehicle> l1_cars_ahead = {};
            vector<Tracked_Vehicle> l1_cars_behind = {};
            vector<Tracked_Vehicle> l2_cars_ahead = {};
            vector<Tracked_Vehicle> l2_cars_behind = {};
            Tracked_Vehicle tracked_car = {};
            
            
            for (int i=0; i<sensor_fusion.size(); i++) {
                
                float d = sensor_fusion[i][6];
                float delta_s = (float)sensor_fusion[i][5] - car_s;
                
                if (d >= 0.0 && d <= 12.0) {
                    
                    // Put tracked cat here!!!!!
                    
                    
                    
                    if (d >= 0.0 && d < 4.0) {
                        cout << "l0: id,d,sensor,sdc,delta_s=" << i     << " " << d << " " << (float)sensor_fusion[i][5] << " " \
                                                            << car_s << " " << delta_s << endl;
                        tracked_car.add_Sensor_Fusion_Data(sensor_fusion[i]); // TODO Change to pass delta_s so one less call
                        tracked_car.setDelta_S(delta_s);
                        
                        if ((delta_s >= 0.0) && (delta_s <= HORIZON)) {
                            cars_ahead[0].push_back(tracked_car);
                        } else if ((delta_s < 0.0) && (delta_s >= -HORIZON)) {
                            cars_behind[0].push_back(tracked_car);
                        }
                        
                    } else if (d >= 4.0 && d < 8.0) {
                        
                        cout << "l1: id,d,sf,cars,delta_s=" << i     << " " << d << " " << (float)sensor_fusion[i][5] << " " \
                                                            << car_s << " " << delta_s << endl;
                        
                        tracked_car.add_Sensor_Fusion_Data(sensor_fusion[i]);
                        tracked_car.setDelta_S(delta_s);
                        
                        if ((delta_s >= 0.0) && (delta_s <= HORIZON)) {
                            cars_ahead[1].push_back(tracked_car);
                        } else if ((delta_s < 0.0) && (delta_s >= -HORIZON)) {
                            cars_behind[1].push_back(tracked_car);
                        }

                    } else if (d >= 8.0 && d <= 12.0) {

                        cout << "l2: id,d,sf,cars,delta_s=" << i     << " " << d << " " << (float)sensor_fusion[i][5] << " " \
                                                            << car_s << " " << delta_s << endl;
                        
                        tracked_car.add_Sensor_Fusion_Data(sensor_fusion[i]);
                        tracked_car.setDelta_S(delta_s);
                        
                        if ((delta_s >= 0.0) && (delta_s <= HORIZON)) {
                            cars_ahead[2].push_back(tracked_car);
                        } else if ((delta_s < 0.0) && (delta_s >= -HORIZON)) {
                            cars_behind[2].push_back(tracked_car);
                        }
                    }
                }
            } // for
            
            
            /* - Sort Experiment
            for (vector<Tracked_Vehicle>::iterator it = cars_ahead[0].begin(); it!=cars_ahead[0].end(); ++it) {
                cout << it->delta_s << ",";
            }
            cout << endl;
            
            sort(l0_cars_ahead.begin(), l0_cars_ahead.end(), SortByAscendingDeltaS()); // Address of original to re-order
            sort(cars_ahead[0].begin(), cars_ahead[0].end(), SortByAscendingDeltaS()); // Address of original to re-order
    
            
            double min_delta_s = __DBL_MAX__;
            for (vector<Tracked_Vehicle>::iterator it = cars_ahead[0].begin(); it!=cars_ahead[0].end(); ++it) {
                min_delta_s = min(min_delta_s, it->delta_s);
            }
            */
             
            /* - Single Debug
            cout << "L0: # of cars in lanes AHEAD =" << l0_cars_ahead.size();
            cout << "L0: # of cars in lanes AHEAD =" << cars_ahead[0].size();
            if (l0_cars_ahead.size() > 0) { cout << " closest=" << l0_cars_ahead.front().getDelta_S() << endl; };
            cout << "L0: min delta_s=" << min_delta_s;
            //cout << "L0: closest delta_s ahead=" << min_Delta_S(l0_cars_ahead) << endl;
            cout << "L0: closest delta_s ahead=" << min_Delta_S(cars_ahead[0]) << endl;
            */
            
            // Next - DO AL 3 SIMULTANEOUSLY
            // Store tracked min car ahead & behind for each lane
            // PROGRAM THIS
            // Store min cars ahead for each lane
            array<Tracked_Vehicle,NUM_LANES> min_ahead_cars = {};
            array<Tracked_Vehicle,NUM_LANES> min_behind_cars = {};
            for (int i=0; i<NUM_LANES; i++) {
                // <TODO> GET RID OF ONE OR THE OTHER
                get_min_ahead_cars(cars_ahead[i],min_ahead_cars[i]);
                get_min_behind_cars(cars_behind[i],min_behind_cars[i]);
                min_ahead_delta_s[i] = min_Delta_S(cars_ahead[i]);       // Switch to object
                min_behind_delta_s[i] = max_Delta_S(cars_behind[i]);     // Switch to object
            }
            
            // Debug mins
            cout << "SDC current: lane,speed=" << lane << " , " << ref_vel << " mph" << endl;
            cout << "Cars ahead:" << endl;
            for (int i=0; i<NUM_LANES; i++) {
                cout << "L" << i << " # of cars=" << cars_ahead[i].size() << " min s=" << min_ahead_delta_s[i] <<  endl;
            }
            cout << "Cars behind:" << endl;
            for (int i=0; i<NUM_LANES; i++) {
                cout << "L" << i << " # of cars=" << cars_behind[i].size() << " min s=" << min_behind_delta_s[i] << endl;
            }
            
            
            /* throw away
            cout << "lanes min ahead =";
            for (int i=0; i<NUM_LANES; i++) {
                cout << min_ahead_delta_s[i] << ",";
            }
            cout << endl;
            cout << "lanes min behind=";
            for (int i=0; i<NUM_LANES; i++) {
                cout << min_behind_delta_s[i] << ",";
            }
            cout << endl;
            }
            */
            
            //
            // !! Safety Check (NOTE: NEED TO FIGURE OUT WHERE TO PUT SPEED BACK UP)
            //
            if (min_ahead_delta_s[lane] <= 10.0) {
               // ref_vel -= .4; // Emergency 20 mph/1sec drop (at project limits)
                av1.set_State(SelfDrivingCar::State::EmergencyStop);
                cout << "Emergency Stop detected! Lane=" << lane << " min ahead detected=" << min_ahead_delta_s[lane] << endl;
            }
            
            
            //-----
            // Step #2: Update car's behavior based on localization data & state
            // This is to be called predict potnetial next states
            //
            av1.update_Behavior(previous_path_x, previous_path_y, sensor_fusion);
            cout << "after Behavior potential costs placeholder" << endl;

            //------
            // Section #2. - Prediction /  Proceess vehicle data
            //------
            // THIS SECTION IS SUPPOSE TO GENERATE POTENTIAL BEHAVIORS (TRAJECTORIES) WITH ASSOCIATED COST function and output next proposed state
            vector<SelfDrivingCar::State>  feasible_states = {};
            SelfDrivingCar::State best_next_state = {};
            double cost;
            
            // row = state  , col=lane
            double projected_costs[4][3] = {
                                           {__DBL_MAX__,__DBL_MAX__,__DBL_MAX__},
                                           {__DBL_MAX__,__DBL_MAX__,__DBL_MAX__},
                                           {__DBL_MAX__,__DBL_MAX__,__DBL_MAX__},
                                           {__DBL_MAX__,__DBL_MAX__,__DBL_MAX__}, };
            
            double min_cost = __DBL_MAX__;
            vector<int> feasible_lanes = {};
            
            // Piviot on current self driving car State
            switch(av1.get_State()) {
                
                // Emergency State. Same new State until ahead_vehicle clears enough
                case (SelfDrivingCar::State::EmergencyStop):
                    
                    cout << "Prediction: Current State=" << (int)SelfDrivingCar::State::EmergencyStop << endl;
                    if (min_ahead_delta_s[lane] < 10.0) {
                        
                        // Fall through - Keep slowing down a
                        av1.set_proposed_next_State(SelfDrivingCar::State::EmergencyStop);
                        cout << "Prediction: Next State=" << (int)SelfDrivingCar::State::EmergencyStop << endl;
                    } else {
                        
                        // Can come out of it
                        av1.set_proposed_next_State(SelfDrivingCar::State::KeepLane);
                        cout << "Prediction: Next State=" << (int)SelfDrivingCar::State::KeepLane << endl;
                    }
                break;
                    
                // Current State = KeepLane.
                // All the ACTION This is where FSM calculates cost of 3 options and decides which one to do
                // Calc cost of staying in lane or switching left or right...
                case (SelfDrivingCar::State::KeepLane):
                    
                    // 1. Project sdc out 3 seconds, project tracked cars out 3 seconds
                    // 2. Calc costs
                    // 3. Choose min cost state
                                  //SelfDrivingCar::State next_state;

                    // Step #1. Project av1 & tracked vehicles forward in time & add to their objects
                    av1.project_future_self(TIME_AHEAD);
                    
                    // Project all surrounding vehicles
                    for (int lane=0; lane<=NUM_LANES-1; lane++) {
                        
                        // Detected cars ahead
                        for (vector<Tracked_Vehicle>::iterator it=cars_ahead[lane].begin(); it!=cars_ahead[lane].end(); ++it) {
                           // it->project_future_self(TIME_AHEAD,lane);
                        }
                        
                        // Detected cars behind
                        for (vector<Tracked_Vehicle>::iterator it=cars_behind[lane].begin(); it!=cars_behind[lane].end(); ++it) {
                          //  it->project_future_self(TIME_AHEAD,lane);
                        }
                    }
                    
                    
                    // Step #2. Calculate forecasted cost
                    
                     
                    /*
                    feasible_states = get_valid_potential_next_States(lane); // lane is current lane of av1
                    for (auto next_state : feasible_states) {
                        vector<int> feasible_lanes = {};
                        feasible_lanes = get_valid_potential_next_Lanes(next_state);
                        cout << " Forecasted cost: " << (int)next_state << " " << " cost=" << 0.0 << endl;
                    } // for
                    */
                    
                    
                    feasible_states = get_feasible_next_States(lane); // lane is current lane of av1
                    for (auto next_state : feasible_states) {
                        vector<int> feasible_lanes = {};
                        feasible_lanes = get_feasible_next_Lanes(next_state,lane);
                        for (auto proposed_lane : feasible_lanes) {
                            
                            int i = (int)next_state;
                            int j =( int)proposed_lane;
                            projected_costs[i][j] = cost_For_Proposed_Trajectory(av1, cars_ahead, cars_behind, proposed_lane);
                            //cout << "Cost: state=" << i << " lane=" << j << " cost=" <<  projected_costs[i][j] << endl;
                        } // for
                    } // for
                    
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
                        feasible_lanes = get_feasible_next_Lanes(next_state,lane);
                        for (auto proposed_lane : feasible_lanes) {
                            int i = (int)next_state;
                            int j = (int)proposed_lane;
                            cost = projected_costs[(int)next_state][(int)proposed_lane];
                            if (cost < min_cost) {
                                min_cost = cost;
                                best_next_state = next_state;
                            }
                        }
                    }
                    cout << "OK! current,best next,cost=" << (int)av1.get_State() << " , " << (int)best_next_state << " , " << min_cost << endl;
                    av1.set_proposed_next_State(best_next_state);
                
                break;
                
                    
                // Current State = LaneChangeLeft
                case (SelfDrivingCar::State::LaneChangeLeft):
                    
                    lane_change_in_progress_cnt += 1;
                    if (lane_change_in_progress_cnt <= (int)TIME_AHEAD*FPS) {   // ~2 seconds!! <TODO> 5 seconds right now
                        //av1.set_State(SelfDrivingCar::State::LaneChangeLeft); // Keep same status until complete
                        av1.set_proposed_next_State(SelfDrivingCar::State::LaneChangeLeft);
                    } else if (lane_change_in_progress_cnt >= (int)TIME_AHEAD*FPS) {
                        lane_change_in_progress_cnt = 0;
                        av1.set_proposed_next_State(SelfDrivingCar::State::KeepLane);
                        av1.set_State(SelfDrivingCar::State::KeepLane);  //ADDDED!!!
                        av1.update_lane(lane); // Commented out
                        av1.set_lane_change_in_process(false);
                    }
                    break;
                
                
                // Current State = Lane Change to Right
                case (SelfDrivingCar::State::LaneChangeRight):
                    
                    lane_change_in_progress_cnt += 1;
                    if (lane_change_in_progress_cnt < (int)TIME_AHEAD*FPS) {   // 3 seconds!!
                        //av1.set_State(SelfDrivingCar::State::LaneChangeRight); // Keep same status until complete (MIGHT NEED TO BE FUTURE) - NOT NEEDED
                        av1.set_proposed_next_State(SelfDrivingCar::State::LaneChangeRight);
                       // State(SelfDrivingCar::State::LaneChangeRight); // Keep same status until complete (MIGHT NEED TO BE FUTURE) - NOT NEEDED
                    } else if (lane_change_in_progress_cnt >= (int)TIME_AHEAD*FPS) {
                        lane_change_in_progress_cnt = 0;
                        av1.set_proposed_next_State(SelfDrivingCar::State::KeepLane);  // <TODO> Set to set current state
                        av1.set_State(SelfDrivingCar::State::KeepLane);  //ADDDED!!!
                        av1.update_lane(lane); // Not needed
                        av1.set_lane_change_in_process(false);
                    }
                    break;
            }; // switch
            
            
            //
            // Section #3 - Behavior / Determine Vehicle Situation
            // Generate optimal next state
            // WITH CURRENT STATE AND PROPOSED NEXT STATE,THIS SECTION IS SUPPOSED TO DECIDE LANE & VELOCITY
            // which means in each switch need to deal with 4 combinations (refer to matrix)
            
            
            double min_delta = min_ahead_delta_s[lane];
            //double car_ahead_vel = 49.5;
            Tracked_Vehicle min_car_ahead = {};
            
            // Piviot by new State!
            //switch(av1.get_State()) {
            
            SelfDrivingCar::State curr_state,next_state;
            
            curr_state = av1.get_State();
            
            // <TODO> See if experiement works
            //SelfDrivingCar::State next_state = av1.get_proposed_next_State(); // Change this naming to just "next_State"
            if (av1.lane_change_in_process()) {
                next_state = av1.get_State(); // Change this naming to just "next_State"
            } else {
                next_state = av1.get_proposed_next_State(); // Change this naming to just "next_State"
            }
            
        
            switch(next_state) {
                    
                // Emergency (until ahead_vehicle clears enough). Lane same,
               
                
                case (SelfDrivingCar::State::EmergencyStop):
                    
                      if (curr_state == SelfDrivingCar::State::EmergencyStop) {
                          
                          ref_vel -= .40; // Emergency 20 mph/1sec drop (at project limits)
                          cout << "ES: emergency decel - ref_vel=" << ref_vel << endl;
                          
                      } else if (curr_state == SelfDrivingCar::State::KeepLane) {
                          
                          ref_vel += .10*ref_vel; // Come out of it
                          cout << "ES: come out of it - ref_vel=" << ref_vel << endl;
                      
                      } else if (curr_state == SelfDrivingCar::State::LaneChangeRight) {
                          
                          lane -= 1;             // Abort lane change, return at same speed
                          av1.update_lane(lane);
                          cout << "TRAPPED: aborted Right Lane Change" << endl;
                          
                      } else if (curr_state == SelfDrivingCar::State::LaneChangeLeft) {
                          
                          lane += 1;             // Abort lane change, return at same speed
                          av1.update_lane(lane);
                          cout << "TRAPPED: aborted Left Lane Change" << endl;

                      }
                    
                    /*
                    if (av1.get_proposed_next_State() == SelfDrivingCar::State::EmergencyStop) {
                        ref_vel -= .50*ref_vel;  // Rapid slow down
                    } else {
                        ref_vel = 0.125;         // Slow increase out of emergency stop
                    }
                    */
                    
                    break;
                    
                    
                // InProcess until lane change completes 3 seconds - dont change anything
                
                /*
                case (SelfDrivingCar::State::LaneChangeInProcess):
                    cout << " Hello!" << endl;
                    break;
                */
                    
                // Stay in same lane Main State of driving in current lane
                case (SelfDrivingCar::State::KeepLane):
                    
                    if (curr_state == SelfDrivingCar::State::EmergencyStop) {
                        
                        ref_vel = 0.5;  // 1.0 (mph) Give small speed to slowly come out of ES in same lane
                    
                    // All other current states
                    } else {
                        
                        // All other current state actions the same - max speed if clear, slow down & match if car ahead
                        if (min_delta > 30.0) {   // meters
                            
                            ref_vel += .420; // (mph) .50     YUK last=.005 Put back to .4 but use low untl old code removed
                            ref_vel = min(ref_vel, MAX_SPEED);  // Cap just under speed limit so dont violate
                        
                        } else if ((min_delta > 10.0) && (min_delta <= 30.0)) {
                            //ref_vel -= .10; // mph
                            get_min_ahead_cars(cars_ahead[lane], min_car_ahead);
                            //double car_ahead_vel = min_car_ahead.v; // Make getv getter!!
                            double delta_speed_mph = min_car_ahead.v*2.236936292 - av1.get_car_speed();
                            cout << "##ADJUST SPEED to car ahead(mph)=" << (min_car_ahead.v*2.236936292) << "," \
                            << av1.get_car_speed() << "," << delta_speed_mph << "," << ref_vel << endl;;
                            ref_vel += .0175*delta_speed_mph;  // close delta speed in 1 second (1.0/(1.0*50))  1=100%, 1.0=1 second
                            //max(ref_vel, car_ahead_vel);
                        
                        // Back-off (could also develop a more sophisticated back-off to elongate and get out of boxing in
                        } else if ((min_delta > 5.0) && (min_delta <= 10.0)) {
                            ref_vel -= .01*ref_vel; //  1mph/1sec(1*50)
                            cout << "BACKOFF: delta mph=" << .005*ref_vel << "  " << min_delta << endl;
                        }
                    }
                    
                    /*
                    cout << "Current=Keep, Next=Keep, min_delta=" << min_delta << endl;
                    if (min_delta > 50.0) {
                        ref_vel += .20; // mph YUK .005 Put back to .4 but use low untl old code removed
                        ref_vel = min(ref_vel, 49.8);
                    } else if ( (min_delta > 10.0) && (min_delta <= 50.0) ) {
                        //ref_vel -= .10; // mph
                        get_min_ahead_cars(cars_ahead[lane], min_car_ahead);
                        //double car_ahead_vel = min_car_ahead.v; // Make getv getter!!
                        double delta_v = (min_car_ahead.v)*2.2369356 - av1.get_car_speed();
                        cout << "##ADJUST SPEED to car ahead(mph)=" << (min_car_ahead.v*2.2369356) << "," \
                             << av1.get_car_speed() << "," << delta_v << "," << ref_vel << endl;;
                        ref_vel += .006*delta_v;  // close delta speed in 3 second (1.0/(3*50))  1=100%
                        //max(ref_vel, car_ahead_vel);
                        // THIS IS WHERE I NEED minCar velocity!!!!!
                    } else {
                        // Emergency Stop
                        ref_vel -= .20; // reduce by mph now (20mph/50 fps)
                    }
                   */
                    
                break;
                
                
                // Next State = Lane Change to Left
                case (SelfDrivingCar::State::LaneChangeLeft):
                    // Note: Could upgrade with an abort lane change check here
                    
                    if (curr_state == SelfDrivingCar::State::KeepLane) {
                        
                        lane -= 1; // Change lane
                       // ref_vel += .001*ref_vel;  // Try a small speed increase to help with lateral acceleration
                        //av1.update_lane(lane);
                        av1.set_State(SelfDrivingCar::State::LaneChangeLeft); // <TODO> JUST ADDED
                        av1.set_lane_change_in_process(true);
                        cout << "Switched from KL to LCL. New lane=" << lane <<  endl;
                        break;
                    }
                    
                    cout << "Dont change 'lane' or 'ref_vel' for LCL until complete" << endl;
                break;
   
                    
                    
                
                
                // Next State = Lane Change to Right
                case (SelfDrivingCar::State::LaneChangeRight):
                    
                    // Note: Could upgrade with an abort lane change check here
                    //cout << "Dont change lane or velocity settings for RIGHT LANE CHANGE until complete" << endl;
                    
                    if (curr_state == SelfDrivingCar::State::KeepLane) {
                        
                        lane += 1; // When 1st triggered -  change "lane" @ same speed
                       // ref_vel += .001*ref_vel;  // Try a small speed increase to help with lateral acceleration
                       // av1.update_lane(lane);  //<TODO> Possibly delay this??????
                        av1.set_State(SelfDrivingCar::State::LaneChangeRight); // <TODO> JUST ADDED
                        av1.set_lane_change_in_process(true);
                        cout << "Switched from KL to LCR. New lane=" << lane << endl;
                        break;
                    }
                       cout << "Dont change 'lane' or 'ref_vel' for LCR until complete" << endl;
                break;
                    
                    
                    
                    
                    
                    
                    
                    
                    
            };  // switch
            
            
            //
            // Section #4 - Trajectory Generation Take Vehicle Action
            //
            
            // Simulator returns previous car path at each timestep
         
            //int detected_cars = sensor_fusion.size();
            //cout << " # of cars detected=" << detected_cars << " Previous car path points=" << prev_size << endl;
            //cout << "previous path points=" << prev_size << endl;
            
            
            // 1st thing is to check if car in front (this might go in "prediction" or behavior section. Yes predicition section as predicting
            // if going to run into car or need to brake
            
            //
            // This seems to have both "predicition" and  "behavior" in it
            //
            //cout << "old logic..." << endl;
            double ahead_car_speed;
            
            
            // ???????This is weird because we already hve car_s and this puts it at the end of last path
            int prev_path_size = previous_path_x.size();
            if (prev_path_size > 0) {
                car_s = end_path_s; // This means out at the far end of the trajectory
            }
            
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
                
                /* working piece!
                ref_vel += .425;
                ref_vel = min(ref_vel, 49.9);  // It isnt just this!! N points must be rounding cause it isnt going above 49.65...
                */
            
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
            double ref_yaw = deg2rad(car_yaw); // Need to convert to radians from degrees
            
            // 1st 2 anchor points are one point behind car and one point is car (makes transition smooth)
            if (prev_path_size < 2) {
            
                // Path tangent to current angle of the car
                double ref_yaw = deg2rad(car_yaw); // Need to convert degrees to radians
                double prev_car_x = car_x - cos(ref_yaw);
                double prev_car_y = car_y - sin(ref_yaw);
                
                // P1
                ptsx.push_back(prev_car_x);
                ptsy.push_back(prev_car_y);
                
                // P2
                ptsx.push_back(car_x);
                ptsy.push_back(car_y);
                
                
                cout << "prev<2 x=" << prev_car_x << " " << car_x << endl;
                cout << "prev<2 y=" << prev_car_y << " " << car_y << endl;
                
            
            } else {
            
                // Use 2 points that make path tangent to the previous path's end points
                ref_x = previous_path_x[prev_path_size-1];
                ref_y = previous_path_y[prev_path_size-1];
                double ref_x_prev = previous_path_x[prev_path_size-2];
                double ref_y_prev = previous_path_y[prev_path_size-2];
                // Where is translation????
                ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev); // ??? ref_yaw not BEING USED????? Being used later
                
               
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
            cout << "car_s=" << car_s << " " << 2+4*lane << endl;
            vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            cout << "wp0,1,2=" << next_wp0[0] << " " << next_wp1[0] << " " << next_wp2[0] << endl;
            cout << "wp0,1,2=" << next_wp0[1] << " " << next_wp1[1] << " " << next_wp2[1] << endl;
            
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
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Main: Connected!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Main: Disconnected!" << std::endl;
  });

    
    // Start of Main message handling
    int port = 4567;
    if (h.listen(port)) {
        cout << "Main: Path Planning message handler listening on port=" << port << "..." << endl;
    } else {
        cout << "Main: Path Planning message handler failed listening on port=" << port << " Exiting!" << endl;
    return -1;
    }
    
    h.run();   // run Websocket message handler
    return 0;  // TODO: find exit conditions of run
    
} // Main
