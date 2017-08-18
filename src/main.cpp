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


// For convenience
using json = nlohmann::json;
using namespace std;

// In-line functions for converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double meters2miles(double x) { return x * 0.000621371; }
double miles2meters(double x) { return x * 1609.3440; }
//double mph2mps(double x) { return x * 1609.3440; }

//
// Important Constants at Project Level: Simulator, Track, & ...
//
#define FULL_TRACK_S 6945.554  // One loop of Simulator track (meters)
#define FPS 50                 // Telemetry data frames/sec (FPS) from Simulator
#define DELTA_T 0.02           // Associated delta_t of FPS (sec)
#define LANE_WIDTH 4.0         // Width of each lane (meters)
#define MAX_LANES 3            // Maximum number of lanes on Width of each lane (meters)





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


///------------------
// Build Class of An Autnomous Vehicle
//


class SelfDrivingCar{
    
//
// Important Constants for Self Driving Car
//
#define HIWAY_SPEED_LIMIT_MPH 49.45  // = 79.58 km/h or 22.xx m/s
    
    
private:
    int sdc_lane;
    double sdc_x; double sdc_y;
    double sdc_s; double sdc_d;
    double sdc_yaw; double sdc_speed;
    double sdc_endpath_s; double sdc_endpath_d;
    
    double SAFETY_DISTANCE; //meters
    double target_vel;  // Target velocity used to set sdc speed through trajectory point spacing
    //double SPEED_LIMIT;// 49.5mph = 22.098m/s
    
    double sdc_future_s;
    string sdc_state; // state includes KL - Keep Lane / LCL - Lane Change Left / LCR - Lane Change Right
    vector<double> lane_speed;
    vector<double> lane_frontcar_s;
    vector<double> lane_backcar_s;
    
public:
    
    // Constructor
    SelfDrivingCar(){
        
        sdc_lane = 1;
        SAFETY_DISTANCE = 25.0;
        target_vel = 0.0;
        //SPEED_LIMIT_MPH = 49.5;
        lane_speed = {HIWAY_SPEED_LIMIT_MPH, HIWAY_SPEED_LIMIT_MPH, HIWAY_SPEED_LIMIT_MPH};
        lane_frontcar_s = {numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max()};
        lane_backcar_s = {numeric_limits<double>::min(), numeric_limits<double>::min(), numeric_limits<double>::min()};
        sdc_state = "KL";
        
        sdc_x = 0.0; sdc_y = 0.0; sdc_s = 0.0; sdc_d = 0.0;
        sdc_yaw = 0.0; sdc_speed = 0.0;
        sdc_endpath_s = 0.0; sdc_endpath_d = 0.0;
        
        
        
        cout << "SDC: Self Driving Car initialized w/ lane=" << sdc_lane << ", speed=" << target_vel << ", max speed=" << \
                HIWAY_SPEED_LIMIT_MPH << endl;
    }
    
    // Destructor
    virtual ~SelfDrivingCar() {}
    
    
    // Update car w/ localization data returned from Simulator.
    // Note: this is extra copy. Could be eliminated for speed but done for readability.
    void update_LocalizationData(double x, double y, double s, double d, double yaw, double speed, \
                                 double endpath_s, double endpath_d) {
        sdc_x = x;
        sdc_y = y;
        sdc_s = s;
        sdc_d = d;
        sdc_yaw = yaw;
        sdc_speed=speed;
        sdc_endpath_s = endpath_s;
        sdc_endpath_d = endpath_d;
    
    };
    
    //
    // Per Path Planning approach, update the car's behavior based on telemetry data, car state, and sensor data.
    // This is a predicition because Trajectory update can always
    // Input: Car data & Car State data (previous path & sensor data)
    // Output: target lane (sdc_lane) and velocity (sdc_velocity). Availablity internally & by Get
    void update_Behavior(const vector<double> &previous_path_x, const double &previous_path_y, \
                         const vector<vector<double>> &sensor_fusion) {
    
   

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
        double ref_y = sdc_y;
        double ref_yaw = deg2rad(sdc_yaw); //?? units but used later
        
        //cout << ref_x << " " << ref_y << " " << car_yaw << " " << ref_yaw << endl;
        //double car_yaw_r = deg2rad(car_yaw);
        int prev_size = previous_path_x.size();
        if (prev_size < 2) {
            
            // Path tangent to current angle of the car
            double prev_car_x = sdc_x - cos(sdc_yaw);  // Need to convert to radians?
            double prev_car_y = sdc_y - sin(sdc_yaw);
            
            //double prev_car_x = car_x - cos(car_yaw_r);  // Need to convert to radians?
            //double prev_car_y = car_y - sin(car_yaw_r);
            
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
        
        // Create 3 Ahead Anchor Points to Frenet Space & create more points
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
        // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
        vector<double> next_x_vals = {};
        vector<double> next_y_vals = {};
        
        for (int i=0; i<previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
        }
        
        // Generate points ahead of car along spline (in local/car coordinates) TODO Change x,y to car_, car_y
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
        }
        
        }; // update_Trajectory
        
        
        
        
    }; // SelfDrivingCar Class
    
    
    
    
//---
// Main method for Path Planning & Message Handling with Simulator Server
//---
int main() {
     cout << "Main: Path Planning project code start..." << endl;
    
    //---
    // NOTE: Key constants & init values for main
    //---
    string map_file_ = "../data/highway_map.csv";  // Waypoint map to read from
    //string map_file_ = "../data/highway_map_bosch1.csv";  // Waypoint map to read from
    //double const FULL_TRACK_S = 6945.554;          // s value before wrapping around the track back to 0 (meters)
    int lane = 1;                                  // Define lane (0=Far Left, 1=Center, 2=Far Right)
    //double ref_v = 49.25;                      // Car speed (start at max)
    double ref_vel = 0.0;                      // Car speed (start at max)
    double dt = .02;                        // This project is a perfect simulator and delta time is fixed per frame (sec)
    double fps = 1.0/dt;
    //---
    
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
    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel,&av1] \
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
          
            //
            // Section #1 : Get all data returned from Simulator
            //
            //double car_x;
            //auto previous_path_x = {NULL};
            //auto previous_path_y = {NULL};

            //double car_y;
            //double car_s;
            //double car_d;
            //double car_yaw;
            //double car_speed;
            
            // Previous path data given to the Planner
            //auto previous_path_x = j[1]["previous_path_x"];
            //auto previous_path_y = j[1]["previous_path_y"];
            
            // Previous path's end s and d values
            //double end_path_s = j[1]["end_path_s"];
            //double end_path_d = j[1]["end_path_d"];
            //convertServerJSONTelemetryToCarData(j, car_x);
            //convertServerJSONToCarTelemetryData(j,car_x, car_y, car_s, car_d, car_yaw, car_speed, previous_path_x,
            //previous_path_y, end_path_s, end_path_d);
            
            
            // j[1] is the data JSON object
        	// Main car's localization Data
          	double car_x = j[1]["x"];  // car x (meters) in map coord
          	double car_y = j[1]["y"];  // car y (meters) in map coord
          	double car_s = j[1]["s"];  // car s (meters) in Frenet coord (along track)
          	double car_d = j[1]["d"];  // car d (meters) in Frenet coord (displacement from center)
          	double car_yaw = j[1]["yaw"];  // car angle in map coord (degrees or radians????)
          	double car_speed = j[1]["speed"];  // car speed in map coord (MPH!)

            // Previous path's end s and d values
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];
            
            
          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	
            

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
            
            /* A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.
            */
            //---
            
            //
            // Section #2. - Prediction /  Proceess vehicle data
            //
            
            //
            // Section #3 - Behavior / Determine Vehicle Situation
            //
            
            //
            // Section #4 - Trajectory Generation Take Vehicle Action
            //
            
            // Simulator returns previous car path at each timestep
            int prev_size = previous_path_x.size();
            int detected_cars = sensor_fusion.size();
            cout << " # of cars detected=" << detected_cars << " Previous car path points=" << prev_size << endl;
            //cout << "previous path points=" << prev_size << endl;
            
            
            // 1st thing is to check if car in front (this might go in "prediction" or behavior section. Yes predicition section as predicting
            // if going to run into car or need to brake
            
            //
            // This seems to have both "predicition" and  "behavior" in it
            //
            
            if (prev_size > 0) {
                car_s = end_path_s; // This means out at the far end of the trajectory
            }
            
            bool too_close = false; // Assume ok until
            
            for (int i=0; i<sensor_fusion.size(); i++) {
            
                float d = sensor_fusion[i][6];  // displacement (d) of a detected car (lane it's in)
                cout << "d=" << d << endl;
                
                // If ith car is in my lane
                if (d > (2+4*lane-2) && d < (2+4*lane+2)) {
                
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double ahead_car_speed = sqrt(vx*vx + vy*vy);  // MPH??
                    double ahead_car_s = sensor_fusion[i][5];
                    
                    ahead_car_s += ((double)prev_size*.02*ahead_car_speed); // if using previous points can project s value out in time because using previous points
                    cout << "ahead_car_s=" << ahead_car_s << endl;
                    // If in front and less than gap, slow down relative to car in front
                    if ((ahead_car_s > car_s) && ((ahead_car_s - car_s) < 30)) {
                    
                        cout << "Car in my lane! Car#=" << i << " v=" << ahead_car_speed << " at " << ahead_car_s << endl;
                        
                        //ref_vel = .90*ahead_car_speed;
                        //ref_vel = 40.0;
                        too_close = true;
                        
                        if (lane == 1) {
                            lane = 0; // Move to left lane
                            cout << "Switch from center to left" << endl;
                            
                        } else if (lane == 0) {
                            lane = 1;
                            cout << "Switch from left to center" << endl;
                        }
                        
                    } // if
                } // if
            } // for
            
            
            //
            // Maybe this is supposed to be behavior section??
            //
            
            if (too_close) {
            
                //ref_vel -= .224; // ~ 5m/s**2 ??
                //ref_vel -= .250; // ~ 5m/s**2 ??
                //ref_vel -= .1250; // ~ 5m/s**2 ??
                ref_vel -= .1750; // ~ 5m/s**2 ??
    
            
            } else if (ref_vel < 49.40) {
            
                ref_vel += .50;
            
            }
            
            //
            // I think this starts the trajectory generation section
            //
            // Using 5 anchor points to send to spline 1 point behind car, car and 3 ahead at 30, 60 & 90 m
            
            
            // Widely spaced waypoints, evenly spaced at 30m (anchor points)
            vector<double> ptsx = {};  // Anchor points for trajectory spline
            vector<double> ptsy = {};
            
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw); //?
            
            //cout << ref_x << " " << ref_y << " " << car_yaw << " " << ref_yaw << endl;
            //double car_yaw_r = deg2rad(car_yaw);
            
            if (prev_size < 2) {
            
                // Path tangent to current angle of the car
                double prev_car_x = car_x - cos(car_yaw);  // Need to convert to radians?
                double prev_car_y = car_y - sin(car_yaw);
                
                //double prev_car_x = car_x - cos(car_yaw_r);  // Need to convert to radians?
                //double prev_car_y = car_y - sin(car_yaw_r);
            
                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);
                
                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            
            
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
            
            // Translate to Frenet Space & create more points
            vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
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
        
            cout << "ptsx,ptsy size=" << ptsx.size() << endl;
            //cout << "ptsx before spline=" << ptsx[0] << endl;
            //cout << "ptsy before spline=" << ptsy[0] << endl;
        
            
            s.set_points(ptsx,ptsy);  // Create spline from anchor points
            
            
            
            
            //---
            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            vector<double> next_x_vals = {};
            vector<double> next_y_vals = {};

            for (int i=0; i<previous_path_x.size(); i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }
            
            // Generate points ahead of car along spline (in local/car coordinates) TODO Change x,y to car_, car_y
            double target_x = 30.0;  // horizon meters ahead
            double target_y = s(target_x);
            double target_dist = sqrt((target_x*target_x) + (target_y*target_y));
            
            double x_add_on = 0;  // Start at vehicle x=0
            for (int i=1; i<=50-previous_path_x.size(); i++) {
            
                double N = (target_dist/(.02*ref_vel/2.24)); // convert from mph to m/s This is where point spread is made & velocity!!!
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
            
            // Do through Class
            vector<double> updated_x_points = {};
            vector<double> updated_y_points = {};
            //av1.update_Trajectory(previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s, \
            //                      updated_x_points, updated_y_points);
            
            
            
            
            /*
            // Second cut at car movement
            double dist_inc = 0.35;  // .5  (meters)
            double const LANE_WIDTH = 4.0;
            int const NUM_LANES = 3;
            for(int i = 0; i < 40; i++) // # of path points
            {
                // Create a path for car in Frenet
                double next_s = car_s + (i+1)*dist_inc;  // Start 1 point in front of car
                double next_d = LANE_WIDTH*1.50;  // Center Lane
                
                vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                next_x_vals.push_back(xy[0]);
                next_y_vals.push_back(xy[1]);
                
                
                // 1st pass - Convert to Map (x,y)
                //next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
                //next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
            }
            */
            
            
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
        cout << "Main: Path Planning message handler listening on port=" << port << endl;
    } else {
        cout << "Main: Path Planning message handler failed listening on port=" << port << " Exiting." << endl;
        cerr << "Main: Path Planning message handler failed listening on port=" << port << " Exiting." << endl;
    return -1;
    }
    
    h.run();   // run Websocket message handler
    return 0;  // TODO: find exit conditions of run
    
} // Main
