//
//  constants.hpp
//  Path_Planning
//
//  Created by Steve Horton on 8/31/17.
//
//

#ifndef constants_hpp
#define constants_hpp

//#include <stdio.h>


//
// Important Project LevelConstants: Simulator, Track, etc...
//
#define FULL_TRACK_S 6945.554  // One loop of Simulator track in s (meters) before wrapping around the track back to 0
#define DT           0.02      // Delta time of Simulator telemetry. NOTE: Fixed for this project (real-world it usually isnt)
#define FPS          50        // Associated frames/second of telemtry from Simulator
#define NUM_LANES    3         // Number of highway lanes. Fixed in this Simulator
#define LANE_WIDTH   4.0       // Width of each lane (meters)

#define HORIZON    175.0       // (meters) Horizon over which to track vehicles from sensor data (old: 200,300)
#define TIME_AHEAD   1.5       // 2.0 (seconds) Time projection ahead for rough trajectory (old: 1.0, 1,5, 1.75)

#define MAX_SPEED  49.75000     // (mph) Max speed allowed minus buffer (49.86 triggered max speed limit)
#define MAX_ACCEL   0.3000     // (mph)no=.4425  converted from 10m/s2 requirement to delta vel based on .02 second deltat - delta
                                 // Maximum 10ms2 accleration limit in mph/.02 sec for startup (0.447872584)
                                 // Need to subtract 2 m/s^2 lateral force (.09 mph) for total acceleration
#define EMERGENCY_DISTANCE 10.0    // 10.0 (meters) Safety distance to nearest car to invoke emergency manuevers

#define AHEAD_COLLISION_RISK   22.0 // (meters) minimum distance at full speed, more if lower speed
#define BEHIND_COLLISION_RISK  17.0 // (meters)



#define LANE_CHANGE_TIMER 1.5  // (seconds) Hold lane change before re-analyzing Keep or not (prevents back and forth lane changes)

#define SAFETY_COST 1000        // Cost function value risk threshold. If above, do not move lanes (consult weights for setting) 

#define MIN_CLEAR_AHEAD_FOR_FULL_SPEED  40.0  // (meters) Also triggers adjust speed to ahead car so little bigger is better
#define MIN_CLEAR_AHEAD_FOR_HOLD_SPEED  10.0  // (meters)


#define DEBUG      true

#endif /* constants_hpp */
