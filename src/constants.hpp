//
//  constants.hpp
//  Path_Planning
//
//  Created by Steve Horton on 8/31/17.
//
//

#ifndef constants_hpp
#define constants_hpp

#include <stdio.h>


//
// Important Project LevelConstants: Simulator, Track, etc...
//
#define FULL_TRACK_S 6945.554  // One loop of Simulator track in s (meters) before wrapping around the track back to 0
#define DT           0.02      // Delta time of Simulator telemetry. NOTE: Fixed for this project (real-world it usually isnt)
#define FPS          50        // Associated frames/second of telemtry from Simulator
#define NUM_LANES    3         // Number of highway lanes. Fixed in this Simulator
#define LANE_WIDTH   4.0       // Width of each lane (meters)


#define HORIZON    175.0       // 200.0 300.0 (meters)  Horizon over which to track vehicles and sensor data
#define TIME_AHEAD   1.75       // 2.0 working 1.0 1.25 1.5 1.0 2.0 (seconds)  Projection ahead for rough trajectory (less time means safer distance)

#define MAX_SPEED  49.86       // !49.87 49.86 works 49.80 49.88 was too high, (49.875,49.8725,49.87120, 49.871, 49.87) too high for max accel speed variations

#define DEBUG      true







#endif /* constants_hpp */
