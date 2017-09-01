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
#define TIME_AHEAD   2.0       // (seconds) Time projection ahead for rough trajectory (old: 1.0, 1,5, 1.75)
#define MAX_SPEED  49.80       // (mph) Max speed allowed (too high 49.86)

#define SAFETY_DISTANCE 5.0    // (meters) Safety distance to nearest car to invoke emergency manuvers


#define DEBUG      true

#endif /* constants_hpp */
