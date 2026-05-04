#ifndef VL53L5CX_SENSOR_H
#define VL53L5CX_SENSOR_H

// Sensor field-of-influence radius and hard-stop distance (mm)
#define TOF_REP_RADIUS_MM 600
#define TOF_STOP_DIST_MM  500

struct ObstacleReading {
    float distance_m;  // closest obstacle distance in meters
    float angle_rad;   // horizontal angle (rad): positive = right, negative = left
    bool  valid;       // true if obstacle is within TOF_REP_RADIUS_MM
};

extern ObstacleReading obstacleReading;

void InitializeSensor();
void ToFData();
void ComputeObstacleReading();

#endif // VL53L5CX_SENSOR_H
