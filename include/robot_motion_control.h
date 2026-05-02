#ifndef ROBOT_MOTION_CONTROL_H
#define ROBOT_MOTION_CONTROL_H

// wheel radius in meters
#define WHEEL_R 0.06
// distance from back wheel to center in meters
#define WHEEL_BASE_R 0.2

//define potential field gains and variables
#define K_ATT 1.0;
#define K_REP 0.5;
#define stopDist 500; // in mm
#define RepRadius 600; // in mm
void followTrajectory();
void updateOdometry();
void parseJetsonSerial();

#endif