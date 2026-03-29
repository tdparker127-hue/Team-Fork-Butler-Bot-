#ifndef ROBOT_MOTION_CONTROL_H
#define ROBOT_MOTION_CONTROL_H

// wheel radius in meters
#define rr 0.06
// distance from back wheel to center in meters
#define br 0.2

void followTrajectory();
void updateOdometry();

#endif