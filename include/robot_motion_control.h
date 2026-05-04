#ifndef ROBOT_MOTION_CONTROL_H
#define ROBOT_MOTION_CONTROL_H

// wheel radius in meters
#define WHEEL_R 0.045
// distance from back wheel to center in meters
#define WHEEL_BASE_R 0.2

// APF gain constants — tune these to adjust obstacle avoidance aggressiveness
#define K_ATT 1.0f   // attractive gain (scales joystick input)
#define K_REP 0.5f   // repulsive gain (scales sensor repulsion)

void followTrajectory();
void updateOdometry();
void parseJetsonSerial();

#endif // ROBOT_MOTION_CONTROL_H
