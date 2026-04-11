#ifndef ROBOT_DRIVE_H
#define ROBOT_DRIVE_H

#define NUM_MOTORS 4

#define Kp 0.25
#define Ki 0.01
#define Kd 0
#define pidTau 0.1

#define MAX_FORWARD 12
#define MAX_TURN 12

void setupDrive();
void updateSetpoints(double frontleft, double frontright, double backleft, double backright);
void updatePIDs();

#endif // ROBOT_DRIVE_H