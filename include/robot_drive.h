#ifndef ROBOT_DRIVE_H
#define ROBOT_DRIVE_H

#define NUM_MOTORS 4

#define Kp 0.2//0.25 original 
#define Ki 0//0.01 orinal 
#define Kd 0.01 //0 original
#define pidTau 0.5 //0.1 original

#define MAX_FORWARD 8
#define MAX_TURN 5

void setupDrive();
void updateSetpoints(double FrLft, double BkLft, double FrRgt, double BkRgt);
void updatePIDs();

#endif // ROBOT_DRIVE_H