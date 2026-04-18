#ifndef ROBOT_DRIVE_H
#define ROBOT_DRIVE_H

#define NUM_MOTORS 4

#define Kp 0.5//0.25 original 
#define Ki 0.05 //0.01 orinal 
#define Kd 0.1 //0 original
#define pidTau 0.1

#define MAX_FORWARD 8
#define MAX_TURN 5

void setupDrive();
void updateSetpoints(double FrRgt, double FrLft, double BkLft, double BkRgt);
void updatePIDs();

#endif // ROBOT_DRIVE_H