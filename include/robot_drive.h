#ifndef ROBOT_DRIVE_H
#define ROBOT_DRIVE_H

#define NUM_MOTORS 4

#define Kp 0.15//0.25 original 
#define Ki 0.01//0.01 orinal 
#define Kd 0.001 //0 original
#define pidTau 0.1 //0.1 original

#define MAX_FORWARD 6
#define MAX_TURN 5
#define MAX_Back 4 //want to slowly move backwards so we don't bump something on accident

void setupDrive();
void updateSetpoints(double FrLft, double BkLft, double FrRgt, double BkRgt);
void updatePIDs();
void getEncoderVelocities(float out[NUM_MOTORS]);

#endif // ROBOT_DRIVE_H