#ifndef ARM_DRIVE_H
#define ARM_DRIVE_H
#include "EncoderVelocity.h"
// ---- LeadLagFilter tuning (from Gripper_control_test.cpp baseline) ----
#define ARM_KP     0.936
#define ARM_ALPHA  10.0
#define ARM_TD     0.0021
#define ARM_TI     0.0183

// ---- Incremental rate integration ----
// Rate sent from Jetson (rad/s). Each loop the ESP integrates: setpoint += rate * dt
// and clamps to the soft limits below.

// ---- Soft limits (radians) — TODO: measure physical travel on the actual robot ----
// Run the arm manually, read encoderLift.getPosition() at each end-stop, then update these.
#define MIN_LIFT_RAD  0// Done!: lowered config set to min safe encoder position (e.g. arm fully lowered)
#define MAX_LIFT_RAD  3.5  // Done! maximum lift config set to max safe encoder position (e.g. arm fully raised)
//lift position for full actuation of the gripper = -3.62
#define MIN_GRIP_RAD   -1.85 // Done!: closed position position set to closed hard-stop position
#define MAX_GRIP_RAD   0// Done!: open positionset to open hard-stop position


// ---- Function declarations ----
void setupArm();
void updateArmSetpoints(double liftPos, double gripPos);
void updateArmControl();
double getLiftPosition();
double getGripPosition();
double getLiftSetpoint();
double getGripSetpoint();
double getGripEffort();
double getOutlierRejectedEncoderPos(EncoderVelocity& encoder, double prevPos);

#endif // ARM_DRIVE_H
