#ifndef ARM_DRIVE_H
#define ARM_DRIVE_H

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
#define MIN_LIFT_RAD  -3.0   // TODO: set to min safe encoder position (e.g. arm fully lowered)
#define MAX_LIFT_RAD   3.0   // TODO: set to max safe encoder position (e.g. arm fully raised)

#define MIN_GRIP_RAD  -2.0   // TODO: set to closed hard-stop position
#define MAX_GRIP_RAD   2.0   // TODO: set to open hard-stop position

// ---- Function declarations ----
void setupArm();
void updateArmSetpointRates(double liftRate, double gripRate);
void updateArmControl();
double getLiftPosition();
double getGripPosition();
double getLiftSetpoint();
double getGripSetpoint();

#endif // ARM_DRIVE_H
