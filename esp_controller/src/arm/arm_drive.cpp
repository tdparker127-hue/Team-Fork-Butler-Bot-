#include <Arduino.h>
#include "arm_pinout.h"
#include "arm_drive.h"
#include "MotorDriver.h"
#include "EncoderVelocity.h"
#include "SimpleFilters.h"
#include "util.h"

// ---- Hardware objects ----
static MotorDriver motorLift(LIFT_DIR_PIN, LIFT_PWM_PIN, 0);
static MotorDriver motorGrip(GRIP_DIR_PIN, GRIP_PWM_PIN, 1);

static EncoderVelocity encoderLift(LIFT_ENC_A_PIN, LIFT_ENC_B_PIN, CPR_312_RPM, 0.2);
static EncoderVelocity encoderGrip(GRIP_ENC_A_PIN, GRIP_ENC_B_PIN, CPR_312_RPM, 0.2);

static LeadLagFilter filterLift(ARM_ALPHA, ARM_TD, ARM_TI);
static LeadLagFilter filterGrip(ARM_ALPHA, ARM_TD, ARM_TI);

// ---- Setpoints commanded by Jetson (absolute positions in radians) ----
static double liftSetpoint = 0.0;
static double gripSetpoint = 0.0;

// ---- Grip effort output ----
static double effortGrip = 0.0;

// ---- Outlier rejection for encoder readings ----
static double prevPosGrip = 0.0;
static double prevPosLift = 0.0;
static double OUTLIER_RANGE = 0.3; // If encoder position changes by more than this much in one control loop, reject it as an outlier and use the previous position instead. (TUNING: 1.0 rad/s seems to work well to reject single outlier readings while still allowing fast movement.)
 

static double clampSetpoint(double value, double bound1, double bound2) {
    if (bound1 <= bound2) {
        return (value < bound1 ? bound1 : (value > bound2 ? bound2 : value));
    }
    return (value > bound1 ? bound1 : (value < bound2 ? bound2 : value));
}


void setupArm() {
    motorLift.setup();
    motorGrip.setup();
   
}

// Called by arm_main every serial-receive cycle.
// Accepts absolute position setpoints (rad) from the Jetson.
// The Jetson owns the incremental stepping and limit logic;
// the ESP clamps here as a hardware safety fallback (belt protection).
void updateArmSetpoints(double newLiftSetpoint, double newGripSetpoint) {
    liftSetpoint = clampSetpoint(newLiftSetpoint, MIN_LIFT_RAD, MAX_LIFT_RAD);
    gripSetpoint = clampSetpoint(newGripSetpoint, MIN_GRIP_RAD, MAX_GRIP_RAD);
}

// Called at high frequency (2 kHz) to run position control.
void updateArmControl() {
    // Position control via LeadLag (proportional + lead/lag compensation)
    double posLift =    getOutlierRejectedEncoderPos(encoderLift, prevPosLift);

    double errorLift = liftSetpoint - posLift;
    //motorLift.drive(ARM_KP*errorLift); //DEBUG
    double effortLift = ARM_KP * filterLift.calculate(errorLift);
    motorLift.drive(effortLift);

    double posGrip =   getOutlierRejectedEncoderPos(encoderGrip, prevPosGrip);  
    double errorGrip = gripSetpoint - posGrip;
    //motorLift.drive(ARM_KP*errorLift); //DEBUG
    effortGrip = ARM_KP * filterGrip.calculate(errorGrip);
    motorGrip.drive(effortGrip);

    prevPosLift = posLift;
    prevPosGrip = posGrip;
}

double getOutlierRejectedEncoderPos(EncoderVelocity& encoder, double prevPos) {
    double pos = encoder.getPosition();
    if (abs(pos - prevPos) > OUTLIER_RANGE) { // If the position change is unreasonably large, reject it as an outlier and return the previous position instead.
        return prevPos;
    }
    return pos;
}

double getLiftPosition() { return encoderLift.getPosition(); }
double getGripPosition()  { return encoderGrip.getPosition(); }
double getLiftSetpoint()  { return liftSetpoint; }
double getGripSetpoint()  { return gripSetpoint; }
double getGripEffort() {return effortGrip;}
