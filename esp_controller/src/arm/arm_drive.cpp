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

void setupArm() {
    motorLift.setup();
    motorGrip.setup();
}

// Called by arm_main every serial-receive cycle.
// Accepts absolute position setpoints (rad) from the Jetson.
// The Jetson owns the incremental stepping and limit logic;
// the ESP clamps here as a hardware safety fallback (belt protection).
void updateArmSetpoints(double newLiftSetpoint, double newGripSetpoint) {
    liftSetpoint = constrain(newLiftSetpoint, MIN_LIFT_RAD, MAX_LIFT_RAD);
    gripSetpoint = constrain(newGripSetpoint, MIN_GRIP_RAD, MAX_GRIP_RAD);
}

// Called at high frequency (2 kHz) to run position control.
void updateArmControl() {
    // Position control via LeadLag (proportional + lead/lag compensation)
    double posLift = encoderLift.getPosition();
    double errorLift = liftSetpoint - posLift;
    double effortLift = ARM_KP * filterLift.calculate(errorLift);
    motorLift.drive(effortLift);

    double posGrip = encoderGrip.getPosition();
    double errorGrip = gripSetpoint - posGrip;
    double effortGrip = ARM_KP * filterGrip.calculate(errorGrip);
    motorGrip.drive(effortGrip);
}

double getLiftPosition() { return encoderLift.getPosition(); }
double getGripPosition()  { return encoderGrip.getPosition(); }
double getLiftSetpoint()  { return liftSetpoint; }
double getGripSetpoint()  { return gripSetpoint; }
