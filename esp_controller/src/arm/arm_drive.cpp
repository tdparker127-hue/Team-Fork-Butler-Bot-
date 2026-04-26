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

// ---- Setpoints accumulated by integrating incoming rate commands ----
static double liftSetpoint = 0.0;  // rad
static double gripSetpoint = 0.0;  // rad

// ---- Pending rate commands written by parseArmSerial() ----
static volatile double liftRate = 0.0;  // rad/s
static volatile double gripRate = 0.0;  // rad/s

static unsigned long lastRateUpdate = 0;

void setupArm() {
    motorLift.setup();
    motorGrip.setup();
    lastRateUpdate = micros();
}

// Called by arm_main every serial-receive cycle.
// Integrates the rate commands into clamped position setpoints.
void updateArmSetpointRates(double newLiftRate, double newGripRate) {
    liftRate = newLiftRate;
    gripRate = newGripRate;
}

// Called at high frequency (2 kHz) to integrate setpoints and run position control.
void updateArmControl() {
    unsigned long now = micros();
    double dt = (now - lastRateUpdate) * 1e-6;
    lastRateUpdate = now;

    // Integrate rates into setpoints and apply soft limits
    liftSetpoint += liftRate * dt;
    liftSetpoint = constrain(liftSetpoint, MIN_LIFT_RAD, MAX_LIFT_RAD);

    gripSetpoint += gripRate * dt;
    gripSetpoint = constrain(gripSetpoint, MIN_GRIP_RAD, MAX_GRIP_RAD);

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
