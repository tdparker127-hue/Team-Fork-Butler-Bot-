#include <Arduino.h>
#include "robot_pinout.h"
#include "MotorDriver.h"
#include "PID.h"
#include "EncoderVelocity.h"
#include "robot_drive.h"
#include "util.h"
#include "SimpleFilters.h"

#define Ti 0.0183
#define Td 0.0021
#define Kp_p 0.936
#define alpha 10
#define ANALOGPIN A1  // For Circuit Playground Express
MotorDriver motorLinkage(A_DIR1, A_PWM1, 0); //armature motor 
EncoderVelocity encoderLinkage(ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_312_RPM, 0.2); //armature encoder
MotorDriver motorGrip(A_DIR2, A_PWM2, 0); //gripper motor
EncoderVelocity encoderGrip(ENCODER2_A_PIN, ENCODER2_B_PIN, CPR_312_RPM, 0.2); //gripper encoder
LeadLagFilter leadLag(alpha, Td, Ti);
double setpointL = 250;
double setpointG = 100;
double positionL = 0;
double positionG = 0;
double controlEffortL = 0;
double controlEffortG = 0;

#define MAX_FREQ 10.0 //rad/s
#define MAX_AMPLITUDE M_PI //rad
double freq = MAX_FREQ/2;
double amplitude = MAX_AMPLITUDE;

    void setup() {
    motorGrip.setup();
   //motorLinkage.setup();
    Serial.begin();
    }

void loop() {
// Update sinusiodal setpoint at 2kHz
// EVERY_N_MICROS(500) {
//     setpoint =  (analogRead(ANALOGPIN));


//update PID at 5khz
// EVERY_N_MICROS(200) {
// positionL = encoderLinkage.getPosition();
// controlEffortL = Kp_p*leadLag.calculate(setpointL-positionL);
// motorLinkage.drive(controlEffortL);

// }
EVERY_N_MICROS(200) {
positionG = encoderGrip.getPosition();
controlEffortG = Kp_p*leadLag.calculate(setpointG-positionG);
motorGrip.drive(controlEffortG);
}
// Print values at 50Hz
EVERY_N_MILLIS(20) {
Serial.printf("SetpointLINKAGE (rad): %.2f, Position (rad): %.2f, Control Effort: %.2f\n",
setpointL, positionL, controlEffortL);
delay(20);
Serial.printf("SetpointGRIPPER (rad): %.2f, Position (rad): %.2f, Control Effort: %.2f\n",
setpointG, positionG, controlEffortG);
}
}