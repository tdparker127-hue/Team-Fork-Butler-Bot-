#include <Arduino.h>
#include "util.h"
#include "robot_drive.h"
#include "EncoderVelocity.h"
#include "robot_motion_control.h"

// Define JETSON_SERIAL to receive wheel velocity setpoints over USB serial from the Jetson.
// When undefined, ESP-NOW wireless control (joystick) is used instead.
#define JETSON_SERIAL

#ifndef JETSON_SERIAL
#include "wireless.h"
#include "Robot_serial.cpp"
// #define UTURN
// #define CIRCLE
#define JOYSTICK
// #define YOUR_TRAJECTORY
extern RobotMessage robotMessage;
extern ControllerMessage controllerMessage;
#endif

int state = 0;
double robotVelocity = 0; // velocity of robot, in m/s
double k = 0; // curvature k is 1/radius from center of rotation circle

extern EncoderVelocity encoders[NUM_MOTORS];
double currPhiL = 0;
double currPhiR = 0;
double prevPhiL = 0;
double prevPhiR = 0;

// ---------------------------------------------------------------------------
// Jetson serial path — parse "lx:X;ly:X;yaw:X;\n" and call updateSetpoints.
// lx/ly/yaw are normalized [-1, 1] values; scaling and mecanum mixing happen
// here so all hardware-specific constants stay on the ESP.
// ---------------------------------------------------------------------------
#ifdef JETSON_SERIAL
void parseJetsonSerial() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    double lx = 0, ly = 0, yaw = 0;
    bool got_lx = false, got_ly = false, got_yaw = false;

    // First split: by ';'
    int start = 0;
    while (start < (int)line.length()) {
        int sep = line.indexOf(';', start);
        if (sep < 0) break;

        String token = line.substring(start, sep);
        start = sep + 1;
        if (token.length() == 0) continue;

        // Second split: by ':'
        int colon = token.indexOf(':');
        if (colon < 0) continue;

        String key = token.substring(0, colon);
        double val = token.substring(colon + 1).toDouble();

        if      (key == "lx")  { lx  = val; got_lx  = true; }
        else if (key == "ly")  { ly  = val; got_ly  = true; }
        else if (key == "yaw") { yaw = val; got_yaw = true; }
    }

    if (got_lx && got_ly && got_yaw) {
        // Scale normalized inputs to motor velocity setpoints (rad/s)
        double forward = ly  * MAX_FORWARD;
        double strafe  = lx  * MAX_FORWARD;
        double turn    = yaw * MAX_TURN;

        // Mecanum mixing — matches original followTrajectory() JOYSTICK mode:
        // updateSetpoints(FrLft, BkLft, FrRgt, BkRgt)
        updateSetpoints(forward + turn - strafe,
                        forward + turn + strafe,
                        forward - turn - strafe,
                        forward - turn + strafe);

        // updateSetpoints(turn - forward + strafe,
        //                 forward + turn + strafe,
        //                 forward - strafe + turn,
        //                 turn - strafe - forward);
    }
}
#endif // JETSON_SERIAL

// Sets the desired wheel velocities based on desired robot velocity in m/s
// and k curvature in 1/m representing 1/(radius of curvature)
// void setWheelVelocities(float robotVelocity, float k){
//     double left = (robotVelocity - k * WHEEL_BASE_R * robotVelocity) / WHEEL_R;
//     double right = 2 * robotVelocity / WHEEL_R  - left;
//     updateSetpoints(left, right);
// }

// Makes robot follow a trajectory
void followTrajectory() {

    #ifdef JETSON_SERIAL
    // All setpoints come directly from parseJetsonSerial(); nothing to do here.
    parseJetsonSerial();
    #endif

    #ifndef JETSON_SERIAL
    #ifdef JOYSTICK
    if (freshWirelessData) {
        double forward = abs(controllerMessage.joystick1.y) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.y, -1, 1, -MAX_FORWARD, MAX_FORWARD);
        double turn = abs(controllerMessage.joystick1.x) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.x, -1, 1, -MAX_TURN, MAX_TURN);
        double strafe =abs(controllerMessage.joystick2.x) <0.1 ?0: mapDouble(controllerMessage.joystick2.x, -1,1, -MAX_FORWARD, MAX_FORWARD);//want our strafe to be as fast as our forward basically? 
         //FrRight, FR left, Bk Left, Bk right
       updateSetpoints(turn - forward+strafe, forward +turn+strafe, forward - strafe+turn,turn-strafe-forward);
    } 
    #endif 

    #ifdef CIRCLE
    robotVelocity = 0.2;
    k = 1/0.5;
    setWheelVelocities(robotVelocity, k);
    #endif 

    #ifdef UTURN
    switch (state) {
        case 0: 
            // Until robot has achieved an x translation of 1 m:
            if (robotMessage.x <= 1.0) {
                // Move in a straight line forward
                robotVelocity = 0.2;
                k = 0;
            } else {
                // Move on to next state
                state++;
            }
            break;

        case 1:
            // Until robot has achieved a 180 deg turn in theta:
            if (robotMessage.theta <= M_PI) {
                // Turn in a circle with radius 25 cm 
                robotVelocity = 0.2;
                k = 1 / 0.25;
            } else {
                state++;
            }
            break;

        case 2:
            // Until robot has achieved an x translation of -1 m:
            if (robotMessage.x >= 0) {
                // Move in a straight line forward
                robotVelocity = 0.2;
                k = 0;
            } else {
                // Move on to next state
                state++;
            }
            break;

        default: 
            // If not in any of the states, robot should just stop
            robotVelocity = 0;
            k = 0;
            break;
    }
    setWheelVelocities(robotVelocity, k);
    #endif

    #ifdef YOUR_TRAJECTORY
    // TODO: Create a state machine to define your custom trajectory!



    setWheelVelocities(robotVelocity, k);
    #endif 
    #endif // ifndef JETSON_SERIAL

}

// void updateOdometry() {
//     // Take angles from traction (rear) wheels only since they don't slip
//     currPhiL = encoders[2].getPosition();
//     currPhiR = -encoders[3].getPosition();
    
//     // Update wheel angles and angular change
//     double dPhiL = currPhiL - prevPhiL;
//     double dPhiR = currPhiR - prevPhiR;
//     prevPhiL = currPhiL;
//     prevPhiR = currPhiR;

//     // Calculate update in robot's base coordinates
//     float dtheta = WHEEL_R / (2 * WHEEL_BASE_R) * (dPhiR - dPhiL);
//     float dx = WHEEL_R / 2.0 * (cos(robotMessage.theta) * dPhiR + cos(robotMessage.theta) * dPhiL);
//     float dy = WHEEL_R / 2.0 * (sin(robotMessage.theta) * dPhiR + sin(robotMessage.theta) * dPhiL);

//     // Update robot message 
//     robotMessage.millis = millis();
//     robotMessage.x += dx;
//     robotMessage.y += dy;
//     robotMessage.theta += dtheta;
// }
