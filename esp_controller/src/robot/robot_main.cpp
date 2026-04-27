#include <Arduino.h>
#include "robot_drive.h"
#include "robot_pinout.h"
#include "util.h"
#include "robot_motion_control.h"
#include "imu.h"

#ifndef JETSON_SERIAL
#include "wireless.h"
#endif

IMU imu(BNO08X_RESET, BNO08X_CS, BNO08X_INT);

void setup() {
    Serial.begin(115200);
    setupDrive();
    imu.setup();
#ifndef JETSON_SERIAL
    setupWireless();
#endif
}

void loop() {
    // Update IMU readings whenever data is ready
    imu.update();

    // Update velocity setpoints based on trajectory / serial input at 50Hz
    EVERY_N_MILLIS(20) {
        followTrajectory();
    }

    // Update PID at 200Hz
    EVERY_N_MILLIS(5) {
        updatePIDs();
    }

    // Send IMU telemetry back to Jetson at 20Hz
    // Format: "IMU:roll:X;pitch:X;yaw:X;rollRate:X;pitchRate:X;yawRate:X;\n"
    // The "IMU:" prefix lets the Jetson distinguish telemetry from debug prints.
    EVERY_N_MILLIS(50) {
        EulerAngles euler = imu.getEulerAngles();
        GyroReadings gyro  = imu.getGyroReadings();
        // Serial.printf("IMU:roll:%.4f;pitch:%.4f;yaw:%.4f;rollRate:%.4f;pitchRate:%.4f;yawRate:%.4f;\n",
        //               euler.roll, euler.pitch, euler.yaw,
        //               gyro.rollRate, gyro.pitchRate, gyro.yawRate);
    }

#ifndef JETSON_SERIAL
    // Send robot odometry back over ESP-NOW at ~3Hz
    EVERY_N_MILLIS(300) {
        sendRobotData();
    }
#endif
}