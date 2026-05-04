#include <Arduino.h>
#include "robot_drive.h"
#include "robot_pinout.h"
#include "util.h"
#include "robot_motion_control.h"
#include "imu.h"
#include "VL53L5cx_Sensor.h"

#ifndef JETSON_SERIAL
#include "wireless.h"
#endif

IMU imu(BNO08X_RESET, BNO08X_CS, BNO08X_INT);

void setup() {
    Serial.begin(115200);
    setupDrive();
    imu.setup();
    InitializeSensor();
#ifndef JETSON_SERIAL
    setupWireless();
#endif
}

void loop() {
    // Update IMU readings whenever data is ready
    imu.update();

    // Poll ToF sensor; updates obstacleReading used by APF in followTrajectory()
    EVERY_N_MILLIS(20) {
        ToFData();
    }

    // Update velocity setpoints based on trajectory / serial input at 50Hz
    EVERY_N_MILLIS(20) {
        followTrajectory();
    }

    // Update PID at 200Hz
    EVERY_N_MILLIS(5) {
        updatePIDs();
    }

    // Send IMU + encoder telemetry back to Jetson at 20Hz
    // IMU format:  "IMU:roll:X;pitch:X;yaw:X;rollRate:X;pitchRate:X;yawRate:X;\n"
    // ENC format:  "ENC:fl:X;bl:X;fr:X;br:X;\n"  (rad/s, FrLft BkLft FrRgt BkRgt)
    // The prefixes let the Jetson distinguish telemetry from debug prints.
    EVERY_N_MILLIS(50) {
        EulerAngles euler = imu.getEulerAngles();
        GyroReadings gyro  = imu.getGyroReadings();
        Serial.printf("IMU:roll:%.4f;pitch:%.4f;yaw:%.4f;rollRate:%.4f;pitchRate:%.4f;yawRate:%.4f;\n",
                      euler.roll, euler.pitch, euler.yaw,
                      gyro.rollRate, gyro.pitchRate, gyro.yawRate);
        float vels[NUM_MOTORS];
        getEncoderVelocities(vels);
        Serial.printf("ENC:fl:%.4f;bl:%.4f;fr:%.4f;br:%.4f;\n",
                      vels[0], vels[1], vels[2], vels[3]);
    }

#ifndef JETSON_SERIAL
    // Send robot odometry back over ESP-NOW at ~3Hz
    EVERY_N_MILLIS(300) {
        sendRobotData();
    }
#endif
}