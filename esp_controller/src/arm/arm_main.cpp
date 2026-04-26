#include <Arduino.h>
#include "arm_drive.h"
#include "arm_pinout.h"
#include "imu.h"
#include "util.h"

IMU imu(ARM_BNO08X_RESET, ARM_BNO08X_CS, ARM_BNO08X_INT);

// ---------------------------------------------------------------------------
// Parse "lift:X;grip:Y;\n" from the Jetson.
// X and Y are absolute position setpoints in radians.
// The Jetson owns incremental stepping and soft limits; the ESP clamps as
// a hardware safety backstop.
// ---------------------------------------------------------------------------
static void parseArmSerial() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    double liftPos = 0.0, gripPos = 0.0;
    bool got_lift = false, got_grip = false;

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
        double val  = token.substring(colon + 1).toDouble();

        if      (key == "lift") { liftPos = val; got_lift = true; }
        else if (key == "grip") { gripPos = val; got_grip = true; }
    }

    if (got_lift && got_grip) {
        updateArmSetpoints(liftPos, gripPos);
    }
}

void setup() {
    Serial.begin(115200);
    setupArm();
    imu.setup();
}

void loop() {
    // Update IMU readings whenever data is ready
    imu.update();

    // Parse incoming Jetson commands at 50 Hz
    EVERY_N_MILLIS(20) {
        parseArmSerial();
    }

    // Run position control at 2 kHz (500 µs)
    EVERY_N_MICROS(500) {
        updateArmControl();
    }

    // Send IMU telemetry back to Jetson at 20 Hz
    // Format: "IMU:roll:X;pitch:X;yaw:X;rollRate:X;pitchRate:X;yawRate:X;\n"
    EVERY_N_MILLIS(50) {
        EulerAngles euler = imu.getEulerAngles();
        GyroReadings gyro  = imu.getGyroReadings();
        Serial.printf("IMU:roll:%.4f;pitch:%.4f;yaw:%.4f;rollRate:%.4f;pitchRate:%.4f;yawRate:%.4f;\n",
                      euler.roll, euler.pitch, euler.yaw,
                      gyro.rollRate, gyro.pitchRate, gyro.yawRate);
    }

    // Debug telemetry at 10 Hz (can be removed once tuned)
    EVERY_N_MILLIS(100) {
        Serial.printf("DBG:lift_sp:%.3f;lift_pos:%.3f;grip_sp:%.3f;grip_pos:%.3f;\n",
                      getLiftSetpoint(), getLiftPosition(),
                      getGripSetpoint(), getGripPosition());
    }
}
