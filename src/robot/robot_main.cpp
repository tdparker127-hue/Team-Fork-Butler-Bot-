#include <Arduino.h>
#include "robot_drive.h"
#include "wireless.h"
#include "util.h"
#include "robot_motion_control.h"
#include "imu.h"
#include <bluepad32.h>



void setup() {
    Serial.begin(921600);
    //setupDrive();
    //setupWireless();
    //ps5.begin("BC:C7:46:6D:E7:47");
    Serial.println("Setup complete.");
}

void loop() {
    // Update velocity setpoints based on trajectory at 50Hz
    // EVERY_N_MILLIS(20) {
    //     followTrajectory();
    // }
while(ps5.isConnected()==true){
     if (ps5.LStickX()) {
      Serial.printf("Left Stick x at %d\n", ps5.LStickX());
    }
    if (ps5.LStickY()) {
      Serial.printf("Left Stick y at %d\n", ps5.LStickY());
    }
    if (ps5.RStickX()) {
      Serial.printf("Right Stick x at %d\n", ps5.RStickX());
    }
    if (ps5.RStickY()) {
      Serial.printf("Right Stick y at %d\n", ps5.RStickY());
    }
     if (ps5.L2()) {
      Serial.printf("L2 button at %d\n", ps5.L2Value());
    }
    if (ps5.R2()) {
      Serial.printf("R2 button at %d\n", ps5.R2Value());
    }
    Serial.println();
    delay(300);
}
    // // Update PID at 200Hz
    // EVERY_N_MILLIS(5) {
    //     updatePIDs();
    // }

  //   // Send and print robot values at 20Hz
  //   EVERY_N_MILLIS(50) {
  //       updateOdometry();
  //       sendRobotData();

  //       Serial.printf("x: %.2f, y: %.2f, theta: %.2f\n",
  //                   robotMessage.x, robotMessage.y, robotMessage.theta);
  //   }
  //  EVERY_N_MILLIS(50){
  //   imu.update();
  //   EulerAngles angles = imu.getEulerAngles();
  //   Serial.print("Yaw: ");
  //   Serial.println(angles.yaw);
  //  }
  
}