#ifndef ARM_PINOUT_H
#define ARM_PINOUT_H

// Lift motor (4-bar linkage) — using first H-bridge channel
#define LIFT_DIR_PIN  39
#define LIFT_PWM_PIN  41

// Gripper motor (end effector open/close) — using second H-bridge channel
#define GRIP_DIR_PIN  40
#define GRIP_PWM_PIN  42

// Encoder for lift motor
#define LIFT_ENC_A_PIN  1
#define LIFT_ENC_B_PIN  2

// Encoder for gripper motor
#define GRIP_ENC_A_PIN  4
#define GRIP_ENC_B_PIN  5

// BNO085 IMU (SPI) — TODO: update pins once arm ESP is wired
#define ARM_BNO08X_CS     12
#define ARM_BNO08X_INT    13
#define ARM_BNO08X_RESET  14

#endif // ARM_PINOUT_H
