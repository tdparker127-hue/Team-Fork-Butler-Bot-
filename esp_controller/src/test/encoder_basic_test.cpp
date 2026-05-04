#include <Arduino.h>
#include "robot_pinout.h"
#include "util.h"
#include "EncoderVelocity.h"

#define PRINT_DELAY 50

EncoderVelocity positions[NUM_MOTORS] = {
    {ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_312_RPM, 0.2},
    {ENCODER2_A_PIN, ENCODER2_B_PIN, CPR_312_RPM, 0.2},
    {ENCODER3_A_PIN, ENCODER3_B_PIN, CPR_312_RPM, 0.2},
    {ENCODER4_A_PIN, ENCODER4_B_PIN, CPR_312_RPM, 0.2}
};
void setup(){
    Serial.begin();
}

void loop(){
    EVERY_N_MILLIS(PRINT_DELAY) {
        Serial.printf("POSITION: ");
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            Serial.printf("E%d %.2f   ", i+1, positions[i].getPosition());
        }
        Serial.println();
    }
}