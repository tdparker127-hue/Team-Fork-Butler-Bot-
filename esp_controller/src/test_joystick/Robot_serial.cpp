#include <Arduino.h>


float joystickLx =0.0;
float joystickLy =0.0;
float joystickRx =0.0;
float joystickRy =0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
    if (Serial.available() >0) {
        String input = Serial.readStringUntil('\n');

        int matches = sscanf(input.c_str(), "%f %f %f %f", &joystickLx, &joystickLy, &joystickRx, &joystickRy);
        if (matches == 4) {
            ;
    }
}
}
