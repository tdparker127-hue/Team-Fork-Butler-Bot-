#include <remote.h>

void setup() {
  ums3.begin();

  initPeripherals();
}

void loop() {
  // put your main code here, to run repeatedly:
  //myFunction();
 readBattery();
 Serial.print("Battery: ");
 Serial.println(batteryVoltage);
 delay(500);
}