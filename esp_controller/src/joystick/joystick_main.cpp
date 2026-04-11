#include <remote.h>

void setup() {
  ums3.begin();
  Serial.begin(115200);

  initPeripherals();
  //initRotary();  
  Serial.println("Starting!");
  delay(1000);
  initSender();

}

void loop() {
  readJoysticks();
  readSwitches();
  // //readRotary();
  sendControllerData(); // Sends ControllerMessage via ESP-NOW to robot
  sendData();
  printData();  // Prints data via serial port
  //delay(20);
}
