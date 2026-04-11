#include <remote.h>

void setup() {
  ums3.begin();

  initPeripherals();
  delay(3000);
  
  Serial.println("Starting!");
  delay(1000);
  initReceiver();

}

void loop() {
  //readJoysticks();
  //readSwitches();
  
  printData();
  delay(200);
}
