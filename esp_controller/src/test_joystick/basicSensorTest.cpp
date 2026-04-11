#include <remote.h>

void setup() {
  ums3.begin();

  initPeripherals();
  //initReceiver();
  //initScreen();
}

void loop() {
  readJoysticks();
  readSwitches();
  printData();
  //printScreen();
}


