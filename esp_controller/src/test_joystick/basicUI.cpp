#include <remote.h>

void setup() {
  initPeripherals();
  initReceiver();
  initScreen();
}

void loop() {
  //printData();
  printScreen();
}


