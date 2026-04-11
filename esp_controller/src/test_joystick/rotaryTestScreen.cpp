#include <remote.h>

void setup() {
  ums3.begin();
  initScreen();
  initRotary();

}

void loop() {
  readRotary();
  printScreen();
}