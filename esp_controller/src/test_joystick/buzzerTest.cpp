#include <remote.h>

void setup() {
  ums3.begin();
  // put your setup code here, to run once:
  initPeripherals();

}

void loop() {
  driveBuzzers(255);
  delay(1000);
  driveBuzzers(0);
  delay(1000);
}