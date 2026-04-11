#include <Arduino.h>
#include <UMS3.h>
#include <WiFi.h>
#include <esp_now.h>
UMS3 ums3;


void setup(){
  ums3.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("ESP Board BASE MAC Address:  ");
  Serial.println(WiFi.macAddress());
  delay(5000);
}
