#include <Arduino.h>
#include <UMS3.h>
UMS3 ums3;

void setup() {
      // Initialize all board peripherals, call this first
  ums3.begin();

  // Brightness is 0-255.
  ums3.setPixelBrightness(255);

    Serial.begin(115200);
    
}
int colorRCV = 100; //received ping color
int colorTX = 255; //transmit pong color
void loop(){
    if(Serial.available() > 0){
        String incoming= Serial.readStringUntil('\n');
        Serial.println("Received: " + incoming);
        delay(150);
        ums3.setPixelColor(UMS3::colorWheel(0));
        if (incoming == "PING"){
            Serial.println("PONG");
            ums3.setPixelColor(UMS3::colorWheel(colorTX));
       }
    }
}