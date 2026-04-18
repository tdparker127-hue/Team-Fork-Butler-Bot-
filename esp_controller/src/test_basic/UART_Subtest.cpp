#include <Arduino.h>

String incoming;

void setup() {
    Serial.begin(115200);
    Serial.println("Suboordinate comm Test Starting...");
   // SubSerial.begin(115200, SERIAL_8N1, 17, 18); // RX, TX pins
}

void loop() {
    Serial.println("Checking for SubSerial communication...");
    // if (SubSerial.available() > 0) {
    //    SubSerial.write("Ping");
    //     incoming = SubSerial.read();
    //     delay(150);
    //     if (incoming == "PONG") {
    //         SubSerial.println("PING");
    //         Serial.println("Sent PING to SubSerial");
    //     }
    //     else {
    //         Serial.println("Sub not communicating.");
    //     }

    }
