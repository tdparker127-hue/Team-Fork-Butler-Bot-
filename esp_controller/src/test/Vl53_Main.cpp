#include <Arduino.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "util.h"
#include "VL53L5cx_Sensor.h"

void setup() {
    Serial.begin(115200);
    //Serial.println("Starting VL53L5CX main"); //Debug print 
    delay(10000);
    InitializeSensor();
}

void loop() {
    ToFData();
    delay(5);
}