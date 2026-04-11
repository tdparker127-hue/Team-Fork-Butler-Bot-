#include <Arduino.h>
#include <Wire.h>
#include <vl53l8cx.h>

#define LPN_PIN -1 // Set to a GPIO pin if using hardware power control
VL53L8CX sensor_vl53l8cx(&Wire, LPN_PIN);

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9); // Explicit ESP32 SDA/SCL pins

  // 1. Hardware Power/Wire setup
  sensor_vl53l8cx.begin();
  
  // 2. Load Firmware (CRITICAL - takes ~1-2 seconds)
  if (sensor_vl53l8cx.init() != 0) {
    Serial.println("VL53L8CX Firmware Load Failed!");
    while(1);
  }

  sensor_vl53l8cx.set_resolution(VL53L8CX_RESOLUTION_4X4);
  sensor_vl53l8cx.start_ranging();
  Serial.println("VL53L8CX Ready.");
}

void loop() {
  VL53L8CX_ResultsData results;
  uint8_t newDataReady = 0;

  // Poll for data (per Hello World example)
  do {
    sensor_vl53l8cx.check_data_ready(&newDataReady);
  } while (!newDataReady);

  if (sensor_vl53l8cx.get_ranging_data(&results) == 0) {
    // Zone 7 is roughly the center of a 4x4 grid
    int zone = 7;
    // Calculate index for multiple targets per zone (default is 1)
    int idx = VL53L8CX_NB_TARGET_PER_ZONE * zone; 
    
    Serial.print("Zone 7 Distance: ");
    Serial.print(results.distance_mm[idx]);
    Serial.println(" mm");
  }
  delay(10);
}
