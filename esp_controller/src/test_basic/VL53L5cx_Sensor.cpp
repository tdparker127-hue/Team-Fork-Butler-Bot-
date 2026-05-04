#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include <math.h>

VL53L5CX_ResultsData ToFmeasurement; //measurement data structure for storing ranging data from sensor
SparkFun_VL53L5CX Sensor; //Sensor object 
int distances[16]; //array to hold distance data zone 0-15
int StopDist = 500; //distance in mm at which the bot should stop
void InitializeSensor() {
    //Serial.println("InitializeSensor() called");
Wire.begin();
Wire.setClock(400000);

//Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
    if (Sensor.begin() == false)
        {
            Serial.println(F("Sensor not found - check your wiring. Freezing"));
            while (1) ;
            // Serial.println("Error: ");
            // Serial.println(Sensor.lastError.lastErrorValue);
        }
    else {
        Serial.println("Sensor initialized successfully");}
        /*
        Physical Mapping of zones when indicator square is oriented to top left corner of sensor mount
        [ 3, 2, 1, 0]
        [ 7, 6, 5, 4]
        [11,10, 9, 8]
        [15,14,13,12]
        This means an object in the top left will appear on zone 0. 
        scanning from left to right goes column zero to column 3
        */
        Sensor.setResolution(4*4); //Enable all 16 zones. can do 8x8 but that is 64 zones and takes more time to transfer data
        if (Sensor.startRanging() == false)
        {
            Serial.println("Error: ");
            Serial.println(Sensor.lastError.lastErrorValue);
        }
    }

void ToFData(){
    if (Sensor.isDataReady() == true)
    {
        if (Sensor.getRangingData(&ToFmeasurement)) //Read distance data into array
        {
            //Serial.println("Data is ready");
            int zones[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
            for (int i = 0; i < 16; i++) {
               distances[i] = ToFmeasurement.distance_mm[i]  
               //Serial.print("Zone ");
                //Serial.print(zones[i]);
                //Serial.print(": ");
                //Serial.print(ToFmeasurement.distance_mm[i]);
                //Serial.println(" mm");
            }
          
            }
        }
    // else {
    //     Serial.print("Error: ");
    //     Serial.println(Sensor.lastError.lastErrorValue);
        
    // }
}
void ObstacleAvoidance(int StopDist){
    for (int i = 0; i < 16; i++) {
        if (distances[i] < StopDist) {
            Serial.println("Obstacle detected! Stopping bot.");
            // Code to stop the bot goes here
            break;
        }
    }
}