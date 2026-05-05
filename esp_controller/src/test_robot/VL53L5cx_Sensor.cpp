#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <math.h>
#include "VL53L5cx_Sensor.h"

VL53L5CX_ResultsData ToFmeasurement;
SparkFun_VL53L5CX Sensor;
static int distances[16];

ObstacleReading obstacleReading = {0.0f, 0.0f, false};

// Center angle (radians) for each column, col 0 = rightmost (+), col 3 = leftmost (-)
// Total FOV = 45 deg, 4 columns, 11.25 deg per column
static const float COL_ANGLE_RAD[4] = {
     16.875f * (float)M_PI / 180.0f,
      5.625f * (float)M_PI / 180.0f,
     -5.625f * (float)M_PI / 180.0f,
    -16.875f * (float)M_PI / 180.0f
};

void InitializeSensor() {
    Wire.begin();
    Wire.setClock(400000);

    if (!Sensor.begin()) {
        Serial.println(F("Sensor not found - check your wiring. Freezing"));
        while (1);
    }
    Serial.println("Sensor initialized successfully");

    /*
    Physical zone mapping (indicator square at top-left corner):
    [ 3, 2, 1, 0]  row 0  (top)
    [ 7, 6, 5, 4]  row 1
    [11,10, 9, 8]  row 2
    [15,14,13,12]  row 3  (bottom)
    Zone index = row * 4 + (3 - col)
    Col 0 = rightmost, col 3 = leftmost
    */
    Sensor.setResolution(4 * 4);
    if (!Sensor.startRanging()) {
        Serial.print("Ranging start error: ");
        Serial.println(Sensor.lastError.lastErrorValue);
    }
}

void ComputeObstacleReading() {
    int   best_dist_mm = INT_MAX;
    float best_angle   = 0.0f;

    for (int col = 0; col < 4; col++) {
        int col_min = INT_MAX;
        for (int row = 0; row < 4; row++) {
            int zone = row * 4 + (3 - col);
            if (distances[zone] > 0 && distances[zone] < col_min)
                col_min = distances[zone];
        }
        if (col_min < best_dist_mm) {
            best_dist_mm = col_min;
            best_angle   = COL_ANGLE_RAD[col];
        }
    }

    obstacleReading.distance_m = best_dist_mm / 1000.0f;
    obstacleReading.angle_rad  = best_angle;
    obstacleReading.valid      = (best_dist_mm > 0) &&
                                 (best_dist_mm < TOF_REP_RADIUS_MM);
}

void ToFData() {
    if (!Sensor.isDataReady()) return;
    if (!Sensor.getRangingData(&ToFmeasurement)) return;

    for (int i = 0; i < 16; i++)
        distances[i] = ToFmeasurement.distance_mm[i];

    ComputeObstacleReading();
}
