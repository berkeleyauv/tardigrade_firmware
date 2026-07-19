#include <Arduino.h>
#include "estimator/IStateEstimator.h"
#include "drivers/IImuSource.h"

using namespace tardigrade;

void setup() {
    Serial.begin(115200);
    VehicleState s;              // default-constructed, all zeros
    Serial.println("Tardigrade FC starting...");
    Serial.printf("state valid=%d yaw=%.2f\n", s.valid, s.yaw_rad);
}

void loop() {}