#pragma once

#include "esp_err.h"

namespace sensor {

class ImuSensor {
public:
    struct SensorData {
        float accel_x, accel_y, accel_z;  // m/s²
        float gyro_x,  gyro_y,  gyro_z;   // °/s
        float temp;                       // °C
    };

    struct MagData {
        float mx, my, mz;  // µT
    };

    virtual ~ImuSensor() = default;

    virtual esp_err_t Init() = 0;
    virtual esp_err_t ReadSensor(SensorData& out) = 0;
    virtual esp_err_t ReadMag(MagData& out) = 0;
};

} // namespace sensor
