#include "i2c_bus.hpp"
#include "mpu9250.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "mpu9250";

extern "C" void app_main(void)
{
    driver::I2cBus::Config i2c_cfg = {
        .sda_pin = GPIO_NUM_5,
        .scl_pin = GPIO_NUM_6,
    };

    driver::I2cBus i2c(i2c_cfg);
    ESP_ERROR_CHECK(i2c.Init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    sensor::Mpu9250::Config imu_cfg = { .i2c_addr = 0x68 };
    sensor::Mpu9250 imu(i2c, imu_cfg);
    sensor::ImuSensor& imu_ref = imu;
    ESP_ERROR_CHECK(imu_ref.Init());
    ESP_LOGI(TAG, "MPU9250 initialized successfully");

    sensor::ImuSensor::SensorData data;
    sensor::ImuSensor::MagData mag;

    while (1)
    {
        esp_err_t err = imu_ref.ReadSensor(data);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read sensor: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        err = imu_ref.ReadMag(mag);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read magnetometer: %s", esp_err_to_name(err));
            mag = {};
        }

        ESP_LOGI(TAG, "Accel: X=%.3f Y=%.3f Z=%.3f m/s^2 | "
                       "Gyro: X=%.3f Y=%.3f Z=%.3f deg/s | "
                       "Mag: X=%.3f Y=%.3f Z=%.3f uT | "
                       "Temp=%.2f C",
                 data.accel_x, data.accel_y, data.accel_z,
                 data.gyro_x, data.gyro_y, data.gyro_z,
                 mag.mx, mag.my, mag.mz,
                 data.temp);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
