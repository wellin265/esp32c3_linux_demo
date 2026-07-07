#pragma once

#include "imu_sensor.hpp"
#include "i2c_bus.hpp"
#include "driver/i2c_master.h"
#include "esp_err.h"

namespace sensor {

class Mpu9250 : public ImuSensor {
public:
    struct Config {
        uint8_t  i2c_addr         = 0x68;
        uint8_t  gyro_range       = 0;   // 0=±250dps, 1=±500, 2=±1000, 3=±2000
        uint8_t  accel_range      = 0;   // 0=±2g, 1=±4g, 2=±8g, 3=±16g
        uint8_t  accel_dlpf       = 1;   // 0=bypass (4kHz), 1-8=filter (1→1kHz internal)
        uint8_t  gyro_dlpf        = 3;   // 0-1=bypass, 2-9=DLPF (3→cfg1=1kHz internal)
        uint32_t timeout_ms       = 1000;
        bool     use_magnetometer = true;
    };

    struct RawData {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        int16_t temp;
    };

    explicit Mpu9250(driver::I2cBus& i2c, const Config& cfg);
    ~Mpu9250() override;

    Mpu9250(const Mpu9250&) = delete;
    Mpu9250& operator=(const Mpu9250&) = delete;

    esp_err_t Init() override;
    esp_err_t ReadSensor(ImuSensor::SensorData& out) override;
    esp_err_t ReadMag(ImuSensor::MagData& out) override;
    esp_err_t ReadRaw(RawData& out);
    void      ConvertToUnits(const RawData& raw, ImuSensor::SensorData& out);
    esp_err_t Reset();

private:
    esp_err_t Configure_();
    esp_err_t InitMag_();
    esp_err_t WriteReg_(uint8_t reg, uint8_t val);
    esp_err_t ReadRegs_(uint8_t reg, uint8_t* data, size_t len);
    esp_err_t WhoAmI_();

    driver::I2cBus& i2c_;
    Config config_;
    i2c_master_dev_handle_t dev_handle_ = nullptr;
    i2c_master_dev_handle_t mag_handle_ = nullptr;
    bool is_initialized_ = false;
    bool is_mag_ready_   = false;

    float mag_asa_x_ = 1.0f;
    float mag_asa_y_ = 1.0f;
    float mag_asa_z_ = 1.0f;

    // MPU9250 registers
    static constexpr uint8_t kWhoAmIValue  = 0x71;
    static constexpr uint8_t kRegWhoAmI    = 0x75;
    static constexpr uint8_t kRegPwrMgmt1  = 0x6B;
    static constexpr uint8_t kRegPwrMgmt2  = 0x6C;
    static constexpr uint8_t kRegConfig    = 0x1A;
    static constexpr uint8_t kRegGyroCfg   = 0x1B;
    static constexpr uint8_t kRegAccelCfg  = 0x1C;
    static constexpr uint8_t kRegAccelCfg2 = 0x1D;
    static constexpr uint8_t kRegIntPinCfg = 0x37;
    static constexpr uint8_t kRegIntEnable = 0x38;
    static constexpr uint8_t kRegIntStatus = 0x3A;
    static constexpr uint8_t kRegAccelX    = 0x3B;
    static constexpr uint8_t kResetBit     = 7;
    static constexpr uint8_t kClkSelPll    = 0x01;
    static constexpr uint8_t kBypassEnBit  = 1;

    // AK8963 registers
    static constexpr uint8_t kMagAddr        = 0x0C;
    static constexpr uint8_t kMagWiaValue    = 0x48;
    static constexpr uint8_t kRegMagWia      = 0x00;
    static constexpr uint8_t kRegMagSt1      = 0x02;
    static constexpr uint8_t kRegMagHxl      = 0x03;
    static constexpr uint8_t kRegMagCntl1    = 0x0A;
    static constexpr uint8_t kRegMagAsa      = 0x10;
    static constexpr uint8_t kMagCntl1FuseRom  = 0x0F;
    static constexpr uint8_t kMagCntl1Single   = 0x01;
};

} // namespace sensor
