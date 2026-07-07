#include "mpu9250.hpp"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "Mpu9250";

namespace sensor {

Mpu9250::Mpu9250(driver::I2cBus& i2c, const Config& cfg)
    : i2c_(i2c)
    , config_(cfg)
{
    ESP_LOGD(TAG, "Mpu9250 created (addr=0x%02x)", config_.i2c_addr);
}

Mpu9250::~Mpu9250()
{
    if (is_initialized_) {
        if (is_mag_ready_) {
            i2c_.RemoveDevice(mag_handle_);
            mag_handle_ = nullptr;
        }
        i2c_.RemoveDevice(dev_handle_);
        dev_handle_ = nullptr;
        is_initialized_ = false;
        ESP_LOGD(TAG, "Mpu9250 destroyed");
    }
}

esp_err_t Mpu9250::Init()
{
    if (!i2c_.Probe(config_.i2c_addr)) {
        ESP_LOGE(TAG, "MPU9250 not found at addr 0x%02x", config_.i2c_addr);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_RETURN_ON_ERROR(
        i2c_.AddDevice(config_.i2c_addr, &dev_handle_),
        TAG,
        "AddDevice failed for addr 0x%02x", config_.i2c_addr
    );

    esp_err_t ret = Configure_();
    if (ret != ESP_OK) {
        i2c_.RemoveDevice(dev_handle_);
        dev_handle_ = nullptr;
        return ret;
    }

    if (config_.use_magnetometer) {
        ret = InitMag_();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Magnetometer init failed, continuing without mag");
        }
    }

    is_initialized_ = true;
    ESP_LOGI(TAG, "MPU9250 initialized successfully");
    return ESP_OK;
}

esp_err_t Mpu9250::Configure_()
{
    // H_RESET + CLKSEL, then delay for reset to complete
    ESP_RETURN_ON_ERROR(
        WriteReg_(kRegPwrMgmt1, (1 << kResetBit) | kClkSelPll),
        TAG,
        "H_RESET failed"
    );
    vTaskDelay(pdMS_TO_TICKS(10));

    // Wake device with PLL clock
    ESP_RETURN_ON_ERROR(
        WriteReg_(kRegPwrMgmt1, kClkSelPll),
        TAG,
        "Failed to wake MPU9250"
    );

    // PWR_MGMT_2: enable all axes
    ESP_RETURN_ON_ERROR(
        WriteReg_(kRegPwrMgmt2, 0x00),
        TAG,
        "PWR_MGMT_2 config failed"
    );

    // CONFIG: gyro/temp DLPF
    {
        uint8_t dlpf_cfg = 0;
        if (config_.gyro_dlpf >= 2 && config_.gyro_dlpf <= 9) {
            dlpf_cfg = config_.gyro_dlpf - 2;
        }
        ESP_RETURN_ON_ERROR(
            WriteReg_(kRegConfig, dlpf_cfg),
            TAG,
            "CONFIG failed"
        );
    }

    // GYRO_CONFIG: FS_SEL
    ESP_RETURN_ON_ERROR(
        WriteReg_(kRegGyroCfg, config_.gyro_range << 3),
        TAG,
        "Gyro config failed"
    );

    // ACCEL_CONFIG: FS_SEL
    ESP_RETURN_ON_ERROR(
        WriteReg_(kRegAccelCfg, config_.accel_range << 3),
        TAG,
        "Accel config failed"
    );

    // ACCEL_CONFIG_2: DLPF
    {
        uint8_t fchoice_b = 1;
        uint8_t dlpf_cfg = 0;
        if (config_.accel_dlpf == 0) {
            fchoice_b = 0;
        } else if (config_.accel_dlpf <= 8) {
            dlpf_cfg = config_.accel_dlpf - 1;
        }
        ESP_RETURN_ON_ERROR(
            WriteReg_(kRegAccelCfg2, (fchoice_b << 3) | dlpf_cfg),
            TAG,
            "Accel config 2 failed"
        );
    }

    // INT_ENABLE: RAW_RDY_EN
    ESP_RETURN_ON_ERROR(
        WriteReg_(kRegIntEnable, 0x01),
        TAG,
        "INT_ENABLE failed"
    );

    // Verify WHO_AM_I after reset
    ESP_RETURN_ON_ERROR(WhoAmI_(), TAG, "WhoAmI check failed");
    ESP_LOGI(TAG, "MPU9250 WHO_AM_I = 0x%02X", kWhoAmIValue);

    return ESP_OK;
}

esp_err_t Mpu9250::InitMag_()
{
    // Enable I2C bypass to access AK8963 directly
    ESP_RETURN_ON_ERROR(
        WriteReg_(kRegIntPinCfg, 1 << kBypassEnBit),
        TAG,
        "Bypass enable failed"
    );

    if (!i2c_.Probe(kMagAddr)) {
        ESP_LOGE(TAG, "AK8963 not found at addr 0x%02x", kMagAddr);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_RETURN_ON_ERROR(
        i2c_.AddDevice(kMagAddr, &mag_handle_),
        TAG,
        "AddDevice failed for AK8963"
    );

    // Verify AK8963 WIA
    {
        uint8_t wia = 0;
        ESP_RETURN_ON_ERROR(
            i2c_master_transmit_receive(mag_handle_, &kRegMagWia, 1, &wia, 1,
                                        pdMS_TO_TICKS(config_.timeout_ms)),
            TAG,
            "AK8963 WIA read failed"
        );
        if (wia != kMagWiaValue) {
            ESP_LOGE(TAG, "AK8963 WIA mismatch: expected 0x%02x, got 0x%02x", kMagWiaValue, wia);
            i2c_.RemoveDevice(mag_handle_);
            mag_handle_ = nullptr;
            return ESP_ERR_INVALID_RESPONSE;
        }
        ESP_LOGI(TAG, "AK8963 WIA = 0x%02X", wia);
    }

    // Enter Fuse ROM mode to read calibration coefficients
    {
        uint8_t ctrl[2] = {kRegMagCntl1, kMagCntl1FuseRom};
        ESP_RETURN_ON_ERROR(
            i2c_master_transmit(mag_handle_, ctrl, sizeof(ctrl),
                                pdMS_TO_TICKS(config_.timeout_ms)),
            TAG,
            "AK8963 Fuse ROM mode failed"
        );
    }

    // Read ASA calibration values (registers 0x10, 0x11, 0x12)
    {
        uint8_t asa_buf[3] = {};
        ESP_RETURN_ON_ERROR(
            i2c_master_transmit_receive(mag_handle_, &kRegMagAsa, 1, asa_buf, sizeof(asa_buf),
                                        pdMS_TO_TICKS(config_.timeout_ms)),
            TAG,
            "AK8963 ASA read failed"
        );
        mag_asa_x_ = ((asa_buf[0] - 128) * 0.5f / 128.0f) + 1.0f;
        mag_asa_y_ = ((asa_buf[1] - 128) * 0.5f / 128.0f) + 1.0f;
        mag_asa_z_ = ((asa_buf[2] - 128) * 0.5f / 128.0f) + 1.0f;
        ESP_LOGI(TAG, "AK8963 ASA: X=0x%02X(%.3f) Y=0x%02X(%.3f) Z=0x%02X(%.3f)",
                 asa_buf[0], static_cast<double>(mag_asa_x_),
                 asa_buf[1], static_cast<double>(mag_asa_y_),
                 asa_buf[2], static_cast<double>(mag_asa_z_));
    }

    // Switch to single measurement mode
    {
        uint8_t ctrl[2] = {kRegMagCntl1, kMagCntl1Single};
        ESP_RETURN_ON_ERROR(
            i2c_master_transmit(mag_handle_, ctrl, sizeof(ctrl),
                                pdMS_TO_TICKS(config_.timeout_ms)),
            TAG,
            "AK8963 single measurement mode failed"
        );
    }

    is_mag_ready_ = true;
    ESP_LOGI(TAG, "AK8963 initialized successfully");
    return ESP_OK;
}

esp_err_t Mpu9250::ReadSensor(ImuSensor::SensorData& out)
{
    RawData raw;
    ESP_RETURN_ON_ERROR(ReadRaw(raw), TAG, "ReadRaw failed");
    ConvertToUnits(raw, out);
    return ESP_OK;
}

esp_err_t Mpu9250::ReadRaw(RawData& out)
{
    // Wait for new data (INT_STATUS BIT0 = RAW_RDY). 1kHz internal → 1ms new data.
    uint8_t int_status = 0;
    int retry = 5;
    while (retry-- > 0) {
        esp_err_t err = ReadRegs_(kRegIntStatus, &int_status, 1);
        if (err == ESP_OK && (int_status & 0x01)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint8_t buf[14];
    ESP_RETURN_ON_ERROR(ReadRegs_(kRegAccelX, buf, sizeof(buf)), TAG, "ReadRaw failed");
    out.ax   = (int16_t)((buf[0]  << 8) | buf[1]);
    out.ay   = (int16_t)((buf[2]  << 8) | buf[3]);
    out.az   = (int16_t)((buf[4]  << 8) | buf[5]);
    out.temp = (int16_t)((buf[6]  << 8) | buf[7]);
    out.gx   = (int16_t)((buf[8]  << 8) | buf[9]);
    out.gy   = (int16_t)((buf[10] << 8) | buf[11]);
    out.gz   = (int16_t)((buf[12] << 8) | buf[13]);
    return ESP_OK;
}

void Mpu9250::ConvertToUnits(const RawData& raw, ImuSensor::SensorData& out)
{
    constexpr float kGToMs2    = 9.80665f;
    const float accel_sens = 16384.0f / (1 << config_.accel_range);
    const float gyro_sens  = 131.0f  / (1 << config_.gyro_range);

    out.accel_x = raw.ax / accel_sens * kGToMs2;
    out.accel_y = raw.ay / accel_sens * kGToMs2;
    out.accel_z = raw.az / accel_sens * kGToMs2;
    out.gyro_x  = raw.gx / gyro_sens;
    out.gyro_y  = raw.gy / gyro_sens;
    out.gyro_z  = raw.gz / gyro_sens;
    out.temp    = raw.temp / 333.87f + 21.0f;
}

esp_err_t Mpu9250::ReadMag(ImuSensor::MagData& out)
{
    if (!is_mag_ready_) {
        ESP_LOGE(TAG, "Magnetometer not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Trigger single measurement
    {
        uint8_t ctrl[2] = {kRegMagCntl1, kMagCntl1Single};
        ESP_RETURN_ON_ERROR(
            i2c_master_transmit(mag_handle_, ctrl, sizeof(ctrl),
                                pdMS_TO_TICKS(config_.timeout_ms)),
            TAG,
            "AK8963 trigger failed"
        );
    }

    // Wait for DRDY (BIT0 of ST1). Single measurement takes ~6-8ms.
    {
        uint8_t st1 = 0;
        int retry = 20;
        while (retry-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            esp_err_t err = i2c_master_transmit_receive(
                mag_handle_, &kRegMagSt1, 1, &st1, 1,
                pdMS_TO_TICKS(config_.timeout_ms));
            if (err == ESP_OK && (st1 & 0x01)) {
                break;
            }
        }
        if (!(st1 & 0x01)) {
            ESP_LOGE(TAG, "AK8963 data not ready");
            return ESP_ERR_INVALID_STATE;
        }
    }

    // Read 7 bytes: HXL, HXH, HYL, HYH, HZL, HZH, ST2
    uint8_t buf[7] = {};
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(mag_handle_, &kRegMagHxl, 1, buf, sizeof(buf),
                                    pdMS_TO_TICKS(config_.timeout_ms)),
        TAG,
        "AK8963 data read failed"
    );

    // ST2: check overflow (bit 4) and data skip
    if (buf[6] & 0x08) {
        ESP_LOGW(TAG, "AK8963 magnetic sensor overflow");
        return ESP_ERR_INVALID_STATE;
    }

    int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]);

    out.mx = raw_x * mag_asa_x_;
    out.my = raw_y * mag_asa_y_;
    out.mz = raw_z * mag_asa_z_;

    return ESP_OK;
}

esp_err_t Mpu9250::Reset()
{
    ESP_RETURN_ON_ERROR(
        WriteReg_(kRegPwrMgmt1, 1 << kResetBit),
        TAG,
        "Reset failed"
    );
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "MPU9250 reset");
    return ESP_OK;
}

esp_err_t Mpu9250::WriteReg_(uint8_t reg, uint8_t val)
{
    uint8_t write_buf[2] = {reg, val};
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(dev_handle_, write_buf, sizeof(write_buf), pdMS_TO_TICKS(config_.timeout_ms)),
        TAG,
        "WriteReg_(0x%02x) failed", reg
    );
    return ESP_OK;
}

esp_err_t Mpu9250::ReadRegs_(uint8_t reg, uint8_t* data, size_t len)
{
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(dev_handle_, &reg, 1, data, len, pdMS_TO_TICKS(config_.timeout_ms)),
        TAG,
        "ReadRegs_(0x%02x, len=%zu) failed", reg, len
    );
    return ESP_OK;
}

esp_err_t Mpu9250::WhoAmI_()
{
    uint8_t whoami = 0;
    ESP_RETURN_ON_ERROR(
        ReadRegs_(kRegWhoAmI, &whoami, 1),
        TAG,
        "WHO_AM_I read failed"
    );
    if (whoami != kWhoAmIValue) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: expected 0x%02X, got 0x%02X", kWhoAmIValue, whoami);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

} // namespace sensor
