#include "i2c_bus.hpp"
#include "esp_check.h"
#include "esp_log.h"

static const char* TAG = "I2cBus";

namespace driver {

I2cBus::I2cBus(const Config& cfg)
    : config_(cfg)
{
    ESP_LOGD(TAG, "I2cBus created (port=%d, sda=%d, scl=%d, freq=%" PRIu32 ")",
             static_cast<int>(config_.i2c_port),
             static_cast<int>(config_.sda_pin),
             static_cast<int>(config_.scl_pin),
             config_.freq_hz);
}

I2cBus::~I2cBus()
{
    if (is_initialized_) {
        esp_err_t err = i2c_del_master_bus(bus_handle_);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "i2c_del_master_bus failed: %s", esp_err_to_name(err));
        }
        is_initialized_ = false;
        bus_handle_ = nullptr;
        ESP_LOGD(TAG, "I2cBus destroyed");
    }
}

esp_err_t I2cBus::Init()
{
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port                   = config_.i2c_port;
    bus_cfg.sda_io_num                 = config_.sda_pin;
    bus_cfg.scl_io_num                 = config_.scl_pin;
    bus_cfg.clk_source                 = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt          = config_.glitch_ignore_cnt;
    bus_cfg.flags.enable_internal_pullup = config_.enable_pullup;

    ESP_RETURN_ON_ERROR(
        i2c_new_master_bus(&bus_cfg, &bus_handle_),
        TAG,
        "i2c_new_master_bus failed"
    );

    is_initialized_ = true;
    ESP_LOGI(TAG, "I2C bus initialized");
    return ESP_OK;
}

esp_err_t I2cBus::AddDevice(uint8_t dev_addr, i2c_master_dev_handle_t* out_handle)
{
    if (!is_initialized_) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = dev_addr;
    dev_cfg.scl_speed_hz    = config_.freq_hz;

    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(bus_handle_, &dev_cfg, out_handle),
        TAG,
        "i2c_master_bus_add_device failed (addr=0x%02x)", dev_addr
    );

    ESP_LOGD(TAG, "Device 0x%02x added to bus", dev_addr);
    return ESP_OK;
}

void I2cBus::RemoveDevice(i2c_master_dev_handle_t dev_handle)
{
    if (dev_handle == nullptr) {
        return;
    }

    esp_err_t err = i2c_master_bus_rm_device(dev_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_bus_rm_device failed: %s", esp_err_to_name(err));
    }
}

bool I2cBus::Probe(uint8_t dev_addr)
{
    if (!is_initialized_) {
        return false;
    }
    return i2c_master_probe(bus_handle_, dev_addr, config_.timeout_ms) == ESP_OK;
}

} // namespace driver
