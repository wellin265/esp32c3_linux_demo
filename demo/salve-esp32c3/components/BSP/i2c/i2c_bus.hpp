#pragma once

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

namespace driver {

class I2cBus {
public:
    struct Config {
        i2c_port_num_t i2c_port            = I2C_NUM_0;
        gpio_num_t     sda_pin             = GPIO_NUM_5;
        gpio_num_t     scl_pin             = GPIO_NUM_6;
        uint32_t       freq_hz             = 400'000;
        bool           enable_pullup       = true;
        uint8_t        glitch_ignore_cnt   = 7;
        uint32_t       timeout_ms          = 1000;
    };

    explicit I2cBus(const Config& cfg);
    ~I2cBus();

    I2cBus(const I2cBus&) = delete;
    I2cBus& operator=(const I2cBus&) = delete;

    esp_err_t Init();
    esp_err_t AddDevice(uint8_t dev_addr, i2c_master_dev_handle_t* out_handle);
    void      RemoveDevice(i2c_master_dev_handle_t dev_handle);
    bool      Probe(uint8_t dev_addr);
    bool      IsInitialized() const { return is_initialized_; }
    uint32_t  GetTimeoutMs() const { return config_.timeout_ms; }

private:
    Config config_;
    i2c_master_bus_handle_t bus_handle_ = nullptr;
    bool is_initialized_ = false;
};

} // namespace driver
