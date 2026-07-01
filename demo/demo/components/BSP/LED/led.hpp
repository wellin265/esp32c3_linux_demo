#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

namespace led {
    
class LedBase {
public:
    LedBase(gpio_num_t pin = GPIO_NUM_8, bool trigger = false);
    virtual ~LedBase();
    virtual void on();
    virtual void off();
    virtual void toggle();
    virtual gpio_num_t get_pin(void) const;
protected:
    gpio_num_t _pin;
    bool _trigger;
};

class LedGpio : public LedBase {
public:
    LedGpio(gpio_num_t pin = GPIO_NUM_8, bool trigger = false);
    ~LedGpio() override;
    void on() override;
    void off() override;
    void toggle() override;
    gpio_num_t get_pin(void) const override;
};

} // namespace led