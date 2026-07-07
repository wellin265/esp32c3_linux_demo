#include "led.hpp"

led::LedBase::LedBase(gpio_num_t pin, bool trigger) : _pin(pin), _trigger(trigger)
{
}

led::LedBase::~LedBase()
{
}

led::LedGpio::~LedGpio()
{
}

led::LedGpio::LedGpio(gpio_num_t pin, bool trigger) : LedBase(pin, trigger)
{
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << _pin),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (trigger == true)
    {
        gpio_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    }
    else
    {
        gpio_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    }
    gpio_config(&gpio_cfg);
    // Set initial level: active-high=low(off), active-low=high(off)
    if (trigger == true)
    {
        gpio_set_level(_pin, 0);
    }
    else
    {
        gpio_set_level(_pin, 1);
    }
}

void led::LedGpio::on()
{   
    if (_trigger == true)
    {
        gpio_set_level(_pin, 1);
    }
    else
    {
        gpio_set_level(_pin, 0);
    }
}

void led::LedGpio::off()
{
    if (_trigger == true)
    {
        gpio_set_level(_pin, 0);
    }
    else
    {
        gpio_set_level(_pin, 1);
    }
}

void led::LedGpio::toggle()
{
    gpio_set_level(_pin, !gpio_get_level(_pin));
}

gpio_num_t led::LedGpio::get_pin(void) const
{
    return _pin;
}

