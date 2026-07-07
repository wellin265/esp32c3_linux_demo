#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "led.hpp"

static const char *TAG = "demo";

extern "C" void app_main(void)
{
    led::LedGpio led_gpio(GPIO_NUM_8);
    led_gpio.on();

    while (1) {
        led_gpio.toggle();
        ESP_LOGI(TAG, "LED toggled");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
