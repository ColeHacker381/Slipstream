#include "h0_i2c.hpp"

#include "codec/wm8960.hpp"
#include "esp_log.h"

namespace {

constexpr const char *TAG = "H0";

}

namespace h0 {

void run()
{
    ESP_LOGI(TAG, "Starting H0 I2C test");

    if (wm8960::init_i2c() != ESP_OK) {
        ESP_LOGE(TAG, "H0 FAIL: I2C initialization failed");
        return;
    }

    if (!wm8960::probe()) {
        ESP_LOGE(TAG, "H0 FAIL: WM8960 not detected");
        return;
    }

    ESP_LOGI(TAG, "H0 PASS: WM8960 detected at 0x1A");
}

}