#pragma once

#include <esp_err.h>

namespace wm8960 {

esp_err_t init_i2c();
bool probe();

esp_err_t configure_headphone_playback();
esp_err_t configure_microphone_capture();
esp_err_t configure_full_duplex();

}