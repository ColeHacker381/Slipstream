#pragma once

#include <cstddef>
#include <esp_err.h>

namespace audio {

/* Historical H1/H2 entry points. */
esp_err_t init_output();
esp_err_t init_input();

esp_err_t init_full_duplex();

esp_err_t read(
    void *samples,
    size_t bytes,
    size_t *bytes_read
);

esp_err_t write(
    const void *samples,
    size_t bytes
);

}
