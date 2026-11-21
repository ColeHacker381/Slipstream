
#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

namespace phone_audio {

constexpr uint32_t SAMPLE_RATE = 48000;

constexpr size_t FRAMES_PER_BLOCK = 480;
constexpr size_t CHANNELS = 2;

constexpr size_t SAMPLES_PER_BLOCK =
    FRAMES_PER_BLOCK * CHANNELS;

struct Stats {
    uint32_t received_blocks = 0;
    uint32_t consumed_blocks = 0;
    uint32_t dropped_blocks = 0;
    uint32_t missing_blocks = 0;
    uint32_t read_errors = 0;
    uint32_t peak = 0;
};

esp_err_t initialize();

// Always returns a complete 10 ms block.
// Returns false and provides silence if no
// phone audio is currently available.
bool read_block(int16_t *output);

void read_and_reset_stats(Stats &stats);

} // namespace phone_audio
