#pragma once

#include <cstddef>
#include <cstdint>

namespace dsp {

void process_voice_filters(
    const int16_t *input,
    int16_t *output,
    size_t samples
);

void reset_voice_filters();

}