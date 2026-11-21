
#include "phone_audio.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "driver/i2s_std.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char TAG[] = "phone_audio";

// ============================================================
// Configuration
// ============================================================

constexpr gpio_num_t PIN_BCLK = GPIO_NUM_26;
constexpr gpio_num_t PIN_WS   = GPIO_NUM_27;
constexpr gpio_num_t PIN_DIN  = GPIO_NUM_34;

constexpr uint32_t SAMPLE_RATE =
    phone_audio::SAMPLE_RATE;

constexpr size_t BLOCK_FRAMES =
    phone_audio::FRAMES_PER_BLOCK;

constexpr size_t BLOCK_SAMPLES =
    phone_audio::SAMPLES_PER_BLOCK;

constexpr size_t BLOCK_BYTES =
    BLOCK_SAMPLES * sizeof(int16_t);

// ============================================================
// PCM ring configuration
// ============================================================

constexpr uint32_t RING_FRAMES = 4096;

constexpr uint32_t RING_MASK =
    RING_FRAMES - 1;

constexpr uint32_t TARGET_FRAMES = 1920;

constexpr uint32_t PREFILL_FRAMES = 1920;

// Thresholds for faster recovery from
// unusually high or low occupancy.

constexpr uint32_t LOW_WATERMARK = 960;

constexpr uint32_t HIGH_WATERMARK = 3072;

// ============================================================
// Resampler configuration
// ============================================================

constexpr uint32_t Q16_ONE = 65536;

constexpr int32_t NORMAL_LIMIT_PPM = 1000;

constexpr int32_t RECOVERY_LIMIT_PPM = 5000;

// Proportional controller:
// occupancy error in frames * Kp = ppm.

constexpr float KP = 0.50f;

// Integral correction per 10 ms block.
// Allows persistent clock offset to be
// compensated without permanent error.

constexpr float KI = 0.001f;

// Low-pass filter for occupancy measurements.
// Avoids reacting aggressively to DMA bursts.

constexpr float OCCUPANCY_ALPHA = 0.02f;

static_assert(
    (RING_FRAMES & RING_MASK) == 0
);

static_assert(
    TARGET_FRAMES < RING_FRAMES
);

static_assert(
    BLOCK_SAMPLES == BLOCK_FRAMES * 2
);

// ============================================================
// Data types
// ============================================================

struct Stereo {
    int16_t left;
    int16_t right;
};

static_assert(sizeof(Stereo) == 4);

// ============================================================
// Shared storage
// ============================================================
//
// Single producer:
//     I2S receiver task
//
// Single consumer:
//     audio::write / read_block
//
// Only the producer modifies write_index.
// Only the consumer modifies read_index.
//

alignas(4) Stereo pcm_ring[RING_FRAMES] = {};

alignas(4) Stereo receive_buffer[
    BLOCK_FRAMES
] = {};

std::atomic<uint32_t> write_index{0};
std::atomic<uint32_t> read_index{0};

// Whenever the receiver loses source data,
// the consumer should rebuffer.

std::atomic<bool> reset_requested{false};

i2s_chan_handle_t rx_channel = nullptr;

// ============================================================
// Consumer state
// ============================================================
//
// These variables are owned exclusively
// by the audio consumer task.
//

bool playback_started = false;
bool need_fade_in = true;

uint32_t phase_q16 = 0;

float filtered_occupancy =
    static_cast<float>(TARGET_FRAMES);

float integral_ppm = 0.0f;

int16_t last_left = 0;
int16_t last_right = 0;

// ============================================================
// Existing public statistics
// ============================================================

std::atomic<uint32_t> received_blocks{0};
std::atomic<uint32_t> consumed_blocks{0};
std::atomic<uint32_t> dropped_blocks{0};
std::atomic<uint32_t> missing_blocks{0};
std::atomic<uint32_t> read_errors{0};
std::atomic<uint32_t> peak{0};

// ============================================================
// Additional synchronization statistics
// ============================================================

std::atomic<uint32_t> received_frames{0};
std::atomic<uint32_t> source_frames_used{0};
std::atomic<uint32_t> output_frames_generated{0};

std::atomic<uint32_t> overflow_events{0};
std::atomic<uint32_t> underflow_events{0};

std::atomic<uint32_t> prebuffer_events{0};
std::atomic<uint32_t> concealed_blocks{0};
std::atomic<uint32_t> resync_events{0};

std::atomic<uint32_t> occupancy_min{
    RING_FRAMES
};

std::atomic<uint32_t> occupancy_max{0};

std::atomic<int32_t> correction_ppm{0};

std::atomic<uint32_t> filtered_frames{
    TARGET_FRAMES
};

// ============================================================
// Atomic statistic helpers
// ============================================================

void update_max(
    std::atomic<uint32_t> &target,
    uint32_t value
) {
    uint32_t previous =
        target.load(
            std::memory_order_relaxed
        );

    while (
        value > previous &&
        !target.compare_exchange_weak(
            previous,
            value,
            std::memory_order_relaxed
        )
    ) {
    }
}

void update_min(
    std::atomic<uint32_t> &target,
    uint32_t value
) {
    uint32_t previous =
        target.load(
            std::memory_order_relaxed
        );

    while (
        value < previous &&
        !target.compare_exchange_weak(
            previous,
            value,
            std::memory_order_relaxed
        )
    ) {
    }
}

// ============================================================
// Ring occupancy
// ============================================================

uint32_t available_frames()
{
    const uint32_t read =
        read_index.load(
            std::memory_order_acquire
        );

    const uint32_t write =
        write_index.load(
            std::memory_order_acquire
        );

    const uint32_t available =
        write - read;

    // Telemetry snapshots may be taken while
    // another task advances one of the indices.

    return std::min<uint32_t>(
        available,
        RING_FRAMES
    );
}

void record_occupancy(uint32_t available)
{
    update_min(
        occupancy_min,
        available
    );

    update_max(
        occupancy_max,
        available
    );
}

// ============================================================
// Ring producer
// ============================================================

bool enqueue_frames(
    const Stereo *input,
    uint32_t frames
) {
    if (frames == 0) {
        return true;
    }

    const uint32_t write =
        write_index.load(
            std::memory_order_relaxed
        );

    const uint32_t read =
        read_index.load(
            std::memory_order_acquire
        );

    const uint32_t occupied =
        write - read;

    if (
        occupied > RING_FRAMES ||
        frames > RING_FRAMES - occupied
    ) {
        overflow_events.fetch_add(1);

        dropped_blocks.fetch_add(1);

        // Do not move read_index here.
        // Only the consumer owns read_index.
        //
        // The new incoming block is discarded.

        reset_requested.store(
            true,
            std::memory_order_release
        );

        return false;
    }

    const uint32_t position =
        write & RING_MASK;

    const uint32_t first_count =
        std::min<uint32_t>(
            frames,
            RING_FRAMES - position
        );

    std::memcpy(
        &pcm_ring[position],
        input,
        first_count * sizeof(Stereo)
    );

    const uint32_t remaining =
        frames - first_count;

    if (remaining > 0) {

        std::memcpy(
            &pcm_ring[0],
            input + first_count,
            remaining * sizeof(Stereo)
        );
    }

    // Publish only after the entire incoming
    // PCM block has been copied.

    write_index.store(
        write + frames,
        std::memory_order_release
    );

    record_occupancy(
        occupied + frames
    );

    return true;
}

// ============================================================
// Input peak measurement
// ============================================================

void record_peak(
    const Stereo *input,
    size_t frames
) {
    uint32_t block_peak = 0;

    for (size_t i = 0;
         i < frames;
         ++i) {

        const int32_t left =
            input[i].left;

        const int32_t right =
            input[i].right;

        const uint32_t left_abs =
            static_cast<uint32_t>(
                left < 0 ? -left : left
            );

        const uint32_t right_abs =
            static_cast<uint32_t>(
                right < 0 ? -right : right
            );

        block_peak =
            std::max<uint32_t>(
                block_peak,
                left_abs
            );

        block_peak =
            std::max<uint32_t>(
                block_peak,
                right_abs
            );
    }

    update_max(
        peak,
        block_peak
    );
}

// ============================================================
// Clock synchronization controller
// ============================================================

void reset_controller()
{
    phase_q16 = 0;

    filtered_occupancy =
        static_cast<float>(
            TARGET_FRAMES
        );

    integral_ppm = 0.0f;

    correction_ppm.store(
        0,
        std::memory_order_relaxed
    );

    filtered_frames.store(
        TARGET_FRAMES,
        std::memory_order_relaxed
    );
}

uint32_t calculate_source_step(
    uint32_t occupancy
) {
    // Smooth block-to-block occupancy changes.

    filtered_occupancy +=
        OCCUPANCY_ALPHA *
        (
            static_cast<float>(occupancy) -
            filtered_occupancy
        );

    const float error =
        filtered_occupancy -
        static_cast<float>(
            TARGET_FRAMES
        );

    // Positive error:
    // consume source PCM more quickly.
    //
    // Negative error:
    // consume source PCM more slowly.

    integral_ppm += KI * error;

    integral_ppm =
        std::clamp<float>(
            integral_ppm,

            -static_cast<float>(
                NORMAL_LIMIT_PPM
            ),

            static_cast<float>(
                NORMAL_LIMIT_PPM
            )
        );

    float requested_ppm =
        KP * error +
        integral_ppm;

    requested_ppm =
        std::clamp<float>(
            requested_ppm,

            -static_cast<float>(
                NORMAL_LIMIT_PPM
            ),

            static_cast<float>(
                NORMAL_LIMIT_PPM
            )
        );

    // Emergency recovery.
    //
    // These corrections are temporary and
    // intended to prevent a much more audible
    // entire-block drop or underflow.

    if (occupancy < LOW_WATERMARK) {

        const float deficit =
            static_cast<float>(
                LOW_WATERMARK -
                occupancy
            );

        requested_ppm -=
            deficit * 4.0f;
    }

    if (occupancy > HIGH_WATERMARK) {

        const float excess =
            static_cast<float>(
                occupancy -
                HIGH_WATERMARK
            );

        requested_ppm +=
            excess * 4.0f;
    }

    requested_ppm =
        std::clamp<float>(
            requested_ppm,

            -static_cast<float>(
                RECOVERY_LIMIT_PPM
            ),

            static_cast<float>(
                RECOVERY_LIMIT_PPM
            )
        );

    const int32_t ppm =
        static_cast<int32_t>(
            requested_ppm
        );

    correction_ppm.store(
        ppm,
        std::memory_order_relaxed
    );

    filtered_frames.store(
        static_cast<uint32_t>(
            filtered_occupancy
        ),
        std::memory_order_relaxed
    );

    // Nominally consume one input frame
    // for every output frame.
    //
    // step = 65536 * (1 + ppm / 1000000)

    int64_t adjustment =
        static_cast<int64_t>(ppm) *
        static_cast<int64_t>(Q16_ONE);

    // Round to nearest Q16 step.

    if (adjustment >= 0) {
        adjustment += 500000;
    } else {
        adjustment -= 500000;
    }

    adjustment /= 1000000;

    const int32_t step =
        static_cast<int32_t>(Q16_ONE) +
        static_cast<int32_t>(adjustment);

    return static_cast<uint32_t>(
        step
    );
}

// ============================================================
// Q16 PCM interpolation
// ============================================================

inline int16_t interpolate_sample(
    int16_t first,
    int16_t second,
    uint32_t fraction_q16
) {
    const int32_t a =
        static_cast<int32_t>(
            first
        );

    const int32_t b =
        static_cast<int32_t>(
            second
        );

    const int32_t difference =
        b - a;

    const int64_t product =
        static_cast<int64_t>(
            difference
        ) *
        static_cast<int64_t>(
            fraction_q16
        );

    const int32_t result =
        a +
        static_cast<int32_t>(
            product >> 16
        );

    return static_cast<int16_t>(
        std::clamp<int32_t>(
            result,
            -32768,
            32767
        )
    );
}

// ============================================================
// Fade concealment
// ============================================================

void conceal_block(int16_t *output)
{
    constexpr int32_t DENOMINATOR =
        static_cast<int32_t>(
            BLOCK_FRAMES - 1
        );

    if (last_left == 0 &&
        last_right == 0) {

        std::memset(
            output,
            0,
            BLOCK_BYTES
        );

    } else {

        for (
            size_t i = 0;
            i < BLOCK_FRAMES;
            ++i
        ) {
            const int32_t gain =
                DENOMINATOR -
                static_cast<int32_t>(i);

            output[i * 2] =
                static_cast<int16_t>(
                    static_cast<int32_t>(
                        last_left
                    ) * gain /
                    DENOMINATOR
                );

            output[i * 2 + 1] =
                static_cast<int16_t>(
                    static_cast<int32_t>(
                        last_right
                    ) * gain /
                    DENOMINATOR
                );
        }
    }

    last_left = 0;
    last_right = 0;

    concealed_blocks.fetch_add(1);

    output_frames_generated.fetch_add(
        BLOCK_FRAMES
    );
}

void fade_in(int16_t *output)
{
    constexpr int32_t DENOMINATOR =
        static_cast<int32_t>(
            BLOCK_FRAMES - 1
        );

    for (
        size_t i = 0;
        i < BLOCK_FRAMES;
        ++i
    ) {
        const int32_t gain =
            static_cast<int32_t>(i);

        output[i * 2] =
            static_cast<int16_t>(
                static_cast<int32_t>(
                    output[i * 2]
                ) * gain /
                DENOMINATOR
            );

        output[i * 2 + 1] =
            static_cast<int16_t>(
                static_cast<int32_t>(
                    output[i * 2 + 1]
                ) * gain /
                DENOMINATOR
            );
    }
}

// ============================================================
// Consumer-controlled resynchronization
// ============================================================

void resynchronize()
{
    const uint32_t write =
        write_index.load(
            std::memory_order_acquire
        );

    // Discard stale buffered audio.
    // Only the consumer advances the read index.

    read_index.store(
        write,
        std::memory_order_release
    );

    playback_started = false;
    need_fade_in = true;

    reset_controller();

    resync_events.fetch_add(1);
}

// ============================================================
// Adaptive PCM consumption
// ============================================================

bool render_audio_block(int16_t *output)
{
    const uint32_t read =
        read_index.load(
            std::memory_order_relaxed
        );

    const uint32_t write =
        write_index.load(
            std::memory_order_acquire
        );

    const uint32_t available =
        write - read;

    if (available > RING_FRAMES) {
        resynchronize();
        return false;
    }

    record_occupancy(
        available
    );

    const uint32_t step_q16 =
        calculate_source_step(
            available
        );

    // Determine the highest input position
    // accessed by this interpolation block,
    // plus one sample of interpolation lookahead.

    const uint64_t final_position =
        static_cast<uint64_t>(
            phase_q16
        ) +
        static_cast<uint64_t>(
            BLOCK_FRAMES - 1
        ) *
        static_cast<uint64_t>(
            step_q16
        );

    const uint32_t required =
        (step_q16 == Q16_ONE && phase_q16 == 0)
            ? static_cast<uint32_t>(BLOCK_FRAMES)
            : static_cast<uint32_t>(
                final_position >> 16
            ) + 2;

    if (available < required) {
        return false;
    }

    uint32_t cursor = read;

    uint32_t phase =
        phase_q16;

    // ========================================================
    // Fast path: nominal sample rate and exact integer phase.
    // ========================================================

    if (
        step_q16 == Q16_ONE &&
        phase == 0
    ) {

        const uint32_t position =
            cursor & RING_MASK;

        const uint32_t first_count =
            std::min<uint32_t>(
                static_cast<uint32_t>(
                    BLOCK_FRAMES
                ),

                RING_FRAMES - position
            );

        std::memcpy(
            output,

            &pcm_ring[position],

            first_count *
            sizeof(Stereo)
        );

        const uint32_t remaining =
            BLOCK_FRAMES -
            first_count;

        if (remaining > 0) {

            std::memcpy(
                output + first_count * 2,

                &pcm_ring[0],

                remaining *
                sizeof(Stereo)
            );
        }

        cursor += BLOCK_FRAMES;

    } else {

        // ====================================================
        // Fractional-rate stereo consumption.
        // ====================================================

        for (
            size_t i = 0;
            i < BLOCK_FRAMES;
            ++i
        ) {

            const Stereo &first =
                pcm_ring[
                    cursor & RING_MASK
                ];

            const Stereo &second =
                pcm_ring[
                    (cursor + 1) &
                    RING_MASK
                ];

            output[i * 2] =
                interpolate_sample(
                    first.left,
                    second.left,
                    phase
                );

            output[i * 2 + 1] =
                interpolate_sample(
                    first.right,
                    second.right,
                    phase
                );

            phase += step_q16;

            cursor += phase >> 16;

            phase &= Q16_ONE - 1;
        }
    }

    phase_q16 = phase;

    const uint32_t consumed =
        cursor - read;

    // Release the source data after rendering
    // the complete output block.

    read_index.store(
        cursor,
        std::memory_order_release
    );

    source_frames_used.fetch_add(
        consumed
    );

    output_frames_generated.fetch_add(
        BLOCK_FRAMES
    );

    consumed_blocks.fetch_add(1);

    record_occupancy(
        available - consumed
    );

    return true;
}

// ============================================================
// I2S slave receiver
// ============================================================

void receiver_task(void *)
{
    size_t filled = 0;

    ESP_LOGI(
        TAG,
        "Adaptive PCM receiver on CPU%d",
        xPortGetCoreID()
    );

    while (true) {

        auto *destination =
            reinterpret_cast<uint8_t *>(
                receive_buffer
            );

        size_t bytes_read = 0;

        const esp_err_t result =
            i2s_channel_read(
                rx_channel,

                destination + filled,

                BLOCK_BYTES - filled,

                &bytes_read,

                30
            );

        if (
            bytes_read >
            BLOCK_BYTES - filled
        ) {

            read_errors.fetch_add(1);

            reset_requested.store(
                true,
                std::memory_order_release
            );

            filled = 0;

            continue;
        }

        filled += bytes_read;

        if (result != ESP_OK) {

            read_errors.fetch_add(1);

            if (result != ESP_ERR_TIMEOUT ||
                bytes_read == 0) {

                filled = 0;

                reset_requested.store(
                    true,
                    std::memory_order_release
                );

                vTaskDelay(1);

                continue;
            }
        }

        if (filled != BLOCK_BYTES) {
            continue;
        }

        filled = 0;

        received_blocks.fetch_add(1);

        received_frames.fetch_add(
            BLOCK_FRAMES
        );

        record_peak(
            receive_buffer,
            BLOCK_FRAMES
        );

        enqueue_frames(
            receive_buffer,
            BLOCK_FRAMES
        );
    }
}

} // anonymous namespace

// ============================================================
// Public interface
// ============================================================

namespace phone_audio {

esp_err_t initialize()
{
    if (rx_channel != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    // ========================================================
    // Validated inter-board I2S configuration.
    //
    // Bluetooth ESP32: master TX
    // DSP ESP32: slave RX
    // ========================================================

    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(
            I2S_NUM_1,
            I2S_ROLE_SLAVE
        );

    channel_config.dma_desc_num = 8;

    channel_config.dma_frame_num = 240;

    ESP_RETURN_ON_ERROR(
        i2s_new_channel(
            &channel_config,
            nullptr,
            &rx_channel
        ),
        TAG,
        "I2S RX allocation failed"
    );

    i2s_std_config_t config = {};

    config.clk_cfg =
        I2S_STD_CLK_DEFAULT_CONFIG(
            SAMPLE_RATE
        );

    config.slot_cfg =
        I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO
        );

    config.gpio_cfg.mclk =
        I2S_GPIO_UNUSED;

    config.gpio_cfg.bclk =
        PIN_BCLK;

    config.gpio_cfg.ws =
        PIN_WS;

    config.gpio_cfg.din =
        PIN_DIN;

    config.gpio_cfg.dout =
        I2S_GPIO_UNUSED;

    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(
            rx_channel,
            &config
        ),
        TAG,
        "I2S RX standard mode failed"
    );

    ESP_RETURN_ON_ERROR(
        i2s_channel_enable(
            rx_channel
        ),
        TAG,
        "I2S RX enable failed"
    );

    reset_controller();

    if (
        xTaskCreatePinnedToCore(
            receiver_task,

            "phone_audio_rx",

            4096,

            nullptr,

            5,

            nullptr,

            1
        ) != pdPASS
    ) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(
        TAG,
        "D1.8.3d: Adaptive PCM clock sync"
    );

    ESP_LOGI(
        TAG,
        "SLAVE RX: BCLK=26 WS=27 DIN=34"
    );

    ESP_LOGI(
        TAG,
        "PCM16 stereo / 48 kHz"
    );

    ESP_LOGI(
        TAG,
        "Ring=%lu frames / %lu bytes",

        static_cast<unsigned long>(
            RING_FRAMES
        ),

        static_cast<unsigned long>(
            sizeof(pcm_ring)
        )
    );

    ESP_LOGI(
        TAG,
        "Target=%lu frames / %lu ms",

        static_cast<unsigned long>(
            TARGET_FRAMES
        ),

        static_cast<unsigned long>(
            TARGET_FRAMES * 1000 /
            SAMPLE_RATE
        )
    );

    ESP_LOGI(
        TAG,
        "Normal correction=+/- %ld ppm "
        "Recovery=+/- %ld ppm",

        static_cast<long>(
            NORMAL_LIMIT_PPM
        ),

        static_cast<long>(
            RECOVERY_LIMIT_PPM
        )
    );

    return ESP_OK;
}

// ============================================================
// Nonblocking audio consumer
// ============================================================

bool read_block(int16_t *output)
{
    if (output == nullptr) {
        return false;
    }

    if (rx_channel == nullptr) {

        std::memset(
            output,
            0,
            BLOCK_BYTES
        );

        return false;
    }

    if (
        reset_requested.exchange(
            false,
            std::memory_order_acq_rel
        )
    ) {

        resynchronize();

        conceal_block(output);

        return true;
    }

    if (!playback_started) {

        const uint32_t available =
            available_frames();

        if (available < PREFILL_FRAMES) {

            conceal_block(output);

            return true;
        }

        playback_started = true;

        need_fade_in = true;

        reset_controller();

        prebuffer_events.fetch_add(1);
    }

    if (!render_audio_block(output)) {

        missing_blocks.fetch_add(1);

        underflow_events.fetch_add(1);

        playback_started = false;

        need_fade_in = true;

        reset_controller();

        conceal_block(output);

        return true;
    }

    if (need_fade_in) {

        fade_in(output);

        need_fade_in = false;
    }

    last_left =
        output[BLOCK_SAMPLES - 2];

    last_right =
        output[BLOCK_SAMPLES - 1];

    return true;
}

// ============================================================
// Statistics
// ============================================================

void read_and_reset_stats(Stats &stats)
{
    stats.received_blocks =
        received_blocks.exchange(0);

    stats.consumed_blocks =
        consumed_blocks.exchange(0);

    stats.dropped_blocks =
        dropped_blocks.exchange(0);

    stats.missing_blocks =
        missing_blocks.exchange(0);

    stats.read_errors =
        read_errors.exchange(0);

    stats.peak =
        peak.exchange(0);

    if (rx_channel == nullptr) {
        return;
    }

    const uint32_t occupancy =
        available_frames();

    const uint32_t minimum =
        occupancy_min.exchange(
            occupancy
        );

    const uint32_t maximum =
        occupancy_max.exchange(
            occupancy
        );

    const uint32_t filtered =
        filtered_frames.load(
            std::memory_order_relaxed
        );

    const int32_t ppm =
        correction_ppm.load(
            std::memory_order_relaxed
        );

    ESP_LOGI(
        TAG,

        "SYNC | buf=%lu filtered=%lu "
        "min=%lu max=%lu "
        "target=%lu ppm=%ld",

        static_cast<unsigned long>(
            occupancy
        ),

        static_cast<unsigned long>(
            filtered
        ),

        static_cast<unsigned long>(
            minimum
        ),

        static_cast<unsigned long>(
            maximum
        ),

        static_cast<unsigned long>(
            TARGET_FRAMES
        ),

        static_cast<long>(ppm)
    );

    const uint32_t incoming =
        received_frames.exchange(0);

    const uint32_t used =
        source_frames_used.exchange(0);

    const uint32_t generated =
        output_frames_generated.exchange(0);

    ESP_LOGI(
        TAG,

        "SYNC RATE | rx=%lu used=%lu "
        "out=%lu",

        static_cast<unsigned long>(
            incoming
        ),

        static_cast<unsigned long>(
            used
        ),

        static_cast<unsigned long>(
            generated
        )
    );

    ESP_LOGI(
        TAG,

        "SYNC ERR | uf=%lu of=%lu "
        "pre=%lu conceal=%lu "
        "resync=%lu",

        static_cast<unsigned long>(
            underflow_events.exchange(0)
        ),

        static_cast<unsigned long>(
            overflow_events.exchange(0)
        ),

        static_cast<unsigned long>(
            prebuffer_events.exchange(0)
        ),

        static_cast<unsigned long>(
            concealed_blocks.exchange(0)
        ),

        static_cast<unsigned long>(
            resync_events.exchange(0)
        )
    );
}

} // namespace phone_audio
