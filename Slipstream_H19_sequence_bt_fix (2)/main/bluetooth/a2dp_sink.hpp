#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_a2dp_api.h"

#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"


namespace slipstream {
namespace bluetooth {


class A2dpSink {
public:

    static constexpr uint32_t OUTPUT_SAMPLE_RATE =
        48000;

    static constexpr size_t OUTPUT_FRAMES_PER_BLOCK =
        480;

    static constexpr size_t OUTPUT_SAMPLES_PER_BLOCK =
        OUTPUT_FRAMES_PER_BLOCK * 2;

    static constexpr size_t OUTPUT_BYTES_PER_BLOCK =
        OUTPUT_SAMPLES_PER_BLOCK *
        sizeof(int16_t);


    struct Stats {

        uint32_t callback_packets = 0;
        uint32_t callback_bytes = 0;
        uint32_t callback_drops = 0;

        uint32_t blocks_produced = 0;
        uint32_t block_drops = 0;
        uint32_t block_underflows = 0;

        uint32_t prebuffer_events = 0;

        uint32_t source_sample_rate = 0;

        size_t output_buffer_bytes = 0;

        bool connected = false;
        bool streaming = false;
        bool playback_started = false;
    };


    A2dpSink();


    bool init();


    bool read_block(
        int16_t *output
    );


    bool connected() const;


    bool streaming() const;


    uint32_t source_sample_rate() const;


    void get_stats(
        Stats &stats
    ) const;


    void reset_interval_stats();


private:

    /*
     * The earlier H19 version used:
     *
     *   input  = 12 KB
     *   output = 8 × 1920 B = 15 KB
     *
     * That worked conceptually but consumed too much internal
     * RAM once Opus and Bluedroid were both active.
     *
     * 8 KB input is still a substantial A2DP burst cushion.
     */
    static constexpr size_t INPUT_STREAM_BYTES =
        8192;


    /*
     * Six 10 ms output blocks = 60 ms.
     *
     * Playback still starts after 30 ms, leaving another
     * ~30 ms for jitter/burst absorption.
     */
    static constexpr size_t OUTPUT_STREAM_BYTES =
        OUTPUT_BYTES_PER_BLOCK * 6;


    static constexpr size_t PREBUFFER_BYTES =
        OUTPUT_BYTES_PER_BLOCK * 3;


    /*
     * Do not rebuild the full prebuffer for one isolated
     * 10 ms starvation. Only rebuffer after three consecutive
     * misses.
     */
    static constexpr uint32_t MAX_CONSECUTIVE_UNDERFLOWS =
        3;


    static constexpr uint32_t WORKER_STACK =
        4096;


    static constexpr UBaseType_t WORKER_PRIORITY =
        5;


    static void a2dp_event_callback(
        esp_a2d_cb_event_t event,
        esp_a2d_cb_param_t *param
    );


    static void a2dp_pcm_callback(
        const uint8_t *data,
        uint32_t length
    );


    static void worker_task_entry(
        void *argument
    );


    void worker_task();


    void process_pcm(
        const uint8_t *data,
        size_t length
    );


    void reset_resampler(
        uint32_t source_rate
    );


    void push_output_sample(
        int16_t left,
        int16_t right
    );


    void push_completed_output_block();


    void drain_output_stream();


    static int16_t interpolate_sample(
        int16_t previous,
        int16_t current,
        uint32_t fraction_q16
    );


    bool initialized_ = false;


    StreamBufferHandle_t input_stream_ =
        nullptr;


    StreamBufferHandle_t output_stream_ =
        nullptr;


    int16_t output_block_[
        OUTPUT_SAMPLES_PER_BLOCK
    ] = {};


    size_t output_frames_in_block_ =
        0;


    bool resampler_ready_ =
        false;


    uint32_t resampler_source_rate_ =
        0;


    uint32_t source_step_q16_ =
        0;


    uint64_t input_frame_index_ =
        0;


    uint64_t next_output_position_q16_ =
        0;


    int16_t previous_left_ =
        0;


    int16_t previous_right_ =
        0;


    bool playback_started_ =
        false;


    uint32_t consecutive_underflows_ =
        0;


    std::atomic<uint32_t>
        source_sample_rate_{
            44100
        };


    std::atomic<bool>
        connected_{
            false
        };


    std::atomic<bool>
        streaming_{
            false
        };


    std::atomic<uint32_t>
        callback_packets_{
            0
        };


    std::atomic<uint32_t>
        callback_bytes_{
            0
        };


    std::atomic<uint32_t>
        callback_drops_{
            0
        };


    std::atomic<uint32_t>
        blocks_produced_{
            0
        };


    std::atomic<uint32_t>
        block_drops_{
            0
        };


    std::atomic<uint32_t>
        block_underflows_{
            0
        };


    std::atomic<uint32_t>
        prebuffer_events_{
            0
        };


    static A2dpSink *instance_;
};


}
}