#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_a2dp_api.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


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
     * ========================================================
     * RAW A2DP INPUT RING
     * ========================================================
     *
     * 44.1 kHz stereo PCM16 consumes:
     *
     * 44100 * 2 * 2 = 176400 bytes/sec
     *
     * 12288 bytes gives us roughly 69 ms of burst absorption.
     *
     * The old implementation used an 8 KB FreeRTOS
     * StreamBuffer. This fixed SPSC ring removes StreamBuffer
     * copies and bookkeeping from the Bluetooth hot path.
     *
     * Must remain divisible by one stereo PCM frame:
     *
     * 2 channels * 16 bit = 4 bytes.
     */

    static constexpr size_t INPUT_RING_BYTES =
        12288;


    /*
     * ========================================================
     * 48 kHz OUTPUT RING
     * ========================================================
     *
     * Four complete 10 ms blocks = 40 ms capacity.
     *
     * We normally target three queued blocks, or roughly
     * 30 ms of ready-to-play audio.
     *
     * The fourth slot gives the producer one block of working
     * headroom.
     */

    static constexpr size_t OUTPUT_RING_BLOCKS =
        4;


    static constexpr size_t OUTPUT_TARGET_BLOCKS =
        3;


    static constexpr size_t PREBUFFER_BLOCKS =
        3;


    /*
     * Do not restart the entire Bluetooth buffer after one
     * isolated late burst.
     *
     * Three consecutive failures mean the stream has actually
     * fallen behind and should rebuild its cushion.
     */

    static constexpr uint32_t MAX_CONSECUTIVE_UNDERFLOWS =
        3;


    /*
     * The Bluetooth worker stays immediately below the H19
     * audio task.
     *
     * H19 audio = priority 6
     * BT worker = priority 5
     */

    static constexpr uint32_t WORKER_STACK =
        4096;


    static constexpr UBaseType_t WORKER_PRIORITY =
        5;


    /*
     * Small fade used after an isolated underflow.
     *
     * This avoids the hard edge created when H19 suddenly
     * inserts an all-zero 10 ms block.
     */

    static constexpr size_t FADE_IN_FRAMES =
        64;


    /*
     * ========================================================
     * ESP-IDF CALLBACKS
     * ========================================================
     */

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


    /*
     * ========================================================
     * WORKER
     * ========================================================
     */

    void worker_task();


    void wake_worker();


    void reset_worker_stream_state();


    /*
     * ========================================================
     * INPUT RING
     * ========================================================
     */

    bool enqueue_input_pcm(
        const uint8_t *data,
        size_t length
    );


    size_t input_bytes_available() const;


    size_t input_free_bytes() const;


    void flush_input_ring();


    /*
     * ========================================================
     * OUTPUT RING
     * ========================================================
     */

    size_t output_blocks_available() const;


    void flush_output_ring();


    /*
     * ========================================================
     * RESAMPLER
     * ========================================================
     */

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


    static int16_t interpolate_sample(
        int16_t previous,
        int16_t current,
        uint32_t fraction_q16
    );


    /*
     * ========================================================
     * UNDERFLOW CONCEALMENT
     * ========================================================
     */

    void create_underflow_fade(
        int16_t *output
    );


    void apply_fade_in(
        int16_t *output
    );


    /*
     * ========================================================
     * STATE
     * ========================================================
     */

    bool initialized_ =
        false;


    TaskHandle_t worker_task_handle_ =
        nullptr;


    /*
     * ========================================================
     * RAW INPUT RING
     *
     * Only the Bluetooth callback writes input_write_bytes_.
     * Only the worker writes input_read_bytes_.
     *
     * Monotonic counters allow the buffer to wrap without
     * making full and empty look identical.
     * ========================================================
     */

    alignas(4)
    uint8_t input_ring_[
        INPUT_RING_BYTES
    ] = {};


    std::atomic<uint32_t>
        input_write_bytes_{
            0
        };


    std::atomic<uint32_t>
        input_read_bytes_{
            0
        };


    /*
     * ========================================================
     * OUTPUT BLOCK RING
     *
     * Worker = producer.
     * Audio task = consumer.
     * ========================================================
     */

    alignas(4)
    int16_t output_ring_[
        OUTPUT_RING_BLOCKS
    ][
        OUTPUT_SAMPLES_PER_BLOCK
    ] = {};


    std::atomic<uint32_t>
        output_write_blocks_{
            0
        };


    std::atomic<uint32_t>
        output_read_blocks_{
            0
        };


    /*
     * One block under construction by the resampler.
     */

    int16_t output_block_[
        OUTPUT_SAMPLES_PER_BLOCK
    ] = {};


    size_t output_frames_in_block_ =
        0;


    /*
     * ========================================================
     * RESAMPLER STATE
     * ========================================================
     */

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


    /*
     * ========================================================
     * PLAYBACK STATE
     * ========================================================
     */

    bool playback_started_ =
        false;


    uint32_t consecutive_underflows_ =
        0;


    bool have_last_output_sample_ =
        false;


    int16_t last_output_left_ =
        0;


    int16_t last_output_right_ =
        0;


    bool fade_in_next_block_ =
        false;


    /*
     * The event callback sets this whenever transport state
     * changes enough that the worker should throw away old
     * resampler/input state.
     */

    std::atomic<bool>
        worker_reset_requested_{
            false
        };


    /*
     * ========================================================
     * CONNECTION STATE
     * ========================================================
     */

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


    /*
     * ========================================================
     * TELEMETRY
     * ========================================================
     */

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