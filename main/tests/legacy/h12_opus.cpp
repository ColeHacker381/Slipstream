#include "h12_opus.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "audio/audio_io.hpp"
#include "audio/audio_processor.hpp"

#include "codec/opus_codec.hpp"
#include "codec/wm8960.hpp"

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"


namespace {

constexpr const char *TAG =
    "H12";


/*
 * ------------------------------------------------
 * Task configuration.
 * ------------------------------------------------
 *
 * Audio:
 *     high priority
 *     Core 1
 *
 * Codec:
 *     slightly lower priority
 *     Core 0
 *
 * Opus gets the large stack because libopus uses
 * substantial temporary stack space.
 * ------------------------------------------------
 */

constexpr uint32_t AUDIO_TASK_STACK =
    24 * 1024;


constexpr uint32_t CODEC_TASK_STACK =
    40 * 1024;


constexpr UBaseType_t AUDIO_TASK_PRIORITY =
    6;


constexpr UBaseType_t CODEC_TASK_PRIORITY =
    5;


/*
 * ------------------------------------------------
 * Complete 20 ms Opus PCM frame.
 * ------------------------------------------------
 */

struct VoiceFrame {

    int16_t samples[
        slipstream::OpusCodec::FRAME_SAMPLES
    ];
};


/*
 * A few frames of buffering are enough.
 *
 * The encoder should normally consume one frame
 * before the next 20 ms frame arrives.
 */
constexpr size_t FRAME_QUEUE_DEPTH =
    3;


QueueHandle_t encode_queue =
    nullptr;


QueueHandle_t decoded_queue =
    nullptr;


/*
 * The Opus encoder and decoder are accessed only
 * by codec_task().
 */
slipstream::OpusCodec opus_codec;


/*
 * ------------------------------------------------
 * Audio task buffers.
 * ------------------------------------------------
 */

int16_t hardware_input[
    audio_processor::
        HARDWARE_FRAMES_PER_BLOCK *
    2
] = {};


int16_t hardware_output[
    audio_processor::
        HARDWARE_FRAMES_PER_BLOCK *
    2
] = {};


int16_t voice_block[
    audio_processor::
        VOICE_SAMPLES_PER_BLOCK
] = {};


int16_t silence_block[
    audio_processor::
        VOICE_SAMPLES_PER_BLOCK
] = {};


/*
 * ------------------------------------------------
 * CODEC TASK — CORE 0
 * ------------------------------------------------
 *
 * Receives one 20 ms frame every 20 ms.
 *
 * At 240 MHz our current measurements are
 * approximately:
 *
 *     encode ~13.8 ms
 *     decode  ~1.9 ms
 *
 * so this task should be able to finish before
 * the next frame arrives.
 * ------------------------------------------------
 */

void codec_task(
    void *argument
)
{
    (void) argument;


    ESP_LOGI(
        TAG,
        "Codec task starting on CPU%d",
        xPortGetCoreID()
    );


    if (
        !opus_codec.init()
    ) {

        ESP_LOGE(
            TAG,
            "H12 FAIL: Opus initialization failed"
        );


        vTaskDelete(
            nullptr
        );


        return;
    }


    opus_codec.reset();


    VoiceFrame input_frame = {};

    VoiceFrame decoded_frame = {};


    /*
     * Codec statistics.
     */
    uint64_t frame_count = 0;

    uint64_t total_encode_us = 0;

    uint64_t total_decode_us = 0;

    uint64_t total_encoded_bytes = 0;


    uint32_t max_encode_us = 0;

    uint32_t max_decode_us = 0;


    int min_packet_bytes =
        1000000;

    int max_packet_bytes =
        0;


    uint64_t dropped_decoded_frames =
        0;


    int64_t last_report_us =
        esp_timer_get_time();


    while (true) {

        /*
         * Block here while waiting for the next
         * complete 20 ms TX frame.
         *
         * This is important:
         *
         * while blocked, Core 0 is free for IDLE0,
         * preventing the watchdog starvation we saw
         * in the synchronous H12 test.
         */
        if (
            xQueueReceive(
                encode_queue,
                &input_frame,
                portMAX_DELAY
            )
            !=
            pdTRUE
        ) {

            continue;
        }


        slipstream::OpusCodecStats
            stats = {};


        bool ok =
            opus_codec.process_frame(
                input_frame.samples,
                decoded_frame.samples,
                stats
            );


        if (!ok) {

            ESP_LOGE(
                TAG,
                "Opus processing failed"
            );


            continue;
        }


        frame_count++;


        total_encode_us +=
            stats.encode_us;


        total_decode_us +=
            stats.decode_us;


        total_encoded_bytes +=
            static_cast<uint64_t>(
                stats.encoded_bytes
            );


        if (
            stats.encode_us >
            max_encode_us
        ) {

            max_encode_us =
                stats.encode_us;
        }


        if (
            stats.decode_us >
            max_decode_us
        ) {

            max_decode_us =
                stats.decode_us;
        }


        if (
            stats.encoded_bytes <
            min_packet_bytes
        ) {

            min_packet_bytes =
                stats.encoded_bytes;
        }


        if (
            stats.encoded_bytes >
            max_packet_bytes
        ) {

            max_packet_bytes =
                stats.encoded_bytes;
        }


        /*
         * Never block the codec waiting for playback.
         *
         * In the final system this will eventually
         * become the RX/jitter-buffer path.
         */
        if (
            xQueueSend(
                decoded_queue,
                &decoded_frame,
                0
            )
            !=
            pdTRUE
        ) {

            dropped_decoded_frames++;
        }


        /*
         * Report approximately once per second.
         */
        int64_t now =
            esp_timer_get_time();


        if (
            now -
            last_report_us >=
            1000000
        ) {

            if (
                frame_count >
                0
            ) {

                uint64_t average_encode =
                    total_encode_us /
                    frame_count;


                uint64_t average_decode =
                    total_decode_us /
                    frame_count;


                uint64_t average_bytes =
                    total_encoded_bytes /
                    frame_count;


                ESP_LOGI(
                    TAG,

                    "CODEC | "
                    "frames=%llu | "
                    "ENC=%llu/%lu us | "
                    "DEC=%llu/%lu us | "
                    "pkt=%llu B [%d..%d] | "
                    "rxdrop=%llu | "
                    "qTX=%u | "
                    "qRX=%u",

                    static_cast<
                        unsigned long long
                    >(
                        frame_count
                    ),

                    static_cast<
                        unsigned long long
                    >(
                        average_encode
                    ),

                    static_cast<
                        unsigned long
                    >(
                        max_encode_us
                    ),

                    static_cast<
                        unsigned long long
                    >(
                        average_decode
                    ),

                    static_cast<
                        unsigned long
                    >(
                        max_decode_us
                    ),

                    static_cast<
                        unsigned long long
                    >(
                        average_bytes
                    ),

                    min_packet_bytes,

                    max_packet_bytes,

                    static_cast<
                        unsigned long long
                    >(
                        dropped_decoded_frames
                    ),

                    static_cast<
                        unsigned int
                    >(
                        uxQueueMessagesWaiting(
                            encode_queue
                        )
                    ),

                    static_cast<
                        unsigned int
                    >(
                        uxQueueMessagesWaiting(
                            decoded_queue
                        )
                    )
                );
            }


            frame_count = 0;

            total_encode_us = 0;

            total_decode_us = 0;

            total_encoded_bytes = 0;


            max_encode_us = 0;

            max_decode_us = 0;


            min_packet_bytes =
                1000000;

            max_packet_bytes = 0;


            dropped_decoded_frames = 0;


            last_report_us =
                now;
        }
    }
}


/*
 * ------------------------------------------------
 * AUDIO TASK — CORE 1
 * ------------------------------------------------
 *
 * This task must NEVER wait for Opus.
 *
 * It only:
 *
 *     captures
 *     runs H6-H11
 *     queues complete TX frames
 *     consumes decoded frames when available
 *     performs playback
 * ------------------------------------------------
 */

void audio_task(
    void *argument
)
{
    (void) argument;


    ESP_LOGI(
        TAG,
        "Audio task starting on CPU%d",
        xPortGetCoreID()
    );


    /*
     * ------------------------------------------------
     * Codec / I2S hardware setup.
     *
     * Do it from the audio task so the audio-side
     * initialization occurs on Core 1.
     * ------------------------------------------------
     */

    ESP_ERROR_CHECK(
        wm8960::init_i2c()
    );


    if (
        !wm8960::probe()
    ) {

        ESP_LOGE(
            TAG,
            "H12 FAIL: WM8960 not detected"
        );


        vTaskDelete(
            nullptr
        );


        return;
    }


    ESP_ERROR_CHECK(
        wm8960::
            configure_full_duplex()
    );


    ESP_ERROR_CHECK(
        audio::
            init_full_duplex()
    );


    /*
     * ------------------------------------------------
     * Frozen H8-H11 pipeline.
     * ------------------------------------------------
     */

    audio_processor::reset();


    audio_processor::
        set_noise_suppression_enabled(
            true
        );


    audio_processor::
        set_vad_enabled(
            true
        );


    audio_processor::
        set_agc_enabled(
            true
        );


    audio_processor::
        set_limiter_enabled(
            true
        );


    ESP_LOGI(
        TAG,
        "Live asynchronous Opus loopback starting"
    );


    ESP_LOGI(
        TAG,
        "Keep quiet for first ~500 ms"
    );


    /*
     * ------------------------------------------------
     * TX framing state.
     * ------------------------------------------------
     */

    VoiceFrame tx_frame = {};


    size_t tx_half =
        0;


    /*
     * ------------------------------------------------
     * RX/playback state.
     * ------------------------------------------------
     */

    VoiceFrame playback_frame = {};


    bool have_playback_frame =
        false;


    size_t playback_half =
        0;


    /*
     * ------------------------------------------------
     * Statistics.
     * ------------------------------------------------
     */

    uint64_t block_count = 0;


    uint64_t total_processing_us = 0;

    uint64_t total_downsample_us = 0;

    uint64_t total_dsp_us = 0;

    uint64_t total_vad_us = 0;

    uint64_t total_agc_us = 0;

    uint64_t total_limiter_us = 0;

    uint64_t total_upsample_us = 0;


    uint32_t max_processing_us = 0;


    uint64_t dropped_tx_frames = 0;

    uint64_t playback_underruns = 0;


    bool noise_ready_reported =
        false;

    bool vad_ready_reported =
        false;


    while (true) {

        /*
         * ------------------------------------------------
         * 1. Read exactly one 10 ms hardware block.
         * ------------------------------------------------
         */

        size_t bytes_read = 0;


        ESP_ERROR_CHECK(
            audio::read(
                hardware_input,
                sizeof(
                    hardware_input
                ),
                &bytes_read
            )
        );


        if (
            bytes_read !=
            sizeof(
                hardware_input
            )
        ) {

            ESP_LOGW(
                TAG,
                "Short RX block: %zu / %zu",
                bytes_read,
                sizeof(
                    hardware_input
                )
            );


            continue;
        }


        int64_t start_us =
            esp_timer_get_time();


        /*
         * ------------------------------------------------
         * 2. H6-H11.
         * ------------------------------------------------
         */

        audio_processor::Stats
            stats = {};


        audio_processor::
            process_input_block(
                hardware_input,
                voice_block,
                stats
            );


        /*
         * ------------------------------------------------
         * 3. Accumulate TX PCM.
         *
         * Two 160-sample blocks make one 320-sample
         * Opus frame.
         * ------------------------------------------------
         */

        std::memcpy(
            &tx_frame.samples[
                tx_half *
                audio_processor::
                    VOICE_SAMPLES_PER_BLOCK
            ],
            voice_block,
            audio_processor::
                VOICE_SAMPLES_PER_BLOCK *
            sizeof(
                int16_t
            )
        );


        tx_half++;


        if (
            tx_half >= 2
        ) {

            /*
             * CRITICAL:
             *
             * Zero timeout.
             *
             * Audio never waits for Opus.
             */
            if (
                xQueueSend(
                    encode_queue,
                    &tx_frame,
                    0
                )
                !=
                pdTRUE
            ) {

                dropped_tx_frames++;
            }


            tx_half =
                0;
        }


        /*
         * ------------------------------------------------
         * 4. Determine 10 ms playback block.
         * ------------------------------------------------
         */

        const int16_t *playback_voice =
            silence_block;


        /*
         * If we consumed both halves of the current
         * decoded frame, try to acquire the next one.
         */
        if (
            !have_playback_frame ||
            playback_half >= 2
        ) {

            if (
                xQueueReceive(
                    decoded_queue,
                    &playback_frame,
                    0
                )
                ==
                pdTRUE
            ) {

                have_playback_frame =
                    true;


                playback_half =
                    0;
            }
            else {

                have_playback_frame =
                    false;


                playback_half =
                    0;


                /*
                 * Ignore startup underruns until NS
                 * has initialized.
                 */
                if (
                    noise_ready_reported
                ) {

                    playback_underruns++;
                }
            }
        }


        if (
            have_playback_frame
        ) {

            playback_voice =
                &playback_frame.samples[
                    playback_half *
                    audio_processor::
                        VOICE_SAMPLES_PER_BLOCK
                ];


            playback_half++;
        }


        /*
         * ------------------------------------------------
         * 5. 16 kHz decoded PCM -> 48 kHz I2S PCM.
         * ------------------------------------------------
         */

        audio_processor::
            render_voice_block(
                playback_voice,
                hardware_output,
                stats
            );


        uint32_t processing_us =
            static_cast<uint32_t>(
                esp_timer_get_time() -
                start_us
            );


        /*
         * ------------------------------------------------
         * 6. Write hardware output.
         * ------------------------------------------------
         */

        ESP_ERROR_CHECK(
            audio::write(
                hardware_output,
                sizeof(
                    hardware_output
                )
            )
        );


        /*
         * ------------------------------------------------
         * Statistics.
         * ------------------------------------------------
         */

        block_count++;


        total_processing_us +=
            processing_us;


        total_downsample_us +=
            stats.downsample_us;


        total_dsp_us +=
            stats.dsp_us;


        total_vad_us +=
            stats.vad_us;


        total_agc_us +=
            stats.agc_us;


        total_limiter_us +=
            stats.limiter_us;


        total_upsample_us +=
            stats.upsample_us;


        if (
            processing_us >
            max_processing_us
        ) {

            max_processing_us =
                processing_us;
        }


        /*
         * ------------------------------------------------
         * Initialization diagnostics.
         * ------------------------------------------------
         */

        if (
            !noise_ready_reported &&
            audio_processor::
                noise_estimate_ready()
        ) {

            ESP_LOGI(
                TAG,
                "H8 noise estimate ready"
            );


            noise_ready_reported =
                true;
        }


        if (
            !vad_ready_reported &&
            stats.vad_noise_ready
        ) {

            ESP_LOGI(
                TAG,
                "H9 VAD noise estimate ready"
            );


            vad_ready_reported =
                true;
        }


        /*
         * ------------------------------------------------
         * Report every 100 audio blocks = ~1 second.
         * ------------------------------------------------
         */

        if (
            block_count %
            100 ==
            0
        ) {

            ESP_LOGI(
                TAG,

                "AUDIO | "
                "blocks=%llu | "
                "proc=%llu/%lu us | "
                "down=%llu | "
                "DSP=%llu | "
                "VAD=%llu | "
                "AGC=%llu | "
                "LIM=%llu | "
                "UP=%llu | "
                "speech=%s | "
                "txdrop=%llu | "
                "underrun=%llu | "
                "qTX=%u | "
                "qRX=%u",

                static_cast<
                    unsigned long long
                >(
                    block_count
                ),

                static_cast<
                    unsigned long long
                >(
                    total_processing_us /
                    100
                ),

                static_cast<
                    unsigned long
                >(
                    max_processing_us
                ),

                static_cast<
                    unsigned long long
                >(
                    total_downsample_us /
                    100
                ),

                static_cast<
                    unsigned long long
                >(
                    total_dsp_us /
                    100
                ),

                static_cast<
                    unsigned long long
                >(
                    total_vad_us /
                    100
                ),

                static_cast<
                    unsigned long long
                >(
                    total_agc_us /
                    100
                ),

                static_cast<
                    unsigned long long
                >(
                    total_limiter_us /
                    100
                ),

                static_cast<
                    unsigned long long
                >(
                    total_upsample_us /
                    100
                ),

                stats.speech_active
                    ? "YES"
                    : "NO",

                static_cast<
                    unsigned long long
                >(
                    dropped_tx_frames
                ),

                static_cast<
                    unsigned long long
                >(
                    playback_underruns
                ),

                static_cast<
                    unsigned int
                >(
                    uxQueueMessagesWaiting(
                        encode_queue
                    )
                ),

                static_cast<
                    unsigned int
                >(
                    uxQueueMessagesWaiting(
                        decoded_queue
                    )
                )
            );


            /*
             * Real-time requirement now applies ONLY
             * to the audio task.
             */
            if (
                max_processing_us >
                10000
            ) {

                ESP_LOGW(
                    TAG,
                    "AUDIO DEADLINE MISSED: "
                    "%lu us > 10000 us",
                    static_cast<
                        unsigned long
                    >(
                        max_processing_us
                    )
                );
            }


            total_processing_us = 0;

            total_downsample_us = 0;

            total_dsp_us = 0;

            total_vad_us = 0;

            total_agc_us = 0;

            total_limiter_us = 0;

            total_upsample_us = 0;


            max_processing_us = 0;


            dropped_tx_frames = 0;

            playback_underruns = 0;
        }
    }
}

}


namespace h12 {

void run()
{
    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "Slipstream H12 - Async Opus"
    );


    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "Audio DSP: Core 1"
    );


    ESP_LOGI(
        TAG,
        "Opus codec: Core 0"
    );


    ESP_LOGI(
        TAG,
        "Opus: 16 kHz mono / 20 ms / 320 samples"
    );


    ESP_LOGI(
        TAG,
        "Opus bitrate: 20000 bps"
    );


    ESP_LOGI(
        TAG,
        "Opus complexity: 5"
    );


    ESP_LOGI(
        TAG,
        "CPU target: 240 MHz"
    );


    /*
     * ------------------------------------------------
     * Create queues.
     * ------------------------------------------------
     */

    encode_queue =
        xQueueCreate(
            FRAME_QUEUE_DEPTH,
            sizeof(
                VoiceFrame
            )
        );


    decoded_queue =
        xQueueCreate(
            FRAME_QUEUE_DEPTH,
            sizeof(
                VoiceFrame
            )
        );


    if (
        encode_queue == nullptr ||
        decoded_queue == nullptr
    ) {

        ESP_LOGE(
            TAG,
            "Failed to create H12 queues"
        );


        return;
    }


    /*
     * ------------------------------------------------
     * Start codec task on Core 0.
     * ------------------------------------------------
     */

    BaseType_t codec_result =
        xTaskCreatePinnedToCore(
            codec_task,
            "h12_codec",
            CODEC_TASK_STACK,
            nullptr,
            CODEC_TASK_PRIORITY,
            nullptr,
            0
        );


    if (
        codec_result !=
        pdPASS
    ) {

        ESP_LOGE(
            TAG,
            "Failed to create codec task"
        );


        return;
    }


    /*
     * ------------------------------------------------
     * Start real-time audio task on Core 1.
     * ------------------------------------------------
     */

    BaseType_t audio_result =
        xTaskCreatePinnedToCore(
            audio_task,
            "h12_audio",
            AUDIO_TASK_STACK,
            nullptr,
            AUDIO_TASK_PRIORITY,
            nullptr,
            1
        );


    if (
        audio_result !=
        pdPASS
    ) {

        ESP_LOGE(
            TAG,
            "Failed to create audio task"
        );


        return;
    }


    ESP_LOGI(
        TAG,
        "H12 tasks created"
    );


    /*
     * app_main may now return.
     */
}

}