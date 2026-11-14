#include "h19_bluetooth_mix.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "audio/audio_io.hpp"
#include "audio/audio_processor.hpp"
#include "audio/priority_mixer.hpp"
#include "audio/rx_output_stage.hpp"

#include "bluetooth/a2dp_sink.hpp"

#include "codec/wm8960.hpp"

#include "transport/jitter_buffer.hpp"
#include "transport/simulated_rf.hpp"
#include "transport/slipstream_packet.hpp"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "opus.h"


namespace {

constexpr const char *TAG =
    "H19";


/*
 * ============================================================
 * OPUS
 * ============================================================
 */

constexpr uint32_t OPUS_SAMPLE_RATE =
    16000;

constexpr int OPUS_CHANNELS =
    1;

constexpr int OPUS_FRAME_SAMPLES =
    320;

constexpr int OPUS_BITRATE =
    20000;

constexpr int H19_OPUS_COMPLEXITY =
    3;

constexpr size_t MAX_OPUS_BYTES =
    512;


/*
 * ============================================================
 * TASK MEMORY
 * ============================================================
 *
 * Previous measurements:
 *
 * Audio 16 KB:
 *     >12 KB remained unused.
 *
 * Encoder 24 KB:
 *     only ~20 bytes remained.
 *
 * RX 8 KB:
 *     only ~20 bytes remained.
 *
 * New allocation preserves roughly the same total RAM while
 * putting it where it is actually required.
 */

constexpr uint32_t AUDIO_TASK_STACK =
    8 * 1024;

constexpr uint32_t ENCODE_TASK_STACK =
    28 * 1024;

constexpr uint32_t NETWORK_TASK_STACK =
    12 * 1024;


constexpr UBaseType_t AUDIO_TASK_PRIORITY =
    6;

constexpr UBaseType_t NETWORK_TASK_PRIORITY =
    4;

constexpr UBaseType_t ENCODE_TASK_PRIORITY =
    3;


constexpr size_t LATEST_QUEUE_DEPTH =
    1;


/*
 * Encoder -> transport must preserve short bursts of sequence
 * continuity. Three 20 ms packets gives 60 ms of scheduling
 * cushion without allowing unbounded intercom latency.
 */
constexpr size_t NETWORK_QUEUE_DEPTH =
    3;


constexpr uint32_t JITTER_DELAY_FRAMES =
    2;


constexpr uint8_t STREAM_ID =
    1;

constexpr uint8_t RF_CHANNEL =
    0;


constexpr size_t HARDWARE_FRAMES =
    audio_processor::
        HARDWARE_FRAMES_PER_BLOCK;


constexpr size_t HARDWARE_SAMPLES =
    HARDWARE_FRAMES * 2;


static_assert(
    HARDWARE_FRAMES ==
        slipstream::bluetooth::
            A2dpSink::
            OUTPUT_FRAMES_PER_BLOCK,
    "Bluetooth and hardware blocks must both be 10 ms"
);


/*
 * ============================================================
 * TYPES
 * ============================================================
 */

struct VoiceFrame {

    int16_t samples[
        OPUS_FRAME_SAMPLES
    ] = {};

    bool speech_active =
        false;

    bool plc =
        false;
};


struct NetworkPacket {

    size_t length =
        0;

    uint16_t sequence =
        0;

    uint8_t bytes[
        slipstream::transport::
            MAX_PACKET_SIZE
    ] = {};
};


/*
 * ============================================================
 * GLOBALS
 * ============================================================
 */

QueueHandle_t encode_queue =
    nullptr;

QueueHandle_t network_queue =
    nullptr;

QueueHandle_t decoded_queue =
    nullptr;


slipstream::transport::
    SimulatedRf simulated_rf;

slipstream::transport::
    PacketJitterBuffer jitter_buffer;

slipstream::bluetooth::
    A2dpSink a2dp_sink;

audio::PriorityMixer priority_mixer;

audio::RxOutputStage rx_output_stage;


std::atomic<bool>
    receiver_started{
        false
    };


std::atomic<bool>
    audio_pipeline_started{
        false
    };


int16_t hardware_input[
    HARDWARE_SAMPLES
] = {};

int16_t rf_hardware[
    HARDWARE_SAMPLES
] = {};

int16_t bt_hardware[
    HARDWARE_SAMPLES
] = {};

float mixed_float[
    HARDWARE_SAMPLES
] = {};

int16_t final_output[
    HARDWARE_SAMPLES
] = {};


int16_t voice_block[
    audio_processor::
        VOICE_SAMPLES_PER_BLOCK
] = {};


int16_t silence_voice_block[
    audio_processor::
        VOICE_SAMPLES_PER_BLOCK
] = {};


/*
 * ============================================================
 * HEAP LOG
 * ============================================================
 */

void log_heap(
    const char *label
)
{
    ESP_LOGI(
        TAG,
        "%s | free=%lu | largest=%lu | DMA=%lu",
        label,

        static_cast<unsigned long>(
            heap_caps_get_free_size(
                MALLOC_CAP_INTERNAL |
                MALLOC_CAP_8BIT
            )
        ),

        static_cast<unsigned long>(
            heap_caps_get_largest_free_block(
                MALLOC_CAP_INTERNAL |
                MALLOC_CAP_8BIT
            )
        ),

        static_cast<unsigned long>(
            heap_caps_get_free_size(
                MALLOC_CAP_DMA
            )
        )
    );
}


/*
 * ============================================================
 * LATEST-ITEM QUEUE
 * ============================================================
 */

template<typename T>
bool overwrite_realtime_queue(
    QueueHandle_t queue,
    const T &item
)
{
    if (
        queue ==
        nullptr
    ) {

        return false;
    }


    const bool replaced =
        uxQueueMessagesWaiting(
            queue
        )
        >
        0;


    xQueueOverwrite(
        queue,
        &item
    );


    return replaced;
}


/*
 * ============================================================
 * SHORT FIFO FOR ENCODED NETWORK PACKETS
 * ============================================================
 *
 * Unlike raw microphone input, encoded packets carry sequence
 * numbers consumed by the jitter buffer. Preserve normal FIFO
 * order across short Bluetooth scheduling stalls. If the queue
 * is truly full, drop the OLDEST packet and enqueue the newest
 * one so latency remains bounded. The receiver detects that
 * sequence gap and performs a clean 40 ms resynchronization.
 */

bool enqueue_network_packet(
    const NetworkPacket &packet
)
{
    if (
        network_queue ==
            nullptr
    ) {

        return false;
    }


    if (
        xQueueSend(
            network_queue,
            &packet,
            0
        )
        ==
        pdTRUE
    ) {

        return false;
    }


    NetworkPacket discarded = {};


    (void) xQueueReceive(
        network_queue,
        &discarded,
        0
    );


    (void) xQueueSend(
        network_queue,
        &packet,
        0
    );


    return true;
}


/*
 * ============================================================
 * H17 SELF TEST
 * ============================================================
 */

bool run_output_self_test()
{
    constexpr size_t TEST_FRAMES =
        32;


    float test_input[
        TEST_FRAMES * 2
    ] = {};


    int16_t test_output[
        TEST_FRAMES * 2
    ] = {};


    for (
        size_t frame = 0;
        frame < TEST_FRAMES;
        ++frame
    ) {

        test_input[
            frame * 2
        ] =
            50000.0f;


        test_input[
            frame * 2 + 1
        ] =
            -45000.0f;
    }


    audio::RxOutputStats
        stats = {};


    rx_output_stage.reset();


    rx_output_stage.process_block(
        test_input,
        test_output,
        TEST_FRAMES,
        stats
    );


    ESP_LOGI(
        TAG,
        "RX limiter self-test | "
        "IN=%.2f dBFS | "
        "OUT=%.2f dBFS | "
        "GR=%.2f dB | "
        "limited=%lu | "
        "clamps=%lu",

        stats.input_peak_dbfs,
        stats.output_peak_dbfs,
        stats.gain_reduction_db,

        static_cast<unsigned long>(
            stats.limited_frames
        ),

        static_cast<unsigned long>(
            stats.safety_clamps
        )
    );


    const bool pass =
        (
            stats.output_peak_dbfs
            <=
            audio::RxOutputStage::
                LIMITER_CEILING_DBFS
                +
                0.05f
        )
        &&
        (
            stats.safety_clamps ==
            0
        );


    rx_output_stage.reset();


    ESP_LOGI(
        TAG,
        "RX limiter self-test: %s",
        pass
            ? "PASS"
            : "FAIL"
    );


    return pass;
}


/*
 * ============================================================
 * AUDIO HARDWARE
 * ============================================================
 */

bool initialize_audio_hardware()
{
    ESP_LOGI(
        TAG,
        "Reserving WM8960 / I2S / DMA resources"
    );


    log_heap(
        "Before WM8960/I2S"
    );


    esp_err_t error =
        wm8960::init_i2c();


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "WM8960 I2C init failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    if (
        !wm8960::probe()
    ) {

        ESP_LOGE(
            TAG,
            "WM8960 not detected"
        );


        return false;
    }


    error =
        wm8960::
            configure_full_duplex();


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "WM8960 configuration failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    error =
        audio::
            init_full_duplex();


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "I2S full-duplex init failed: %s",
            esp_err_to_name(
                error
            )
        );


        log_heap(
            "I2S allocation failure"
        );


        return false;
    }


    log_heap(
        "After WM8960/I2S"
    );


    ESP_LOGI(
        TAG,
        "WM8960 + I2S DMA reserved successfully"
    );


    return true;
}


/*
 * ============================================================
 * ENCODER
 *
 * Core 0
 * ============================================================
 */

void encode_task(
    void *argument
)
{
    (void)argument;


    ESP_LOGI(
        TAG,
        "Encoder task running on CPU%d",
        xPortGetCoreID()
    );


    int opus_error =
        OPUS_OK;


    OpusEncoder *encoder =
        opus_encoder_create(
            OPUS_SAMPLE_RATE,
            OPUS_CHANNELS,
            OPUS_APPLICATION_VOIP,
            &opus_error
        );


    if (
        encoder ==
            nullptr ||
        opus_error !=
            OPUS_OK
    ) {

        ESP_LOGE(
            TAG,
            "Opus encoder create failed: %d",
            opus_error
        );


        vTaskDelete(
            nullptr
        );


        return;
    }


    opus_encoder_ctl(
        encoder,
        OPUS_SET_BITRATE(
            OPUS_BITRATE
        )
    );


    opus_encoder_ctl(
        encoder,
        OPUS_SET_COMPLEXITY(
            H19_OPUS_COMPLEXITY
        )
    );


    opus_encoder_ctl(
        encoder,
        OPUS_SET_VBR(
            1
        )
    );


    opus_encoder_ctl(
        encoder,
        OPUS_SET_INBAND_FEC(
            0
        )
    );


    opus_encoder_ctl(
        encoder,
        OPUS_SET_DTX(
            0
        )
    );


    ESP_LOGI(
        TAG,
        "H19 Opus TX: "
        "16k mono / 20ms / 20kbps / complexity %d",
        H19_OPUS_COMPLEXITY
    );


    VoiceFrame input_frame = {};


    uint8_t opus_payload[
        MAX_OPUS_BYTES
    ] = {};


    NetworkPacket
        network_packet = {};


    uint16_t sequence =
        0;


    uint32_t timestamp =
        0;


    uint64_t frames =
        0;


    uint64_t encode_total_us =
        0;


    uint32_t encode_max_us =
        0;


    uint64_t output_replacements =
        0;


    int64_t last_report =
        esp_timer_get_time();


    while (
        true
    ) {

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


        const int64_t start =
            esp_timer_get_time();


        const int encoded_bytes =
            opus_encode(
                encoder,
                input_frame.samples,
                OPUS_FRAME_SAMPLES,
                opus_payload,
                sizeof(
                    opus_payload
                )
            );


        const uint32_t encode_us =
            static_cast<uint32_t>(
                esp_timer_get_time()
                -
                start
            );


        if (
            encoded_bytes <
            0
        ) {

            ESP_LOGE(
                TAG,
                "Opus encode failed: %d",
                encoded_bytes
            );


            vTaskDelay(
                1
            );


            continue;
        }


        frames++;


        encode_total_us +=
            encode_us;


        encode_max_us =
            std::max(
                encode_max_us,
                encode_us
            );


        slipstream::transport::
            PacketHeader header = {};


        header.flags =
            input_frame.
                speech_active
            ?
            slipstream::transport::
                FLAG_SPEECH
            :
            slipstream::transport::
                FLAG_NONE;


        header.stream_id =
            STREAM_ID;


        header.channel =
            RF_CHANNEL;


        header.sequence =
            sequence;


        header.timestamp =
            timestamp;


        network_packet.length =
            0;


        network_packet.sequence =
            sequence;


        if (
            slipstream::transport::
                build_packet(
                    header,
                    opus_payload,
                    static_cast<size_t>(
                        encoded_bytes
                    ),
                    network_packet.bytes,
                    sizeof(
                        network_packet.bytes
                    ),
                    network_packet.length
                )
        ) {

            if (
                enqueue_network_packet(
                    network_packet
                )
            ) {

                output_replacements++;
            }
        }


        sequence++;


        timestamp +=
            OPUS_FRAME_SAMPLES;


        /*
         * Give controller / Bluedroid / IDLE0 a guaranteed
         * scheduling opportunity.
         */
        vTaskDelay(
            1
        );


        const int64_t now =
            esp_timer_get_time();


        if (
            now -
            last_report >=
            1000000
        ) {

            const uint64_t average =
                frames >
                    0
                ?
                encode_total_us /
                    frames
                :
                0;


            ESP_LOGI(
                TAG,
                "ENC | "
                "frames=%llu | "
                "time=%llu/%lu us | "
                "netDrop=%llu | "
                "q=%u | "
                "stack=%u",

                static_cast<
                    unsigned long long
                >(
                    frames
                ),

                static_cast<
                    unsigned long long
                >(
                    average
                ),

                static_cast<
                    unsigned long
                >(
                    encode_max_us
                ),

                static_cast<
                    unsigned long long
                >(
                    output_replacements
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
                    uxTaskGetStackHighWaterMark(
                        nullptr
                    )
                )
            );


            frames =
                0;


            encode_total_us =
                0;


            encode_max_us =
                0;


            output_replacements =
                0;


            last_report =
                now;
        }
    }
}


/*
 * ============================================================
 * NETWORK / DECODER
 *
 * Core 1
 * ============================================================
 */

void network_decode_task(
    void *argument
)
{
    (void)argument;


    ESP_LOGI(
        TAG,
        "Network/decoder task running on CPU%d",
        xPortGetCoreID()
    );


    /*
     * Do not generate hundreds of artificial PLC frames while
     * waiting for the actual audio producer to start.
     */

    while (
        !audio_pipeline_started.
            load()
    ) {

        vTaskDelay(
            pdMS_TO_TICKS(
                10
            )
        );
    }


    int opus_error =
        OPUS_OK;


    OpusDecoder *decoder =
        opus_decoder_create(
            OPUS_SAMPLE_RATE,
            OPUS_CHANNELS,
            &opus_error
        );


    if (
        decoder ==
            nullptr ||
        opus_error !=
            OPUS_OK
    ) {

        ESP_LOGE(
            TAG,
            "Opus decoder create failed: %d",
            opus_error
        );


        vTaskDelete(
            nullptr
        );


        return;
    }


    simulated_rf.reset();


    jitter_buffer.reset();


    NetworkPacket
        network_packet = {};


    uint8_t received_packet[
        slipstream::transport::
            MAX_PACKET_SIZE
    ] = {};


    uint8_t playout_packet[
        slipstream::transport::
            MAX_PACKET_SIZE
    ] = {};


    VoiceFrame output_frame = {};


    uint32_t rf_tick =
        0;


    uint16_t playout_sequence =
        0;


    bool network_sequence_valid =
        false;


    uint16_t expected_network_sequence =
        0;


    uint64_t sequence_resyncs =
        0;


    uint64_t decoded_frames =
        0;


    uint64_t plc_frames =
        0;


    uint64_t misses =
        0;


    uint64_t crc_rejects =
        0;


    uint64_t output_replacements =
        0;


    uint64_t decode_total_us =
        0;


    uint32_t decode_max_us =
        0;


    int64_t last_report =
        esp_timer_get_time();


    TickType_t last_wake =
        xTaskGetTickCount();


    constexpr TickType_t PERIOD =
        pdMS_TO_TICKS(
            20
        );


    while (
        true
    ) {

        vTaskDelayUntil(
            &last_wake,
            PERIOD
        );


        if (
            xQueueReceive(
                network_queue,
                &network_packet,
                0
            )
            ==
            pdTRUE
        ) {

            /*
             * The queue normally preserves every sequence. If
             * overload forces one old packet to be discarded,
             * the old playout_sequence would otherwise chase a
             * packet that can never arrive and produce PLC
             * forever. Reset the local simulated RF/jitter path
             * and rebuild the frozen 40 ms delay from the first
             * packet after the gap.
             */

            const bool sequence_gap =
                network_sequence_valid &&
                network_packet.sequence !=
                    expected_network_sequence;


            if (
                !network_sequence_valid ||
                sequence_gap
            ) {

                if (
                    sequence_gap
                ) {

                    sequence_resyncs++;
                }


                simulated_rf.reset();


                jitter_buffer.reset();


                rf_tick =
                    0;


                playout_sequence =
                    network_packet.sequence;


                receiver_started.store(
                    false
                );


                network_sequence_valid =
                    true;
            }


            expected_network_sequence =
                static_cast<uint16_t>(
                    network_packet.sequence +
                    1U
                );


            simulated_rf.send(
                network_packet.bytes,
                network_packet.length,
                network_packet.sequence,
                rf_tick
            );
        }


        while (
            true
        ) {

            size_t received_length =
                0;


            if (
                !simulated_rf.receive(
                    rf_tick,
                    received_packet,
                    sizeof(
                        received_packet
                    ),
                    received_length
                )
            ) {

                break;
            }


            slipstream::transport::
                ParsedPacket parsed = {};


            const auto result =
                slipstream::transport::
                    parse_packet(
                        received_packet,
                        received_length,
                        parsed
                    );


            if (
                result ==
                slipstream::transport::
                    ParseResult::BAD_CRC
            ) {

                crc_rejects++;


                continue;
            }


            if (
                result !=
                slipstream::transport::
                    ParseResult::OK
            ) {

                continue;
            }


            jitter_buffer.insert(
                received_packet,
                received_length,
                parsed.header.sequence
            );
        }


        if (
            rf_tick >=
            JITTER_DELAY_FRAMES
        ) {

            receiver_started.store(
                true
            );


            jitter_buffer.
                set_expected_sequence(
                    playout_sequence
                );


            size_t playout_length =
                0;


            const bool packet_available =
                jitter_buffer.take(
                    playout_sequence,
                    playout_packet,
                    sizeof(
                        playout_packet
                    ),
                    playout_length
                );


            output_frame = {};


            bool decoded_ok =
                false;


            uint32_t decode_us =
                0;


            if (
                packet_available
            ) {

                slipstream::transport::
                    ParsedPacket parsed = {};


                if (
                    slipstream::transport::
                        parse_packet(
                            playout_packet,
                            playout_length,
                            parsed
                        )
                    ==
                    slipstream::transport::
                        ParseResult::OK
                ) {

                    const int64_t start =
                        esp_timer_get_time();


                    const int samples =
                        opus_decode(
                            decoder,
                            parsed.payload,
                            static_cast<
                                opus_int32
                            >(
                                parsed.payload_length
                            ),
                            output_frame.samples,
                            OPUS_FRAME_SAMPLES,
                            0
                        );


                    decode_us =
                        static_cast<uint32_t>(
                            esp_timer_get_time()
                            -
                            start
                        );


                    if (
                        samples ==
                        OPUS_FRAME_SAMPLES
                    ) {

                        decoded_ok =
                            true;


                        output_frame.plc =
                            false;


                        output_frame.
                            speech_active =
                                (
                                    parsed.header.flags
                                    &
                                    slipstream::transport::
                                        FLAG_SPEECH
                                )
                                !=
                                0;


                        decoded_frames++;
                    }
                }
            }


            if (
                !decoded_ok
            ) {

                misses++;


                const int64_t start =
                    esp_timer_get_time();


                const int samples =
                    opus_decode(
                        decoder,
                        nullptr,
                        0,
                        output_frame.samples,
                        OPUS_FRAME_SAMPLES,
                        0
                    );


                decode_us =
                    static_cast<uint32_t>(
                        esp_timer_get_time()
                        -
                        start
                    );


                if (
                    samples ==
                    OPUS_FRAME_SAMPLES
                ) {

                    output_frame.plc =
                        true;


                    output_frame.
                        speech_active =
                            false;


                    plc_frames++;
                }
            }


            decode_total_us +=
                decode_us;


            decode_max_us =
                std::max(
                    decode_max_us,
                    decode_us
                );


            if (
                overwrite_realtime_queue(
                    decoded_queue,
                    output_frame
                )
            ) {

                output_replacements++;
            }


            playout_sequence++;
        }


        rf_tick++;


        const int64_t now =
            esp_timer_get_time();


        if (
            now -
            last_report >=
            1000000
        ) {

            const uint64_t total =
                decoded_frames +
                plc_frames;


            const uint64_t average =
                total >
                    0
                ?
                decode_total_us /
                    total
                :
                0;


            ESP_LOGI(
                TAG,
                "RX | "
                "decoded=%llu | "
                "PLC=%llu | "
                "miss=%llu | "
                "crc=%llu | "
                "DEC=%llu/%lu us | "
                "jbuf=%u | "
                "replace=%llu | "
                "resync=%llu | "
                "stack=%u",

                static_cast<
                    unsigned long long
                >(
                    decoded_frames
                ),

                static_cast<
                    unsigned long long
                >(
                    plc_frames
                ),

                static_cast<
                    unsigned long long
                >(
                    misses
                ),

                static_cast<
                    unsigned long long
                >(
                    crc_rejects
                ),

                static_cast<
                    unsigned long long
                >(
                    average
                ),

                static_cast<
                    unsigned long
                >(
                    decode_max_us
                ),

                static_cast<
                    unsigned int
                >(
                    jitter_buffer.size()
                ),

                static_cast<
                    unsigned long long
                >(
                    output_replacements
                ),

                static_cast<
                    unsigned long long
                >(
                    sequence_resyncs
                ),

                static_cast<
                    unsigned int
                >(
                    uxTaskGetStackHighWaterMark(
                        nullptr
                    )
                )
            );


            decoded_frames =
                0;


            plc_frames =
                0;


            misses =
                0;


            crc_rejects =
                0;


            output_replacements =
                0;


            sequence_resyncs =
                0;


            decode_total_us =
                0;


            decode_max_us =
                0;


            last_report =
                now;
        }
    }
}


/*
 * ============================================================
 * AUDIO
 *
 * Core 1 / highest application priority
 * ============================================================
 */

void audio_task(
    void *argument
)
{
    (void)argument;


    ESP_LOGI(
        TAG,
        "Audio/mixer task running on CPU%d",
        xPortGetCoreID()
    );


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


    priority_mixer.reset();


    rx_output_stage.reset();


    rx_output_stage.
        set_eq_enabled(
            false
        );


    rx_output_stage.
        set_master_gain_db(
            audio::RxOutputStage::
                MASTER_GAIN_DB
        );


    VoiceFrame tx_frame = {};


    size_t tx_half =
        0;


    VoiceFrame playback_frame = {};


    bool have_playback =
        false;


    size_t playback_half =
        0;


    uint64_t blocks =
        0;


    uint64_t tx_replacements =
        0;


    uint64_t rx_underruns =
        0;


    uint64_t plc_played =
        0;


    uint64_t bt_good_blocks =
        0;


    uint64_t bt_silence_blocks =
        0;


    uint64_t mic_speech_blocks =
        0;


    uint64_t limiter_frames =
        0;


    uint64_t safety_clamps =
        0;


    uint64_t total_processing_us =
        0;


    uint32_t max_processing_us =
        0;


    float max_input_peak =
        -120.0f;


    float max_output_peak =
        -120.0f;


    float max_gain_reduction =
        0.0f;


    audio_pipeline_started.store(
        true
    );


    ESP_LOGI(
        TAG,
        "H19 integrated Bluetooth pipeline active"
    );


    while (
        true
    ) {

        size_t bytes_read =
            0;


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

            continue;
        }


        const int64_t start =
            esp_timer_get_time();


        audio_processor::Stats
            processor_stats = {};


        audio_processor::
            process_input_block(
                hardware_input,
                voice_block,
                processor_stats
            );


        if (
            processor_stats.
                speech_active
        ) {

            mic_speech_blocks++;
        }


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


        if (
            processor_stats.
                speech_active
        ) {

            tx_frame.
                speech_active =
                    true;
        }


        tx_half++;


        if (
            tx_half >=
            2
        ) {

            if (
                overwrite_realtime_queue(
                    encode_queue,
                    tx_frame
                )
            ) {

                tx_replacements++;
            }


            tx_frame = {};


            tx_half =
                0;
        }


        const int16_t *rf_voice =
            silence_voice_block;


        bool rf_speech =
            false;


        if (
            !have_playback ||
            playback_half >=
                2
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

                have_playback =
                    true;


                playback_half =
                    0;


                if (
                    playback_frame.
                        plc
                ) {

                    plc_played++;
                }
            }
            else {

                have_playback =
                    false;


                playback_half =
                    0;


                if (
                    receiver_started.
                        load()
                ) {

                    rx_underruns++;
                }
            }
        }


        if (
            have_playback
        ) {

            rf_voice =
                &playback_frame.samples[
                    playback_half *
                    audio_processor::
                        VOICE_SAMPLES_PER_BLOCK
                ];


            rf_speech =
                playback_frame.
                    speech_active;


            playback_half++;
        }


        audio_processor::
            render_voice_block(
                rf_voice,
                rf_hardware,
                processor_stats
            );


        const bool bt_available =
            a2dp_sink.read_block(
                bt_hardware
            );


        if (
            bt_available
        ) {

            bt_good_blocks++;
        }
        else {

            std::memset(
                bt_hardware,
                0,
                sizeof(
                    bt_hardware
                )
            );


            bt_silence_blocks++;
        }


        priority_mixer.
            set_rf_speech_active(
                rf_speech
            );


        priority_mixer.
            process_block_float(
                bt_hardware,
                rf_hardware,
                mixed_float,
                HARDWARE_SAMPLES
            );


        audio::RxOutputStats
            output_stats = {};


        rx_output_stage.
            process_block(
                mixed_float,
                final_output,
                HARDWARE_FRAMES,
                output_stats
            );


        ESP_ERROR_CHECK(
            audio::write(
                final_output,
                sizeof(
                    final_output
                )
            )
        );


        const uint32_t processing_us =
            static_cast<uint32_t>(
                esp_timer_get_time()
                -
                start
            );


        blocks++;


        total_processing_us +=
            processing_us;


        max_processing_us =
            std::max(
                max_processing_us,
                processing_us
            );


        limiter_frames +=
            output_stats.
                limited_frames;


        safety_clamps +=
            output_stats.
                safety_clamps;


        max_input_peak =
            std::max(
                max_input_peak,
                output_stats.
                    input_peak_dbfs
            );


        max_output_peak =
            std::max(
                max_output_peak,
                output_stats.
                    output_peak_dbfs
            );


        max_gain_reduction =
            std::max(
                max_gain_reduction,
                output_stats.
                    gain_reduction_db
            );


        if (
            blocks >=
            100
        ) {

            slipstream::bluetooth::
                A2dpSink::Stats bt_stats = {};


            a2dp_sink.get_stats(
                bt_stats
            );


            ESP_LOGI(
                TAG,
                "AUDIO | "
                "proc=%llu/%lu us | "
                "micSpeech=%llu | "
                "RF=%s | "
                "duck=%s | "
                "BTgain=%.2f | "
                "BT=%s/%s/%s %luHz | "
                "bt=%llu/%llu | "
                "buf=%uB | "
                "btDrop=%lu/%lu | "
                "btUF=%lu | "
                "pre=%lu | "
                "IN=%.2f | "
                "OUT=%.2f | "
                "GR=%.2f | "
                "lim=%llu | "
                "clamp=%llu | "
                "txReplace=%llu | "
                "rxUF=%llu | "
                "PLC=%llu | "
                "heap=%lu | "
                "dma=%lu | "
                "stack=%u",

                static_cast<
                    unsigned long long
                >(
                    total_processing_us /
                    blocks
                ),

                static_cast<
                    unsigned long
                >(
                    max_processing_us
                ),

                static_cast<
                    unsigned long long
                >(
                    mic_speech_blocks
                ),

                rf_speech ?
                    "YES" :
                    "NO",

                priority_mixer.ducking() ?
                    "YES" :
                    "NO",

                priority_mixer.
                    current_bt_gain_db(),

                bt_stats.connected ?
                    "CONN" :
                    "DISC",

                bt_stats.streaming ?
                    "PLAY" :
                    "IDLE",

                bt_stats.playback_started ?
                    "RUN" :
                    "PRE",

                static_cast<
                    unsigned long
                >(
                    bt_stats.
                        source_sample_rate
                ),

                static_cast<
                    unsigned long long
                >(
                    bt_good_blocks
                ),

                static_cast<
                    unsigned long long
                >(
                    bt_silence_blocks
                ),

                static_cast<
                    unsigned int
                >(
                    bt_stats.
                        output_buffer_bytes
                ),

                static_cast<
                    unsigned long
                >(
                    bt_stats.
                        callback_drops
                ),

                static_cast<
                    unsigned long
                >(
                    bt_stats.
                        block_drops
                ),

                static_cast<
                    unsigned long
                >(
                    bt_stats.
                        block_underflows
                ),

                static_cast<
                    unsigned long
                >(
                    bt_stats.
                        prebuffer_events
                ),

                max_input_peak,

                max_output_peak,

                max_gain_reduction,

                static_cast<
                    unsigned long long
                >(
                    limiter_frames
                ),

                static_cast<
                    unsigned long long
                >(
                    safety_clamps
                ),

                static_cast<
                    unsigned long long
                >(
                    tx_replacements
                ),

                static_cast<
                    unsigned long long
                >(
                    rx_underruns
                ),

                static_cast<
                    unsigned long long
                >(
                    plc_played
                ),

                static_cast<
                    unsigned long
                >(
                    heap_caps_get_free_size(
                        MALLOC_CAP_INTERNAL |
                        MALLOC_CAP_8BIT
                    )
                ),

                static_cast<
                    unsigned long
                >(
                    heap_caps_get_free_size(
                        MALLOC_CAP_DMA
                    )
                ),

                static_cast<
                    unsigned int
                >(
                    uxTaskGetStackHighWaterMark(
                        nullptr
                    )
                )
            );


            if (
                max_processing_us >
                10000
            ) {

                ESP_LOGW(
                    TAG,
                    "AUDIO DEADLINE MISSED"
                );
            }


            if (
                safety_clamps >
                0
            ) {

                ESP_LOGE(
                    TAG,
                    "OUTPUT SAFETY CLAMP"
                );
            }


            blocks =
                0;


            total_processing_us =
                0;


            max_processing_us =
                0;


            limiter_frames =
                0;


            safety_clamps =
                0;


            tx_replacements =
                0;


            rx_underruns =
                0;


            plc_played =
                0;


            bt_good_blocks =
                0;


            bt_silence_blocks =
                0;


            mic_speech_blocks =
                0;


            max_input_peak =
                -120.0f;


            max_output_peak =
                -120.0f;


            max_gain_reduction =
                0.0f;


            a2dp_sink.
                reset_interval_stats();
        }
    }
}


}


namespace h19 {


void run()
{
    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "Slipstream H19 - Integrated Bluetooth"
    );


    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "Bluetooth A2DP decoded internally"
    );


    ESP_LOGI(
        TAG,
        "Voice: full DSP / Opus / packet / jitter loopback"
    );


    ESP_LOGI(
        TAG,
        "Opus: 16k mono / 20ms / 20kbps / complexity %d",
        H19_OPUS_COMPLEXITY
    );


    ESP_LOGI(
        TAG,
        "BT normal / ducked: %.1f / %.1f dB",
        audio::PriorityMixer::
            BT_NORMAL_DB,
        audio::PriorityMixer::
            BT_DUCKED_DB
    );


    ESP_LOGI(
        TAG,
        "Jitter buffer: 40 ms"
    );


    if (
        !run_output_self_test()
    ) {

        ESP_LOGE(
            TAG,
            "H19 aborted: output self-test failed"
        );


        return;
    }


    /*
     * Reserve hardware DMA before Bluetooth/tasks.
     */

    if (
        !initialize_audio_hardware()
    ) {

        ESP_LOGE(
            TAG,
            "H19 aborted: audio hardware init failed"
        );


        return;
    }


    encode_queue =
        xQueueCreate(
            LATEST_QUEUE_DEPTH,
            sizeof(
                VoiceFrame
            )
        );


    network_queue =
        xQueueCreate(
            NETWORK_QUEUE_DEPTH,
            sizeof(
                NetworkPacket
            )
        );


    decoded_queue =
        xQueueCreate(
            LATEST_QUEUE_DEPTH,
            sizeof(
                VoiceFrame
            )
        );


    if (
        encode_queue ==
            nullptr ||
        network_queue ==
            nullptr ||
        decoded_queue ==
            nullptr
    ) {

        ESP_LOGE(
            TAG,
            "H19 queue creation failed"
        );


        return;
    }


    log_heap(
        "After queues"
    );


    if (
        !a2dp_sink.init()
    ) {

        ESP_LOGE(
            TAG,
            "Bluetooth initialization failed"
        );


        return;
    }


    log_heap(
        "After Bluetooth"
    );


    /*
     * --------------------------------------------------------
     * Task allocation order:
     *
     * encoder
     * audio
     * decoder
     * --------------------------------------------------------
     */

    BaseType_t result =
        xTaskCreatePinnedToCore(
            encode_task,
            "h19_encode",
            ENCODE_TASK_STACK,
            nullptr,
            ENCODE_TASK_PRIORITY,
            nullptr,
            0
        );


    if (
        result !=
        pdPASS
    ) {

        ESP_LOGE(
            TAG,
            "Encoder task creation failed"
        );


        return;
    }


    log_heap(
        "After encoder task"
    );


    result =
        xTaskCreatePinnedToCore(
            audio_task,
            "h19_audio",
            AUDIO_TASK_STACK,
            nullptr,
            AUDIO_TASK_PRIORITY,
            nullptr,
            1
        );


    if (
        result !=
        pdPASS
    ) {

        ESP_LOGE(
            TAG,
            "Audio task creation failed"
        );


        return;
    }


    log_heap(
        "After audio task"
    );


    result =
        xTaskCreatePinnedToCore(
            network_decode_task,
            "h19_rx",
            NETWORK_TASK_STACK,
            nullptr,
            NETWORK_TASK_PRIORITY,
            nullptr,
            1
        );


    if (
        result !=
        pdPASS
    ) {

        ESP_LOGE(
            TAG,
            "Network/decode task creation failed"
        );


        return;
    }


    log_heap(
        "After RX task"
    );


    ESP_LOGI(
        TAG,
        "H19 tasks created"
    );


    ESP_LOGI(
        TAG,
        "Pair phone with \"Slipstream\""
    );
}


}