#include "h13_packet.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "audio/audio_io.hpp"
#include "audio/audio_processor.hpp"

#include "codec/opus_codec.hpp"
#include "codec/wm8960.hpp"

#include "transport/slipstream_packet.hpp"

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"


namespace {

constexpr const char *TAG =
    "H13";


constexpr uint32_t AUDIO_TASK_STACK =
    24 * 1024;

constexpr uint32_t CODEC_TASK_STACK =
    40 * 1024;


constexpr UBaseType_t AUDIO_TASK_PRIORITY =
    6;

constexpr UBaseType_t CODEC_TASK_PRIORITY =
    5;


constexpr size_t FRAME_QUEUE_DEPTH =
    3;


/*
 * ------------------------------------------------
 * H13 packet identity.
 * ------------------------------------------------
 */

constexpr uint8_t STREAM_ID =
    1;

constexpr uint8_t RF_CHANNEL =
    0;


/*
 * ------------------------------------------------
 * 20 ms PCM frame.
 * ------------------------------------------------
 */

struct VoiceFrame {

    int16_t samples[
        slipstream::OpusCodec::
            FRAME_SAMPLES
    ];

    bool speech_active =
        false;
};


QueueHandle_t encode_queue =
    nullptr;

QueueHandle_t decoded_queue =
    nullptr;


slipstream::OpusCodec opus_codec;


/*
 * ------------------------------------------------
 * Hardware/audio buffers.
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
 * H13 startup CRC sanity test.
 * ------------------------------------------------
 */

bool run_crc_test()
{
    ESP_LOGI(
        TAG,
        "Running H13 CRC corruption test"
    );


    const uint8_t payload[] = {
        0x11,
        0x22,
        0x33,
        0x44,
        0x55
    };


    uint8_t packet[
        slipstream::transport::
            MAX_PACKET_SIZE
    ] = {};


    slipstream::transport::
        PacketHeader header = {};


    header.flags =
        slipstream::transport::
            FLAG_SPEECH;

    header.stream_id =
        STREAM_ID;

    header.channel =
        RF_CHANNEL;

    header.sequence =
        1234;

    header.timestamp =
        567890;


    size_t packet_length =
        0;


    bool built =
        slipstream::transport::
            build_packet(
                header,
                payload,
                sizeof(
                    payload
                ),
                packet,
                sizeof(
                    packet
                ),
                packet_length
            );


    if (!built) {

        ESP_LOGE(
            TAG,
            "CRC self-test packet build failed"
        );


        return false;
    }


    slipstream::transport::
        ParsedPacket parsed = {};


    auto result =
        slipstream::transport::
            parse_packet(
                packet,
                packet_length,
                parsed
            );


    if (
        result !=
        slipstream::transport::
            ParseResult::OK
    ) {

        ESP_LOGE(
            TAG,
            "Valid packet failed parse: %s",
            slipstream::transport::
                parse_result_string(
                    result
                )
        );


        return false;
    }


    /*
     * Corrupt one payload byte.
     */
    packet[
        slipstream::transport::
            HEADER_SIZE +
        2
    ] ^=
        0x01;


    result =
        slipstream::transport::
            parse_packet(
                packet,
                packet_length,
                parsed
            );


    if (
        result !=
        slipstream::transport::
            ParseResult::BAD_CRC
    ) {

        ESP_LOGE(
            TAG,
            "Corrupted packet was NOT rejected"
        );


        return false;
    }


    ESP_LOGI(
        TAG,
        "CRC corruption test: PASS"
    );


    return true;
}


/*
 * ------------------------------------------------
 * Core 0: Opus + H13 packet processing.
 * ------------------------------------------------
 */

void codec_task(
    void *argument
)
{
    (void) argument;


    ESP_LOGI(
        TAG,
        "Codec/packet task running on CPU%d",
        xPortGetCoreID()
    );


    if (
        !opus_codec.init()
    ) {

        ESP_LOGE(
            TAG,
            "Opus initialization failed"
        );


        vTaskDelete(
            nullptr
        );


        return;
    }


    opus_codec.reset();


    VoiceFrame input_frame = {};

    VoiceFrame decoded_frame = {};


    uint8_t opus_payload[
        slipstream::OpusCodec::
            MAX_PACKET_BYTES
    ] = {};


    uint8_t packet_buffer[
        slipstream::transport::
            MAX_PACKET_SIZE
    ] = {};


    uint16_t sequence =
        0;


    uint32_t timestamp =
        0;


    /*
     * Statistics.
     */

    uint64_t frame_count =
        0;

    uint64_t total_encode_us =
        0;

    uint64_t total_packet_build_us =
        0;

    uint64_t total_packet_parse_us =
        0;

    uint64_t total_decode_us =
        0;

    uint64_t total_payload_bytes =
        0;

    uint64_t total_wire_bytes =
        0;


    uint64_t packet_failures =
        0;

    uint64_t sequence_failures =
        0;

    uint64_t decoded_drops =
        0;


    uint32_t max_encode_us =
        0;

    uint32_t max_build_us =
        0;

    uint32_t max_parse_us =
        0;

    uint32_t max_decode_us =
        0;


    uint16_t expected_sequence =
        0;

    bool have_expected_sequence =
        false;


    int64_t last_report_us =
        esp_timer_get_time();


    while (true) {

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


        /*
         * -----------------------------------------
         * Opus encode.
         * -----------------------------------------
         */

        uint32_t encode_us =
            0;


        int encoded_bytes =
            opus_codec.encode_frame(
                input_frame.samples,
                opus_payload,
                sizeof(
                    opus_payload
                ),
                encode_us
            );


        if (
            encoded_bytes < 0
        ) {

            ESP_LOGE(
                TAG,
                "Opus encode failed"
            );


            continue;
        }


        /*
         * -----------------------------------------
         * Build H13 packet.
         * -----------------------------------------
         */

        slipstream::transport::
            PacketHeader header = {};


        header.flags =
            input_frame.speech_active
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


        size_t packet_length =
            0;


        int64_t build_start =
            esp_timer_get_time();


        bool built =
            slipstream::transport::
                build_packet(
                    header,
                    opus_payload,
                    static_cast<size_t>(
                        encoded_bytes
                    ),
                    packet_buffer,
                    sizeof(
                        packet_buffer
                    ),
                    packet_length
                );


        uint32_t build_us =
            static_cast<uint32_t>(
                esp_timer_get_time() -
                build_start
            );


        if (!built) {

            ESP_LOGE(
                TAG,
                "Packet build failed"
            );


            packet_failures++;


            continue;
        }


        /*
         * Advance transmitter counters for the
         * NEXT packet.
         *
         * 320 samples = 20 ms @ 16 kHz.
         */
        sequence++;


        timestamp +=
            slipstream::OpusCodec::
                FRAME_SAMPLES;


        /*
         * -----------------------------------------
         * Parse + CRC verify.
         *
         * This represents the future receiver.
         * -----------------------------------------
         */

        slipstream::transport::
            ParsedPacket parsed = {};


        int64_t parse_start =
            esp_timer_get_time();


        auto parse_result =
            slipstream::transport::
                parse_packet(
                    packet_buffer,
                    packet_length,
                    parsed
                );


        uint32_t parse_us =
            static_cast<uint32_t>(
                esp_timer_get_time() -
                parse_start
            );


        if (
            parse_result !=
            slipstream::transport::
                ParseResult::OK
        ) {

            ESP_LOGE(
                TAG,
                "Packet parse failed: %s",
                slipstream::transport::
                    parse_result_string(
                        parse_result
                    )
            );


            packet_failures++;


            continue;
        }


        /*
         * -----------------------------------------
         * Sequence validation.
         * -----------------------------------------
         */

        if (
            have_expected_sequence
        ) {

            if (
                parsed.header.sequence !=
                expected_sequence
            ) {

                sequence_failures++;
            }
        }


        expected_sequence =
            static_cast<uint16_t>(
                parsed.header.sequence +
                1
            );


        have_expected_sequence =
            true;


        /*
         * -----------------------------------------
         * Opus decode FROM PARSED PAYLOAD.
         * -----------------------------------------
         */

        uint32_t decode_us =
            0;


        int decoded_samples =
            opus_codec.decode_frame(
                parsed.payload,
                parsed.payload_length,
                decoded_frame.samples,
                decode_us
            );


        if (
            decoded_samples !=
            slipstream::OpusCodec::
                FRAME_SAMPLES
        ) {

            ESP_LOGE(
                TAG,
                "Unexpected decode size: %d",
                decoded_samples
            );


            continue;
        }


        /*
         * Preserve speech flag for diagnostics.
         */
        decoded_frame.speech_active =
            (
                parsed.header.flags &
                slipstream::transport::
                    FLAG_SPEECH
            )
            != 0;


        /*
         * -----------------------------------------
         * Queue decoded PCM.
         * -----------------------------------------
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

            decoded_drops++;
        }


        /*
         * -----------------------------------------
         * Statistics.
         * -----------------------------------------
         */

        frame_count++;


        total_encode_us +=
            encode_us;


        total_packet_build_us +=
            build_us;


        total_packet_parse_us +=
            parse_us;


        total_decode_us +=
            decode_us;


        total_payload_bytes +=
            static_cast<uint64_t>(
                encoded_bytes
            );


        total_wire_bytes +=
            static_cast<uint64_t>(
                packet_length
            );


        if (
            encode_us >
            max_encode_us
        ) {
            max_encode_us =
                encode_us;
        }


        if (
            build_us >
            max_build_us
        ) {
            max_build_us =
                build_us;
        }


        if (
            parse_us >
            max_parse_us
        ) {
            max_parse_us =
                parse_us;
        }


        if (
            decode_us >
            max_decode_us
        ) {
            max_decode_us =
                decode_us;
        }


        /*
         * -----------------------------------------
         * Approximately 1 second.
         * -----------------------------------------
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

                ESP_LOGI(
                    TAG,

                    "PKT | "
                    "frames=%llu | "
                    "ENC=%llu/%lu us | "
                    "BUILD=%llu/%lu us | "
                    "PARSE=%llu/%lu us | "
                    "DEC=%llu/%lu us | "
                    "opus=%llu B | "
                    "wire=%llu B | "
                    "overhead=%lld B | "
                    "pktfail=%llu | "
                    "seqfail=%llu | "
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
                        total_encode_us /
                        frame_count
                    ),

                    static_cast<
                        unsigned long
                    >(
                        max_encode_us
                    ),

                    static_cast<
                        unsigned long long
                    >(
                        total_packet_build_us /
                        frame_count
                    ),

                    static_cast<
                        unsigned long
                    >(
                        max_build_us
                    ),

                    static_cast<
                        unsigned long long
                    >(
                        total_packet_parse_us /
                        frame_count
                    ),

                    static_cast<
                        unsigned long
                    >(
                        max_parse_us
                    ),

                    static_cast<
                        unsigned long long
                    >(
                        total_decode_us /
                        frame_count
                    ),

                    static_cast<
                        unsigned long
                    >(
                        max_decode_us
                    ),

                    static_cast<
                        unsigned long long
                    >(
                        total_payload_bytes /
                        frame_count
                    ),

                    static_cast<
                        unsigned long long
                    >(
                        total_wire_bytes /
                        frame_count
                    ),

                    static_cast<
                        long long
                    >(
                        (
                            total_wire_bytes /
                            frame_count
                        )
                        -
                        (
                            total_payload_bytes /
                            frame_count
                        )
                    ),

                    static_cast<
                        unsigned long long
                    >(
                        packet_failures
                    ),

                    static_cast<
                        unsigned long long
                    >(
                        sequence_failures
                    ),

                    static_cast<
                        unsigned long long
                    >(
                        decoded_drops
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


            frame_count =
                0;


            total_encode_us =
                0;

            total_packet_build_us =
                0;

            total_packet_parse_us =
                0;

            total_decode_us =
                0;

            total_payload_bytes =
                0;

            total_wire_bytes =
                0;


            packet_failures =
                0;

            sequence_failures =
                0;

            decoded_drops =
                0;


            max_encode_us =
                0;

            max_build_us =
                0;

            max_parse_us =
                0;

            max_decode_us =
                0;


            last_report_us =
                now;
        }
    }
}


/*
 * ------------------------------------------------
 * Core 1 audio task.
 * ------------------------------------------------
 */

void audio_task(
    void *argument
)
{
    (void) argument;


    ESP_LOGI(
        TAG,
        "Audio task running on CPU%d",
        xPortGetCoreID()
    );


    ESP_ERROR_CHECK(
        wm8960::init_i2c()
    );


    if (
        !wm8960::probe()
    ) {

        ESP_LOGE(
            TAG,
            "WM8960 not detected"
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


    VoiceFrame tx_frame = {};


    size_t tx_half =
        0;


    VoiceFrame playback_frame = {};


    bool have_playback_frame =
        false;


    size_t playback_half =
        0;


    uint64_t block_count =
        0;

    uint64_t total_processing_us =
        0;

    uint32_t max_processing_us =
        0;

    uint64_t tx_drops =
        0;

    uint64_t underruns =
        0;


    bool noise_ready_reported =
        false;


    ESP_LOGI(
        TAG,
        "H13 live packet loopback starting"
    );


    ESP_LOGI(
        TAG,
        "Keep quiet for first ~500 ms"
    );


    while (true) {

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


        int64_t start =
            esp_timer_get_time();


        audio_processor::Stats
            stats = {};


        audio_processor::
            process_input_block(
                hardware_input,
                voice_block,
                stats
            );


        /*
         * -----------------------------------------
         * Build 20 ms PCM frame.
         * -----------------------------------------
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


        /*
         * If either half contains speech,
         * mark the complete 20 ms frame as speech.
         */
        if (
            stats.speech_active
        ) {

            tx_frame.speech_active =
                true;
        }


        tx_half++;


        if (
            tx_half >= 2
        ) {

            if (
                xQueueSend(
                    encode_queue,
                    &tx_frame,
                    0
                )
                !=
                pdTRUE
            ) {

                tx_drops++;
            }


            tx_half =
                0;


            tx_frame.speech_active =
                false;
        }


        /*
         * -----------------------------------------
         * Playback decoded packets.
         * -----------------------------------------
         */

        const int16_t *playback_voice =
            silence_block;


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


                if (
                    noise_ready_reported
                ) {

                    underruns++;
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


        audio_processor::
            render_voice_block(
                playback_voice,
                hardware_output,
                stats
            );


        uint32_t processing_us =
            static_cast<uint32_t>(
                esp_timer_get_time() -
                start
            );


        ESP_ERROR_CHECK(
            audio::write(
                hardware_output,
                sizeof(
                    hardware_output
                )
            )
        );


        block_count++;


        total_processing_us +=
            processing_us;


        if (
            processing_us >
            max_processing_us
        ) {

            max_processing_us =
                processing_us;
        }


        if (
            !noise_ready_reported &&
            audio_processor::
                noise_estimate_ready()
        ) {

            noise_ready_reported =
                true;


            ESP_LOGI(
                TAG,
                "Noise estimate ready"
            );
        }


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

                stats.speech_active
                    ? "YES"
                    : "NO",

                static_cast<
                    unsigned long long
                >(
                    tx_drops
                ),

                static_cast<
                    unsigned long long
                >(
                    underruns
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


            if (
                max_processing_us >
                10000
            ) {

                ESP_LOGW(
                    TAG,
                    "AUDIO DEADLINE MISSED"
                );
            }


            total_processing_us =
                0;

            max_processing_us =
                0;

            tx_drops =
                0;

            underruns =
                0;
        }
    }
}

}


namespace h13 {

void run()
{
    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "Slipstream H13 - Packet + CRC32"
    );


    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "Header: 16 bytes"
    );


    ESP_LOGI(
        TAG,
        "CRC32: 4 bytes"
    );


    ESP_LOGI(
        TAG,
        "Packet overhead: 20 bytes"
    );


    if (
        !run_crc_test()
    ) {

        ESP_LOGE(
            TAG,
            "H13 FAIL: CRC self-test failed"
        );


        return;
    }


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
            "Queue creation failed"
        );


        return;
    }


    BaseType_t codec_result =
        xTaskCreatePinnedToCore(
            codec_task,
            "h13_codec",
            CODEC_TASK_STACK,
            nullptr,
            CODEC_TASK_PRIORITY,
            nullptr,
            0
        );


    BaseType_t audio_result =
        xTaskCreatePinnedToCore(
            audio_task,
            "h13_audio",
            AUDIO_TASK_STACK,
            nullptr,
            AUDIO_TASK_PRIORITY,
            nullptr,
            1
        );


    if (
        codec_result !=
            pdPASS ||
        audio_result !=
            pdPASS
    ) {

        ESP_LOGE(
            TAG,
            "Task creation failed"
        );


        return;
    }


    ESP_LOGI(
        TAG,
        "H13 tasks created"
    );
}

}