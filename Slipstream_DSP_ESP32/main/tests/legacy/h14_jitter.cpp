#include "h14_jitter.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "audio/audio_io.hpp"
#include "audio/audio_processor.hpp"

#include "codec/opus_codec.hpp"
#include "codec/wm8960.hpp"

#include "transport/jitter_buffer.hpp"
#include "transport/simulated_rf.hpp"
#include "transport/slipstream_packet.hpp"

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"


namespace {

constexpr const char *TAG =
    "H14";


constexpr uint32_t AUDIO_TASK_STACK =
    24 * 1024;


constexpr uint32_t CODEC_TASK_STACK =
    40 * 1024;


constexpr UBaseType_t AUDIO_TASK_PRIORITY =
    6;


constexpr UBaseType_t CODEC_TASK_PRIORITY =
    5;


constexpr size_t FRAME_QUEUE_DEPTH =
    4;


/*
 * 3 complete Opus frames:
 *
 *     3 × 20 ms = 60 ms
 *
 * This is deliberately conservative for H14.
 */
constexpr uint32_t JITTER_DELAY_FRAMES =
    3;


constexpr uint8_t STREAM_ID =
    1;


constexpr uint8_t RF_CHANNEL =
    0;


struct VoiceFrame {

    int16_t samples[
        slipstream::OpusCodec::
            FRAME_SAMPLES
    ] = {};


    bool speech_active =
        false;


    bool plc =
        false;
};


QueueHandle_t encode_queue =
    nullptr;


QueueHandle_t decoded_queue =
    nullptr;


slipstream::OpusCodec opus_codec;


slipstream::transport::
    SimulatedRf simulated_rf;


slipstream::transport::
    PacketJitterBuffer jitter_buffer;


/*
 * Audio task ignores startup output starvation until
 * the jitter buffer begins playout.
 */
std::atomic<bool> receiver_started{
    false
};


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
 * Core 0
 *
 * TX codec
 * simulated RF
 * packet receiver
 * jitter buffer
 * Opus decode / PLC
 * ------------------------------------------------
 */

void codec_task(
    void *argument
)
{
    (void) argument;


    ESP_LOGI(
        TAG,
        "Codec/RF task running on CPU%d",
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

    simulated_rf.reset();

    jitter_buffer.reset();


    VoiceFrame input_frame = {};

    VoiceFrame output_frame = {};


    uint8_t opus_payload[
        slipstream::OpusCodec::
            MAX_PACKET_BYTES
    ] = {};


    uint8_t transmit_packet[
        slipstream::transport::
            MAX_PACKET_SIZE
    ] = {};


    uint8_t received_packet[
        slipstream::transport::
            MAX_PACKET_SIZE
    ] = {};


    uint8_t playout_packet[
        slipstream::transport::
            MAX_PACKET_SIZE
    ] = {};


    uint16_t tx_sequence =
        0;


    uint32_t tx_timestamp =
        0;


    uint16_t playout_sequence =
        0;


    uint32_t rf_tick =
        0;


    /*
     * -----------------------------------------
     * Stats.
     * -----------------------------------------
     */

    uint64_t frames =
        0;


    uint64_t normal_packets =
        0;

    uint64_t delayed_20 =
        0;

    uint64_t delayed_40 =
        0;

    uint64_t simulated_drops =
        0;

    uint64_t simulated_corruptions =
        0;

    uint64_t simulated_full =
        0;


    uint64_t crc_rejects =
        0;

    uint64_t late_packets =
        0;

    uint64_t duplicate_packets =
        0;

    uint64_t jitter_full =
        0;


    uint64_t decoded_packets =
        0;

    uint64_t plc_frames =
        0;

    uint64_t decoded_queue_drops =
        0;


    uint64_t reordered_arrivals =
        0;


    uint64_t total_encode_us =
        0;

    uint64_t total_decode_us =
        0;

    uint64_t total_plc_us =
        0;


    uint32_t max_encode_us =
        0;

    uint32_t max_decode_us =
        0;

    uint32_t max_plc_us =
        0;


    bool have_last_arrival =
        false;

    uint16_t last_arrival_sequence =
        0;


    int64_t last_report =
        esp_timer_get_time();


    while (true) {

        /*
         * One TX source frame represents one
         * 20 ms network clock tick.
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


        frames++;


        /*
         * -----------------------------------------
         * Encode.
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


        total_encode_us +=
            encode_us;


        if (
            encode_us >
            max_encode_us
        ) {

            max_encode_us =
                encode_us;
        }


        /*
         * -----------------------------------------
         * H13 packet.
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
            tx_sequence;


        header.timestamp =
            tx_timestamp;


        size_t transmit_length =
            0;


        bool built =
            slipstream::transport::
                build_packet(
                    header,
                    opus_payload,
                    static_cast<size_t>(
                        encoded_bytes
                    ),
                    transmit_packet,
                    sizeof(
                        transmit_packet
                    ),
                    transmit_length
                );


        if (!built) {

            ESP_LOGE(
                TAG,
                "Packet build failed"
            );

            continue;
        }


        /*
         * -----------------------------------------
         * Simulated RF transmission.
         * -----------------------------------------
         */

        auto rf_event =
            simulated_rf.send(
                transmit_packet,
                transmit_length,
                tx_sequence,
                rf_tick
            );


        switch (
            rf_event
        ) {

            case slipstream::transport::
                SimulatedRfEvent::NORMAL:

                normal_packets++;

                break;


            case slipstream::transport::
                SimulatedRfEvent::
                    DELAY_20_MS:

                delayed_20++;

                break;


            case slipstream::transport::
                SimulatedRfEvent::
                    DELAY_40_MS:

                delayed_40++;

                break;


            case slipstream::transport::
                SimulatedRfEvent::DROPPED:

                simulated_drops++;

                break;


            case slipstream::transport::
                SimulatedRfEvent::
                    CORRUPTED:

                simulated_corruptions++;

                break;


            case slipstream::transport::
                SimulatedRfEvent::FULL:

                simulated_full++;

                break;
        }


        tx_sequence++;


        tx_timestamp +=
            slipstream::OpusCodec::
                FRAME_SAMPLES;


        /*
         * -----------------------------------------
         * Receive every packet whose simulated
         * arrival time has now been reached.
         * -----------------------------------------
         */

        while (true) {

            size_t received_length =
                0;


            bool received =
                simulated_rf.receive(
                    rf_tick,
                    received_packet,
                    sizeof(
                        received_packet
                    ),
                    received_length
                );


            if (!received) {

                break;
            }


            slipstream::transport::
                ParsedPacket parsed = {};


            auto parse_result =
                slipstream::transport::
                    parse_packet(
                        received_packet,
                        received_length,
                        parsed
                    );


            /*
             * Corrupted RF packet disappears here.
             */
            if (
                parse_result ==
                slipstream::transport::
                    ParseResult::BAD_CRC
            ) {

                crc_rejects++;

                continue;
            }


            if (
                parse_result !=
                slipstream::transport::
                    ParseResult::OK
            ) {

                continue;
            }


            /*
             * Detect out-of-order physical arrival.
             */
            if (
                have_last_arrival
            ) {

                if (
                    static_cast<int16_t>(
                        parsed.header.sequence -
                        last_arrival_sequence
                    )
                    <
                    0
                ) {

                    reordered_arrivals++;
                }
            }


            last_arrival_sequence =
                parsed.header.sequence;


            have_last_arrival =
                true;


            /*
             * Put valid packet into jitter buffer.
             */
            auto insert_result =
                jitter_buffer.insert(
                    received_packet,
                    received_length,
                    parsed.header.sequence
                );


            switch (
                insert_result
            ) {

                case slipstream::transport::
                    JitterInsertResult::OK:

                    break;


                case slipstream::transport::
                    JitterInsertResult::LATE:

                    late_packets++;

                    break;


                case slipstream::transport::
                    JitterInsertResult::
                        DUPLICATE:

                    duplicate_packets++;

                    break;


                case slipstream::transport::
                    JitterInsertResult::FULL:

                    jitter_full++;

                    break;


                case slipstream::transport::
                    JitterInsertResult::
                        TOO_LARGE:

                    jitter_full++;

                    break;
            }
        }


        /*
         * -----------------------------------------
         * Receiver playout.
         *
         * Wait 3 frame periods before requesting
         * sequence 0:
         *
         *     3 × 20 ms = 60 ms jitter delay.
         * -----------------------------------------
         */

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


            bool packet_available =
                jitter_buffer.take(
                    playout_sequence,
                    playout_packet,
                    sizeof(
                        playout_packet
                    ),
                    playout_length
                );


            uint32_t decode_us =
                0;


            if (
                packet_available
            ) {

                slipstream::transport::
                    ParsedPacket parsed = {};


                auto result =
                    slipstream::transport::
                        parse_packet(
                            playout_packet,
                            playout_length,
                            parsed
                        );


                if (
                    result ==
                    slipstream::transport::
                        ParseResult::OK
                ) {

                    int samples =
                        opus_codec.decode_frame(
                            parsed.payload,
                            parsed.payload_length,
                            output_frame.samples,
                            decode_us
                        );


                    if (
                        samples ==
                        slipstream::OpusCodec::
                            FRAME_SAMPLES
                    ) {

                        decoded_packets++;


                        output_frame.plc =
                            false;


                        total_decode_us +=
                            decode_us;


                        if (
                            decode_us >
                            max_decode_us
                        ) {

                            max_decode_us =
                                decode_us;
                        }
                    }
                    else {

                        packet_available =
                            false;
                    }
                }
                else {

                    packet_available =
                        false;
                }
            }


            /*
             * -----------------------------------------
             * Missing packet -> Opus PLC.
             * -----------------------------------------
             */

            if (
                !packet_available
            ) {

                int samples =
                    opus_codec.decode_plc(
                        output_frame.samples,
                        decode_us
                    );


                output_frame.plc =
                    true;


                plc_frames++;


                total_plc_us +=
                    decode_us;


                if (
                    decode_us >
                    max_plc_us
                ) {

                    max_plc_us =
                        decode_us;
                }


                if (
                    samples !=
                    slipstream::OpusCodec::
                        FRAME_SAMPLES
                ) {

                    ESP_LOGE(
                        TAG,
                        "PLC returned %d samples",
                        samples
                    );
                }
            }


            /*
             * Playback always gets one 20 ms frame,
             * whether real or concealed.
             */
            if (
                xQueueSend(
                    decoded_queue,
                    &output_frame,
                    0
                )
                !=
                pdTRUE
            ) {

                decoded_queue_drops++;
            }


            playout_sequence++;
        }


        /*
         * Next simulated 20 ms instant.
         */
        rf_tick++;


        /*
         * -----------------------------------------
         * Report ~1 second.
         * -----------------------------------------
         */

        int64_t now =
            esp_timer_get_time();


        if (
            now -
            last_report >=
            1000000
        ) {

            uint64_t avg_encode =
                frames > 0
                ?
                total_encode_us /
                    frames
                :
                0;


            uint64_t avg_decode =
                decoded_packets > 0
                ?
                total_decode_us /
                    decoded_packets
                :
                0;


            uint64_t avg_plc =
                plc_frames > 0
                ?
                total_plc_us /
                    plc_frames
                :
                0;


            ESP_LOGI(
                TAG,

                "RF | "
                "frames=%llu | "
                "ENC=%llu/%lu us | "
                "normal=%llu | "
                "d20=%llu | "
                "d40=%llu | "
                "drop=%llu | "
                "corrupt=%llu | "
                "crc=%llu | "
                "reorder=%llu | "
                "late=%llu | "
                "jbuf=%u | "
                "decoded=%llu | "
                "PLC=%llu | "
                "DEC=%llu/%lu us | "
                "PLCtime=%llu/%lu us | "
                "qTX=%u | "
                "qRX=%u",

                static_cast<
                    unsigned long long
                >(
                    frames
                ),

                static_cast<
                    unsigned long long
                >(
                    avg_encode
                ),

                static_cast<
                    unsigned long
                >(
                    max_encode_us
                ),

                static_cast<
                    unsigned long long
                >(
                    normal_packets
                ),

                static_cast<
                    unsigned long long
                >(
                    delayed_20
                ),

                static_cast<
                    unsigned long long
                >(
                    delayed_40
                ),

                static_cast<
                    unsigned long long
                >(
                    simulated_drops
                ),

                static_cast<
                    unsigned long long
                >(
                    simulated_corruptions
                ),

                static_cast<
                    unsigned long long
                >(
                    crc_rejects
                ),

                static_cast<
                    unsigned long long
                >(
                    reordered_arrivals
                ),

                static_cast<
                    unsigned long long
                >(
                    late_packets
                ),

                static_cast<
                    unsigned int
                >(
                    jitter_buffer.size()
                ),

                static_cast<
                    unsigned long long
                >(
                    decoded_packets
                ),

                static_cast<
                    unsigned long long
                >(
                    plc_frames
                ),

                static_cast<
                    unsigned long long
                >(
                    avg_decode
                ),

                static_cast<
                    unsigned long
                >(
                    max_decode_us
                ),

                static_cast<
                    unsigned long long
                >(
                    avg_plc
                ),

                static_cast<
                    unsigned long
                >(
                    max_plc_us
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


            frames =
                0;


            normal_packets =
                0;

            delayed_20 =
                0;

            delayed_40 =
                0;

            simulated_drops =
                0;

            simulated_corruptions =
                0;

            simulated_full =
                0;


            crc_rejects =
                0;

            late_packets =
                0;

            duplicate_packets =
                0;

            jitter_full =
                0;

            reordered_arrivals =
                0;


            decoded_packets =
                0;

            plc_frames =
                0;

            decoded_queue_drops =
                0;


            total_encode_us =
                0;

            total_decode_us =
                0;

            total_plc_us =
                0;


            max_encode_us =
                0;

            max_decode_us =
                0;

            max_plc_us =
                0;


            last_report =
                now;
        }
    }
}


/*
 * ------------------------------------------------
 * Core 1 real-time audio task.
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

    bool have_playback =
        false;

    size_t playback_half =
        0;


    uint64_t blocks =
        0;


    uint64_t total_processing_us =
        0;


    uint32_t max_processing_us =
        0;


    uint64_t tx_drops =
        0;


    uint64_t playback_underruns =
        0;


    uint64_t plc_played =
        0;


    ESP_LOGI(
        TAG,
        "H14 simulated RF loopback starting"
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
         * Build next 20 ms TX frame.
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


            tx_frame =
                {};


            tx_half =
                0;
        }


        /*
         * -----------------------------------------
         * RX playback.
         * -----------------------------------------
         */

        const int16_t *playback_voice =
            silence_block;


        if (
            !have_playback ||
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

                have_playback =
                    true;


                playback_half =
                    0;


                if (
                    playback_frame.plc
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
                    receiver_started.load()
                ) {

                    playback_underruns++;
                }
            }
        }


        if (
            have_playback
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


        blocks++;


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
            blocks %
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
                "PLCplayed=%llu | "
                "qTX=%u | "
                "qRX=%u",

                static_cast<
                    unsigned long long
                >(
                    blocks
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
                    playback_underruns
                ),

                static_cast<
                    unsigned long long
                >(
                    plc_played
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


            playback_underruns =
                0;


            plc_played =
                0;
        }
    }
}

}


namespace h14 {

void run()
{
    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "Slipstream H14 - Jitter + PLC"
    );


    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "Simulated RF enabled"
    );


    ESP_LOGI(
        TAG,
        "Jitter buffer: %lu frames / %lu ms",
        static_cast<unsigned long>(
            JITTER_DELAY_FRAMES
        ),
        static_cast<unsigned long>(
            JITTER_DELAY_FRAMES *
            20
        )
    );


    ESP_LOGI(
        TAG,
        "RF impairments: drop + corruption + "
        "20/40 ms jitter + reordering"
    );


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
            "h14_codec",
            CODEC_TASK_STACK,
            nullptr,
            CODEC_TASK_PRIORITY,
            nullptr,
            0
        );


    BaseType_t audio_result =
        xTaskCreatePinnedToCore(
            audio_task,
            "h14_audio",
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
        "H14 tasks created"
    );
}

}