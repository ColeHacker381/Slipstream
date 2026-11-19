#include "h17_rx_output.hpp"

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "audio/audio_io.hpp"
#include "audio/audio_processor.hpp"
#include "audio/priority_mixer.hpp"
#include "audio/rx_output_stage.hpp"

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
    "H17";


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
 * Frozen H15 result.
 */
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
    HARDWARE_FRAMES *
    2;


/*
 * Same synthetic H16 Bluetooth source.
 */
constexpr float BT_TEST_AMPLITUDE =
    5000.0f;


constexpr float BT_LEFT_HZ =
    440.0f;


constexpr float BT_RIGHT_HZ =
    660.0f;


constexpr float PI =
    3.14159265358979323846f;


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


audio::PriorityMixer priority_mixer;


audio::RxOutputStage rx_output_stage;


std::atomic<bool> receiver_started{
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


int16_t silence_block[
    audio_processor::
        VOICE_SAMPLES_PER_BLOCK
] = {};


/*
 * ============================================================
 * Synthetic Bluetooth source
 * ============================================================
 */

void generate_bt_test_audio()
{
    static float left_phase =
        0.0f;


    static float right_phase =
        0.0f;


    const float left_step =
        2.0f *
        PI *
        BT_LEFT_HZ /
        48000.0f;


    const float right_step =
        2.0f *
        PI *
        BT_RIGHT_HZ /
        48000.0f;


    for (
        size_t frame = 0;
        frame < HARDWARE_FRAMES;
        ++frame
    ) {

        bt_hardware[
            frame * 2
        ] =
            static_cast<int16_t>(
                std::sin(
                    left_phase
                )
                *
                BT_TEST_AMPLITUDE
            );


        bt_hardware[
            frame * 2 + 1
        ] =
            static_cast<int16_t>(
                std::sin(
                    right_phase
                )
                *
                BT_TEST_AMPLITUDE
            );


        left_phase +=
            left_step;


        right_phase +=
            right_step;


        if (
            left_phase >=
            2.0f * PI
        ) {

            left_phase -=
                2.0f * PI;
        }


        if (
            right_phase >=
            2.0f * PI
        ) {

            right_phase -=
                2.0f * PI;
        }
    }
}


/*
 * ============================================================
 * H17 limiter self-test
 * ============================================================
 */

bool run_output_self_test()
{
    constexpr size_t TEST_FRAMES =
        32;


    float test_input[
        TEST_FRAMES *
        2
    ] = {};


    int16_t test_output[
        TEST_FRAMES *
        2
    ] = {};


    for (
        size_t frame = 0;
        frame < TEST_FRAMES;
        ++frame
    ) {

        /*
         * Deliberately impossible int16 amplitude.
         *
         * About +3.7 dBFS.
         */
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


    const bool ceiling_ok =
        stats.output_peak_dbfs <=
        (
            audio::RxOutputStage::
                LIMITER_CEILING_DBFS
            +
            0.05f
        );


    const bool clamps_ok =
        stats.safety_clamps ==
        0;


    rx_output_stage.reset();


    if (
        !ceiling_ok ||
        !clamps_ok
    ) {

        ESP_LOGE(
            TAG,
            "RX limiter self-test: FAIL"
        );


        return false;
    }


    ESP_LOGI(
        TAG,
        "RX limiter self-test: PASS"
    );


    return true;
}


/*
 * ============================================================
 * Core 0 — frozen H15/H16 network path
 * ============================================================
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


    uint64_t frames =
        0;


    uint64_t plc_frames =
        0;


    uint64_t misses =
        0;


    uint64_t crc_rejects =
        0;


    int64_t last_report =
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


        frames++;


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
            encoded_bytes <
            0
        ) {

            continue;
        }


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


        if (
            !slipstream::transport::
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
                )
        ) {

            continue;
        }


        simulated_rf.send(
            transmit_packet,
            transmit_length,
            tx_sequence,
            rf_tick
        );


        tx_sequence++;


        tx_timestamp +=
            slipstream::OpusCodec::
                FRAME_SAMPLES;


        while (true) {

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


            auto result =
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


            bool decoded_ok =
                false;


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

                        decoded_ok =
                            true;


                        output_frame.plc =
                            false;


                        output_frame.speech_active =
                            (
                                parsed.header.flags &
                                slipstream::transport::
                                    FLAG_SPEECH
                            )
                            !=
                            0;
                    }
                }
            }


            if (
                !decoded_ok
            ) {

                misses++;


                int samples =
                    opus_codec.decode_plc(
                        output_frame.samples,
                        decode_us
                    );


                if (
                    samples ==
                    slipstream::OpusCodec::
                        FRAME_SAMPLES
                ) {

                    output_frame.plc =
                        true;


                    plc_frames++;
                }
            }


            xQueueSend(
                decoded_queue,
                &output_frame,
                0
            );


            playout_sequence++;
        }


        rf_tick++;


        int64_t now =
            esp_timer_get_time();


        if (
            now -
            last_report >=
            1000000
        ) {

            ESP_LOGI(
                TAG,
                "NET | "
                "frames=%llu | "
                "miss=%llu | "
                "PLC=%llu | "
                "crc=%llu | "
                "jbuf=%u | "
                "qTX=%u | "
                "qRX=%u",
                static_cast<unsigned long long>(
                    frames
                ),
                static_cast<unsigned long long>(
                    misses
                ),
                static_cast<unsigned long long>(
                    plc_frames
                ),
                static_cast<unsigned long long>(
                    crc_rejects
                ),
                static_cast<unsigned int>(
                    jitter_buffer.size()
                ),
                static_cast<unsigned int>(
                    uxQueueMessagesWaiting(
                        encode_queue
                    )
                ),
                static_cast<unsigned int>(
                    uxQueueMessagesWaiting(
                        decoded_queue
                    )
                )
            );


            frames =
                0;


            misses =
                0;


            plc_frames =
                0;


            crc_rejects =
                0;


            last_report =
                now;
        }
    }
}


/*
 * ============================================================
 * Core 1 — audio / mixer / H17 output
 * ============================================================
 */

void audio_task(
    void *argument
)
{
    (void) argument;


    ESP_LOGI(
        TAG,
        "Audio/output task running on CPU%d",
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


    priority_mixer.reset();


    rx_output_stage.reset();


    rx_output_stage.set_eq_enabled(
        false
    );


    rx_output_stage.set_master_gain_db(
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


    uint64_t tx_drops =
        0;


    uint64_t underruns =
        0;


    uint64_t plc_played =
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


    ESP_LOGI(
        TAG,
        "H17 RX output stage starting"
    );


    ESP_LOGI(
        TAG,
        "EQ: BYPASS"
    );


    ESP_LOGI(
        TAG,
        "Master gain: %.1f dB",
        rx_output_stage.
            master_gain_db()
    );


    ESP_LOGI(
        TAG,
        "Limiter ceiling: %.1f dBFS",
        audio::RxOutputStage::
            LIMITER_CEILING_DBFS
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


        const int64_t start =
            esp_timer_get_time();


        audio_processor::Stats
            processor_stats = {};


        /*
         * TX DSP.
         */
        audio_processor::
            process_input_block(
                hardware_input,
                voice_block,
                processor_stats
            );


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
            processor_stats.speech_active
        ) {

            tx_frame.speech_active =
                true;
        }


        tx_half++;


        if (
            tx_half >=
            2
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
         * RX PCM.
         */
        const int16_t *rf_voice =
            silence_block;


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

                    underruns++;
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


        /*
         * 16 kHz RF -> 48 kHz stereo.
         */
        audio_processor::
            render_voice_block(
                rf_voice,
                rf_hardware,
                processor_stats
            );


        /*
         * Synthetic BT.
         */
        generate_bt_test_audio();


        /*
         * H16 mixer, now WITHOUT premature int16
         * saturation.
         */
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


        /*
         * H17 final output processing.
         */
        audio::RxOutputStats
            output_stats = {};


        rx_output_stage.process_block(
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
                esp_timer_get_time() -
                start
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


        limiter_frames +=
            output_stats.limited_frames;


        safety_clamps +=
            output_stats.safety_clamps;


        if (
            output_stats.input_peak_dbfs >
            max_input_peak
        ) {

            max_input_peak =
                output_stats.input_peak_dbfs;
        }


        if (
            output_stats.output_peak_dbfs >
            max_output_peak
        ) {

            max_output_peak =
                output_stats.output_peak_dbfs;
        }


        if (
            output_stats.gain_reduction_db >
            max_gain_reduction
        ) {

            max_gain_reduction =
                output_stats.
                    gain_reduction_db;
        }


        if (
            blocks %
            100 ==
            0
        ) {

            ESP_LOGI(
                TAG,
                "OUT | "
                "blocks=%llu | "
                "proc=%llu/%lu us | "
                "RF=%s | "
                "duck=%s | "
                "BT=%.2f dB | "
                "INpk=%.2f dBFS | "
                "OUTpk=%.2f dBFS | "
                "GR=%.2f dB | "
                "limited=%llu | "
                "clamps=%llu | "
                "txdrop=%llu | "
                "underrun=%llu | "
                "PLC=%llu",
                static_cast<unsigned long long>(
                    blocks
                ),
                static_cast<unsigned long long>(
                    total_processing_us /
                    100
                ),
                static_cast<unsigned long>(
                    max_processing_us
                ),
                rf_speech
                    ? "YES"
                    : "NO",
                priority_mixer.ducking()
                    ? "YES"
                    : "NO",
                priority_mixer.
                    current_bt_gain_db(),
                max_input_peak,
                max_output_peak,
                max_gain_reduction,
                static_cast<unsigned long long>(
                    limiter_frames
                ),
                static_cast<unsigned long long>(
                    safety_clamps
                ),
                static_cast<unsigned long long>(
                    tx_drops
                ),
                static_cast<unsigned long long>(
                    underruns
                ),
                static_cast<unsigned long long>(
                    plc_played
                )
            );


            if (
                max_output_peak >
                (
                    audio::RxOutputStage::
                        LIMITER_CEILING_DBFS
                    +
                    0.05f
                )
            ) {

                ESP_LOGE(
                    TAG,
                    "RX LIMITER CEILING VIOLATION"
                );
            }


            if (
                safety_clamps >
                0
            ) {

                ESP_LOGE(
                    TAG,
                    "RX OUTPUT SAFETY CLAMP OCCURRED"
                );
            }


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


            plc_played =
                0;


            limiter_frames =
                0;


            safety_clamps =
                0;


            max_input_peak =
                -120.0f;


            max_output_peak =
                -120.0f;


            max_gain_reduction =
                0.0f;
        }
    }
}

}


namespace h17 {


void run()
{
    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "Slipstream H17 - RX Output Stage"
    );


    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "Jitter buffer: 40 ms"
    );


    ESP_LOGI(
        TAG,
        "Mixer: H16 frozen behavior"
    );


    ESP_LOGI(
        TAG,
        "EQ: bypassed"
    );


    ESP_LOGI(
        TAG,
        "Limiter: %.1f dBFS / %lu ms release",
        audio::RxOutputStage::
            LIMITER_CEILING_DBFS,
        static_cast<unsigned long>(
            audio::RxOutputStage::
                LIMITER_RELEASE_MS
        )
    );


    if (
        !run_output_self_test()
    ) {

        ESP_LOGE(
            TAG,
            "H17 aborted"
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
        encode_queue ==
            nullptr ||
        decoded_queue ==
            nullptr
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
            "h17_codec",
            CODEC_TASK_STACK,
            nullptr,
            CODEC_TASK_PRIORITY,
            nullptr,
            0
        );


    BaseType_t audio_result =
        xTaskCreatePinnedToCore(
            audio_task,
            "h17_audio",
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
        "H17 tasks created"
    );
}


}