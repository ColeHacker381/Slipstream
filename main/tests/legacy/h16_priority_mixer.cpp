#include "h16_priority_mixer.hpp"

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "audio/audio_io.hpp"
#include "audio/audio_processor.hpp"
#include "audio/priority_mixer.hpp"

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
    "H16";


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
 * Frozen H15 winner.
 */
constexpr uint32_t JITTER_DELAY_FRAMES =
    2;


constexpr uint8_t STREAM_ID =
    1;


constexpr uint8_t RF_CHANNEL =
    0;


/*
 * 48 kHz stereo / 10 ms.
 */
constexpr size_t HARDWARE_FRAMES =
    audio_processor::
        HARDWARE_FRAMES_PER_BLOCK;


constexpr size_t HARDWARE_SAMPLES =
    HARDWARE_FRAMES *
    2;


/*
 * Synthetic BT tone level.
 *
 * Keep it low enough that RF speech + BT doesn't
 * spend all its time saturating.
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


int16_t mixed_hardware[
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


audio::PriorityMixer priority_mixer;


/*
 * ============================================================
 * Synthetic Bluetooth source
 * ============================================================
 *
 * Left:
 *     440 Hz
 *
 * Right:
 *     660 Hz
 *
 * This makes it easy to hear that a stereo background
 * source is being attenuated while RF speech continues.
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
 * CORE 0
 *
 * Same H15 RF path.
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


    uint64_t crc_rejects =
        0;


    uint64_t misses =
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


        /*
         * ----------------------------------------------------
         * Encode.
         * ----------------------------------------------------
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

            continue;
        }


        /*
         * ----------------------------------------------------
         * Build packet.
         * ----------------------------------------------------
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


        /*
         * ----------------------------------------------------
         * Simulated RF.
         * ----------------------------------------------------
         */

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


        /*
         * ----------------------------------------------------
         * Receive arrivals.
         * ----------------------------------------------------
         */

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


        /*
         * ----------------------------------------------------
         * 40 ms H15 playout.
         * ----------------------------------------------------
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


                        /*
                         * IMPORTANT H16 CHANGE:
                         *
                         * Carry the transmitted VAD flag all
                         * the way into the RX mixer.
                         */
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


            /*
             * ------------------------------------------------
             * Missing packet -> PLC.
             *
             * Preserve RF speech priority for isolated loss.
             *
             * If the previous decoded state was active,
             * remain active through the PLC frame.
             * ------------------------------------------------
             */

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


            if (
                xQueueSend(
                    decoded_queue,
                    &output_frame,
                    0
                )
                !=
                pdTRUE
            ) {

                /*
                 * Keep H16 simple; queue state is visible
                 * in the periodic report.
                 */
            }


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

                static_cast<
                    unsigned long long
                >(
                    frames
                ),

                static_cast<
                    unsigned long long
                >(
                    misses
                ),

                static_cast<
                    unsigned long long
                >(
                    plc_frames
                ),

                static_cast<
                    unsigned long long
                >(
                    crc_rejects
                ),

                static_cast<
                    unsigned int
                >(
                    jitter_buffer.size()
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
 * CORE 1
 *
 * Audio + H16 priority mixer.
 * ============================================================
 */

void audio_task(
    void *argument
)
{
    (void) argument;


    ESP_LOGI(
        TAG,
        "Audio/mixer task running on CPU%d",
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


    VoiceFrame tx_frame = {};


    size_t tx_half =
        0;


    VoiceFrame playback_frame = {};


    bool have_playback =
        false;


    size_t playback_half =
        0;


    bool rf_speech_this_block =
        false;


    uint64_t blocks =
        0;


    uint64_t tx_drops =
        0;


    uint64_t underruns =
        0;


    uint64_t plc_played =
        0;


    uint64_t total_processing_us =
        0;


    uint32_t max_processing_us =
        0;


    ESP_LOGI(
        TAG,
        "H16 priority mixer starting"
    );


    ESP_LOGI(
        TAG,
        "BT normal: %.1f dB",
        audio::PriorityMixer::
            BT_NORMAL_DB
    );


    ESP_LOGI(
        TAG,
        "BT ducked: %.1f dB",
        audio::PriorityMixer::
            BT_DUCKED_DB
    );


    ESP_LOGI(
        TAG,
        "RF gain: %.1f dB",
        audio::PriorityMixer::
            RF_GAIN_DB
    );


    ESP_LOGI(
        TAG,
        "Attack=%lu ms Hold=%lu ms Release=%lu ms",
        static_cast<unsigned long>(
            audio::PriorityMixer::
                ATTACK_MS
        ),
        static_cast<unsigned long>(
            audio::PriorityMixer::
                HOLD_MS
        ),
        static_cast<unsigned long>(
            audio::PriorityMixer::
                RELEASE_MS
        )
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


        /*
         * ----------------------------------------------------
         * TX side H6-H11.
         * ----------------------------------------------------
         */

        audio_processor::
            process_input_block(
                hardware_input,
                voice_block,
                stats
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
            stats.speech_active
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
         * ----------------------------------------------------
         * RX frame acquisition.
         * ----------------------------------------------------
         */

        const int16_t *rf_voice =
            silence_block;


        rf_speech_this_block =
            false;


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


            rf_speech_this_block =
                playback_frame.
                    speech_active;


            playback_half++;
        }


        /*
         * ----------------------------------------------------
         * 16 kHz RF → 48 kHz stereo.
         * ----------------------------------------------------
         */

        audio_processor::
            render_voice_block(
                rf_voice,
                rf_hardware,
                stats
            );


        /*
         * ----------------------------------------------------
         * Fake Bluetooth/music PCM.
         * ----------------------------------------------------
         */

        generate_bt_test_audio();


        /*
         * ----------------------------------------------------
         * H16 mixer.
         * ----------------------------------------------------
         */

        priority_mixer.
            set_rf_speech_active(
                rf_speech_this_block
            );


        priority_mixer.
            process_block(
                bt_hardware,
                rf_hardware,
                mixed_hardware,
                HARDWARE_SAMPLES
            );


        /*
         * ----------------------------------------------------
         * Playback.
         * ----------------------------------------------------
         */

        uint32_t processing_us =
            static_cast<uint32_t>(
                esp_timer_get_time() -
                start
            );


        ESP_ERROR_CHECK(
            audio::write(
                mixed_hardware,
                sizeof(
                    mixed_hardware
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

                "MIX | "
                "blocks=%llu | "
                "proc=%llu/%lu us | "
                "RF=%s | "
                "duck=%s | "
                "BTgain=%.2f dB | "
                "txdrop=%llu | "
                "underrun=%llu | "
                "PLC=%llu | "
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

                rf_speech_this_block
                    ? "YES"
                    : "NO",

                priority_mixer.
                    ducking()
                    ? "YES"
                    : "NO",

                priority_mixer.
                    current_bt_gain_db(),

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


            underruns =
                0;


            plc_played =
                0;
        }
    }
}

}


namespace h16 {


void run()
{
    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "Slipstream H16 - Priority Mixer"
    );


    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "RX jitter buffer: 40 ms"
    );


    ESP_LOGI(
        TAG,
        "Synthetic stereo BT source enabled"
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
            "h16_codec",
            CODEC_TASK_STACK,
            nullptr,
            CODEC_TASK_PRIORITY,
            nullptr,
            0
        );


    BaseType_t audio_result =
        xTaskCreatePinnedToCore(
            audio_task,
            "h16_audio",
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
        "H16 tasks created"
    );
}


}