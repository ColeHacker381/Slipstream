#pragma once

#include <cstddef>
#include <cstdint>

namespace dsp {

struct VadResult {
    bool speech_active = false;

    bool speech_started = false;
    bool speech_ended = false;

    bool noise_estimate_ready = false;

    float level_dbfs = -120.0f;
    float noise_dbfs = -120.0f;
};


class VoiceActivityDetector {
public:

    static constexpr size_t BLOCK_SIZE = 160;

    VoiceActivityDetector();

    void reset();

    VadResult process_block(
        const int16_t *input,
        size_t samples
    );

    bool speech_active() const
    {
        return speech_active_;
    }

    bool noise_estimate_ready() const
    {
        return
            noise_frame_count_ >=
            NOISE_INIT_FRAMES;
    }


private:

    /*
     * 500 ms initial noise learning:
     *
     * 50 frames × 10 ms
     */
    static constexpr uint32_t
        NOISE_INIT_FRAMES = 50;


    /*
     * Require 20 ms above the start threshold
     * before entering speech state.
     */
    static constexpr uint32_t
        ATTACK_FRAMES = 4;


    /*
     * Keep VAD active for 250 ms after energy
     * drops below the speech threshold.
     */
    static constexpr uint32_t
        HANGOVER_FRAMES = 25;


    /*
     * Start speech when energy is approximately
     * 8 dB above the learned noise floor.
     *
     * Power ratio:
     *
     * 10^(8 / 10) = 6.3096
     */
    static constexpr float
        START_POWER_RATIO = 6.309573f;


    /*
     * Once speech has started, allow it to
     * continue down to approximately 4 dB
     * above the noise floor.
     *
     * This hysteresis prevents rapid toggling.
     */
    static constexpr float
        CONTINUE_POWER_RATIO = 2.511886f;


    /*
     * Slowly adapt the noise estimate when
     * speech is not active.
     */
    static constexpr float
        NOISE_ALPHA = 0.995f;


    /*
     * Absolute lower limit for speech triggering.
     *
     * This avoids tiny numerical/background
     * fluctuations becoming speech after H8
     * drives the noise floor very low.
     */
    static constexpr float
        MIN_SPEECH_DBFS = -50.0f;


    static float power_to_dbfs(
        float power
    );


    static float dbfs_to_power(
        float dbfs
    );


    bool speech_active_ = false;

    uint32_t noise_frame_count_ = 0;

    uint32_t attack_counter_ = 0;

    uint32_t hangover_counter_ = 0;


    float noise_power_ = 1.0f;

    float initial_noise_sum_ = 0.0f;
};

}