idf_component_register(
    SRCS
        "main.cpp"

        "codec/wm8960.cpp"
        "codec/opus_codec.cpp"

        "audio/audio_io.cpp"
        "audio/audio_processor.cpp"
        "audio/priority_mixer.cpp"
        "audio/rx_output_stage.cpp"

        "dsp/biquad.cpp"
        "dsp/voice_filters.cpp"
        "dsp/resampler.cpp"
        "dsp/noise_suppressor.cpp"
        "dsp/vad.cpp"
        "dsp/agc.cpp"
        "dsp/limiter.cpp"

        "transport/slipstream_packet.cpp"
        "transport/simulated_rf.cpp"
        "transport/jitter_buffer.cpp"
        "transport/espnow_transport.cpp"

        "bluetooth/a2dp_sink.cpp"

        "tests/legacy/h0_i2c.cpp"
        "tests/legacy/h1_playback.cpp"
        "tests/legacy/h2_microphone.cpp"
        "tests/legacy/h3_loopback.cpp"
        "tests/legacy/h4_block_processing.cpp"
        "tests/legacy/h5_voice_domain.cpp"
        "tests/legacy/h6_filters.cpp"
        "tests/legacy/h7_resampler.cpp"
        "tests/legacy/h8_noise_suppression.cpp"
        "tests/legacy/h9_vad.cpp"
        "tests/legacy/h10_agc.cpp"
        "tests/legacy/h11_limiter.cpp"
        "tests/legacy/h12_opus.cpp"
        "tests/legacy/h13_packet.cpp"
        "tests/legacy/h14_jitter.cpp"
        "tests/legacy/h15_latency.cpp"
        "tests/legacy/h16_priority_mixer.cpp"
        "tests/legacy/h17_rx_output.cpp"

        "tests/h18_espnow.cpp"
        "tests/h19_bluetooth_mix.cpp"


    INCLUDE_DIRS
        "."

    PRIV_REQUIRES
        esp_driver_i2c
        esp_driver_i2s
        esp_timer
        esp-dsp
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        bt
        esp_ringbuf
)