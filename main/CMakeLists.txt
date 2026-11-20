
idf_component_register(
    SRCS
        "main.cpp"

        # Hardware and codecs
        "codec/wm8960.cpp"
        "codec/opus_codec.cpp"

        # Audio
        "audio/audio_io.cpp"
        "audio/audio_processor.cpp"
        "audio/priority_mixer.cpp"
        "audio/rx_output_stage.cpp"

        # DSP
        "dsp/biquad.cpp"
        "dsp/voice_filters.cpp"
        "dsp/resampler.cpp"
        "dsp/noise_suppressor.cpp"
        "dsp/vad.cpp"
        "dsp/agc.cpp"
        "dsp/limiter.cpp"

        # Slipstream packet transport
        "transport/slipstream_packet.cpp"
        "transport/simulated_rf.cpp"
        "transport/jitter_buffer.cpp"
        "transport/espnow_transport.cpp"

        # New D1.5 test
        "tests/d1_5_rf_pipeline.cpp"
        "tests/d1_6_espnow_duplex.cpp"

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
)
