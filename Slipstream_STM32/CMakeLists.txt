cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Slipstream_STM32)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE

    # CMSIS-DSP FIR resampling functions used by S7
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_init_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_interpolate_init_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_interpolate_f32.c

    # S8 CMSIS 512-point complex FFT and ESP32-compatible noise suppressor
    ${CMAKE_SOURCE_DIR}/Core/Src/slipstream_s8.c
    ${CMAKE_SOURCE_DIR}/Core/Src/slipstream_s9.c
    ${CMAKE_SOURCE_DIR}/Core/Src/slipstream_s10.c
    ${CMAKE_SOURCE_DIR}/Core/Src/slipstream_s11.c
    ${CMAKE_SOURCE_DIR}/Core/Src/slipstream_s13_frames.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix8_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/TransformFunctions/arm_bitreversal2.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

    # CMSIS-DSP
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/PrivateInclude
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE

    # STM32F412 = Cortex-M4F
    ARM_MATH_CM4
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)
# S13 genuine libopus is an explicit external ARM source/static-library input.
# No ESP32 prebuilt library is linked and no fake codec fallback is provided.
option(S13_OPUS_BENCHMARK "Run real Opus startup benchmark before I2S DMA" OFF)
option(S13_OPUS_LIVE_TEST "Opt-in sustained live DSP + Opus stress test (no decoded playback)" OFF)
option(S13_OPUS_LIVE_DECODE "Also run Opus decoder in live stress test (OFF isolates TX encode cost)" ON)
if(S13_OPUS_LIVE_TEST AND NOT S13_OPUS_BENCHMARK)
    message(FATAL_ERROR "S13_OPUS_LIVE_TEST requires S13_OPUS_BENCHMARK=ON (real library)")
endif()
set(S13_OPUS_COMPLEXITY 5 CACHE STRING "Opus encoder complexity for benchmarking (0..10)")
if(S13_OPUS_COMPLEXITY LESS 0 OR S13_OPUS_COMPLEXITY GREATER 10)
    message(FATAL_ERROR "S13_OPUS_COMPLEXITY must be between 0 and 10")
endif()
set(S13_OPUS_INCLUDE_DIR "" CACHE PATH "ARM-compatible libopus include directory containing opus.h")
set(S13_OPUS_LIBRARY "" CACHE FILEPATH "ARM Cortex-M4F libopus static archive (.a)")
if(S13_OPUS_BENCHMARK)
    if(NOT EXISTS "${S13_OPUS_INCLUDE_DIR}/opus.h")
        message(FATAL_ERROR "S13 requires a real libopus opus.h at S13_OPUS_INCLUDE_DIR")
    endif()
    if(NOT EXISTS "${S13_OPUS_LIBRARY}")
        message(FATAL_ERROR "S13 requires an ARM-targeted libopus archive at S13_OPUS_LIBRARY")
    endif()
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/Core/Src/slipstream_s13_opus_bench.c)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        "${S13_OPUS_INCLUDE_DIR}")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        S13_OPUS_BENCHMARK=1 S13_OPUS_COMPLEXITY=${S13_OPUS_COMPLEXITY})
    # GNU ld resolves static archives left-to-right. CubeMX already links -lm
    # before this external archive, so repeat m AFTER libopus.a to resolve
    # floor/log and other math functions referenced only by Opus.
    target_link_libraries(${CMAKE_PROJECT_NAME}
        "${S13_OPUS_LIBRARY}"
        m
    )
    if(S13_OPUS_LIVE_TEST)
        target_sources(${CMAKE_PROJECT_NAME} PRIVATE
            ${CMAKE_SOURCE_DIR}/Core/Src/slipstream_s13_live.c)
        target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE S13_OPUS_LIVE_TEST=1
            S13_OPUS_LIVE_DECODE=$<BOOL:${S13_OPUS_LIVE_DECODE}>)
        message(WARNING "S13 LIVE stress test is enabled; may drop audio blocks. NOT production-safe.")
    endif()
    message(STATUS "S13 real Opus benchmark enabled; verify archive ABI and available SRAM")
endif()

# S14 isolated Speex benchmark. It does not require, enable or modify Opus.
option(S14_SPEEX_BENCHMARK "Run Speex 1.2.1 startup benchmark before audio DMA" OFF)
set(S14_SPEEX_QUALITY 6 CACHE STRING "Speex quality 0..10")
set(S14_SPEEX_COMPLEXITY 1 CACHE STRING "Speex complexity 0..10")
set(S14_SPEEX_ROOT "" CACHE PATH "Speex upstream 1.2.1 source root")
set(S14_SPEEX_BUILD "" CACHE PATH "Speex fixed Cortex-M4 build output")
if(S14_SPEEX_BENCHMARK)
    if(S14_SPEEX_QUALITY LESS 0 OR S14_SPEEX_QUALITY GREATER 10 OR
       S14_SPEEX_COMPLEXITY LESS 0 OR S14_SPEEX_COMPLEXITY GREATER 10)
        message(FATAL_ERROR "S14 Speex quality/complexity must be 0..10")
    endif()
    if(NOT EXISTS "${S14_SPEEX_ROOT}/include/speex/speex.h" OR
       NOT EXISTS "${S14_SPEEX_BUILD}/include/speex/speex_config_types.h" OR
       NOT EXISTS "${S14_SPEEX_BUILD}/libspeex.a")
        message(FATAL_ERROR "Run bash setup_speex.sh and set S14_SPEEX_ROOT / S14_SPEEX_BUILD")
    endif()
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/Core/Src/slipstream_s14_speex_bench.c)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        "${S14_SPEEX_ROOT}/include" "${S14_SPEEX_BUILD}/include"
        "${S14_SPEEX_BUILD}/include/speex")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        S14_SPEEX_BENCHMARK=1
        S14_SPEEX_QUALITY=${S14_SPEEX_QUALITY}
        S14_SPEEX_COMPLEXITY=${S14_SPEEX_COMPLEXITY})
    target_link_libraries(${CMAKE_PROJECT_NAME} "${S14_SPEEX_BUILD}/libspeex.a" m)
    message(STATUS "S14 Speex startup-only benchmark enabled")
endif()

# S15 startup-only G.722 reference codec benchmark, 16kHz mono, 64kbps.
option(S15_G722_BENCHMARK "Run G.722 startup encode/decode benchmark" OFF)
set(S15_G722_ROOT "" CACHE PATH "Standalone G.722 source root")
set(S15_G722_BUILD "" CACHE PATH "Standalone G.722 Cortex-M4 library directory")
if(S15_G722_BENCHMARK)
    if(NOT EXISTS "${S15_G722_ROOT}/src/g722.h" OR
       NOT EXISTS "${S15_G722_BUILD}/libg722.a")
       message(FATAL_ERROR "S15 requires bash setup_g722.sh and source/archive paths")
    endif()
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/Core/Src/slipstream_s15_g722_bench.c)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE "${S15_G722_ROOT}/src")
    target_link_libraries(${CMAKE_PROJECT_NAME} "${S15_G722_BUILD}/libg722.a")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE S15_G722_BENCHMARK=1)
    message(STATUS "S15 G.722 isolated startup benchmark enabled")
endif()

# S16 independent Codec 2 3200bps startup-only benchmark, includes 16<->8 kHz FIR.
option(S16_CODEC2_BENCHMARK "Run Codec2 3200 startup encode/decode+resample benchmark" OFF)
set(S16_CODEC2_ROOT "" CACHE PATH "Upstream Codec2 checkout root")
set(S16_CODEC2_BUILD "" CACHE PATH "Cross-built Codec2 library/build root")
if(S16_CODEC2_BENCHMARK)
    if(NOT EXISTS "${S16_CODEC2_ROOT}/src/codec2.h" OR
       NOT EXISTS "${S16_CODEC2_BUILD}/src/libcodec2.a" OR
       NOT EXISTS "${S16_CODEC2_BUILD}/codec2/version.h")
        message(FATAL_ERROR "Run bash setup_codec2.sh and configure S16_CODEC2_ROOT/BUILD")
    endif()
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/Core/Src/slipstream_s16_codec2_bench.c)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        "${S16_CODEC2_ROOT}/src" "${S16_CODEC2_BUILD}")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE S16_CODEC2_BENCHMARK=1)
    target_link_libraries(${CMAKE_PROJECT_NAME} "${S16_CODEC2_BUILD}/src/libcodec2.a" m)
    message(STATUS "S16 Codec 2 3200 mode benchmark enabled (includes FIR sample conversion)")
endif()
