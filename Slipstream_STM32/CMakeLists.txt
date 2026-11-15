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