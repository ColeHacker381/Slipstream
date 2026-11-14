#include "wm8960.hpp"

#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"

namespace {

constexpr const char *TAG = "wm8960";

constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;

constexpr uint8_t WM8960_ADDR = 0x1A;
constexpr uint8_t REG_LEFT_INPUT_VOLUME   = 0x00;
constexpr uint8_t REG_LEFT_ADC_VOLUME     = 0x15;
constexpr uint8_t REG_ADCL_SIGNAL_PATH    = 0x20;
constexpr uint8_t REG_ADDITIONAL_CTRL_4   = 0x30;

// WM8960 register addresses used by H1
constexpr uint8_t REG_LOUT1_VOLUME       = 0x02;
constexpr uint8_t REG_ROUT1_VOLUME       = 0x03;
constexpr uint8_t REG_CLOCKING_1         = 0x04;
constexpr uint8_t REG_ADC_DAC_CTRL_1     = 0x05;
constexpr uint8_t REG_AUDIO_INTERFACE_1  = 0x07;
constexpr uint8_t REG_CLOCKING_2         = 0x08;
constexpr uint8_t REG_RESET              = 0x0F;
constexpr uint8_t REG_PWR_MGMT_1         = 0x19;
constexpr uint8_t REG_PWR_MGMT_2         = 0x1A;
constexpr uint8_t REG_LEFT_OUT_MIX_1     = 0x22;
constexpr uint8_t REG_RIGHT_OUT_MIX_2    = 0x25;
constexpr uint8_t REG_PWR_MGMT_3         = 0x2F;
constexpr uint8_t REG_PLL_N               = 0x34;
constexpr uint8_t REG_PLL_K_1             = 0x35;
constexpr uint8_t REG_PLL_K_2             = 0x36;
constexpr uint8_t REG_PLL_K_3             = 0x37;

i2c_master_bus_handle_t bus_handle = nullptr;
i2c_master_dev_handle_t dev_handle = nullptr;


/*
 * WM8960 registers are 9 bits wide.
 *
 * I2C transfer format:
 *
 * byte 0:
 *   register address in bits [7:1]
 *   data bit 8 in bit [0]
 *
 * byte 1:
 *   data bits [7:0]
 */
esp_err_t write_register(uint8_t reg, uint16_t value)
{
    value &= 0x01FF;

    uint8_t data[2];

    data[0] =
        static_cast<uint8_t>(
            (reg << 1) |
            ((value >> 8) & 0x01)
        );

    data[1] =
        static_cast<uint8_t>(
            value & 0xFF
        );

    esp_err_t err = i2c_master_transmit(
        dev_handle,
        data,
        sizeof(data),
        100
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Register write failed: reg=0x%02X value=0x%03X error=%s",
            reg,
            value,
            esp_err_to_name(err)
        );
    }

    return err;
}


/*
 * Helper so the codec setup stops immediately if a register write fails.
 */
esp_err_t write_checked(uint8_t reg, uint16_t value)
{
    esp_err_t err = write_register(reg, value);

    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

}


namespace wm8960 {

esp_err_t init_i2c()
{
    i2c_master_bus_config_t bus_config = {};

    bus_config.i2c_port = I2C_NUM_0;
    bus_config.sda_io_num =
        static_cast<gpio_num_t>(SDA_PIN);
    bus_config.scl_io_num =
        static_cast<gpio_num_t>(SCL_PIN);
    bus_config.clk_source =
        I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.intr_priority = 0;
    bus_config.trans_queue_depth = 0;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(
        &bus_config,
        &bus_handle
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to create I2C bus: %s",
            esp_err_to_name(err)
        );

        return err;
    }

    i2c_device_config_t dev_config = {};

    dev_config.dev_addr_length =
        I2C_ADDR_BIT_LEN_7;
    dev_config.device_address =
        WM8960_ADDR;
    dev_config.scl_speed_hz =
        100000;

    err = i2c_master_bus_add_device(
        bus_handle,
        &dev_config,
        &dev_handle
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to add WM8960: %s",
            esp_err_to_name(err)
        );

        return err;
    }

    return ESP_OK;
}

bool probe()
{
    esp_err_t err = i2c_master_probe(
        bus_handle,
        WM8960_ADDR,
        100
    );

    if (err == ESP_OK) {
        ESP_LOGI(
            TAG,
            "WM8960 found at 0x1A"
        );

        return true;
    }

    ESP_LOGE(
        TAG,
        "WM8960 not found: %s",
        esp_err_to_name(err)
    );

    return false;
}

esp_err_t configure_headphone_playback()
{
    ESP_LOGI(
        TAG,
        "Configuring WM8960 for headphone playback"
    );

    /*
     * Reset all codec registers.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_RESET, 0x000),
        TAG,
        "Reset failed"
    );

    vTaskDelay(pdMS_TO_TICKS(10));


    /*
     * R25 - Power Management 1
     *
     * VREF = enabled
     * VMID = 2 x 50k
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_PWR_MGMT_1, 0x0C0),
        TAG,
        "PWR1 failed"
    );

    /*
     * Give VMID/VREF a little time to stabilize.
     */
    vTaskDelay(pdMS_TO_TICKS(50));


    /*
     * Configure internal PLL.
     *
     * SparkFun board has a 24 MHz onboard MCLK.
     *
     * These settings generate the clock required for
     * 44.1 kHz audio.
     */

    // PLLN = 7
    // fractional mode
    // input prescale /2
    ESP_RETURN_ON_ERROR(
        write_checked(REG_PLL_N, 0x037),
        TAG,
        "PLL N failed"
    );

    // PLL K = 0x86C226
    ESP_RETURN_ON_ERROR(
        write_checked(REG_PLL_K_1, 0x086),
        TAG,
        "PLL K1 failed"
    );

    ESP_RETURN_ON_ERROR(
        write_checked(REG_PLL_K_2, 0x0C2),
        TAG,
        "PLL K2 failed"
    );

    ESP_RETURN_ON_ERROR(
        write_checked(REG_PLL_K_3, 0x026),
        TAG,
        "PLL K3 failed"
    );


    /*
     * Clock configuration.
     *
     * CLKSEL = PLL
     * SYSCLKDIV = /2
     * DACDIV = /1
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_CLOCKING_1, 0x005),
        TAG,
        "Clocking 1 failed"
    );


    /*
     * BCLKDIV = /4
     * DCLKDIV = /16
     *
     * DCLK isn't important to H1, but this matches
     * SparkFun's normal 44.1k configuration.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_CLOCKING_2, 0x1C4),
        TAG,
        "Clocking 2 failed"
    );


    /*
     * Digital audio interface.
     *
     * I2S format
     * 16-bit samples
     * Codec peripheral/slave mode
     *
     * ESP32 supplies BCLK and LRCLK.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_AUDIO_INTERFACE_1, 0x002),
        TAG,
        "Audio interface failed"
    );


    /*
     * Route LEFT DAC -> LEFT output mixer.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_LEFT_OUT_MIX_1, 0x150),
        TAG,
        "Left mixer failed"
    );


    /*
     * Route RIGHT DAC -> RIGHT output mixer.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_RIGHT_OUT_MIX_2, 0x150),
        TAG,
        "Right mixer failed"
    );


    /*
     * Enable both output mixers.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_PWR_MGMT_3, 0x00C),
        TAG,
        "PWR3 failed"
    );


    /*
     * Power Management 2
     *
     * Left DAC        ON
     * Right DAC       ON
     * Left headphone ON
     * Right headphone ON
     * OUT3 mixer      ON
     * PLL             ON
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_PWR_MGMT_2, 0x1E3),
        TAG,
        "PWR2 failed"
    );


    /*
     * Disable DAC digital mute.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_ADC_DAC_CTRL_1, 0x000),
        TAG,
        "DAC unmute failed"
    );


    /*
     * Headphone volume.
     *
     * -20 dB initially.
     *
     * WM8960 headphone setting:
     *
     * 48  = -73 dB
     * 121 =   0 dB
     * 127 =  +6 dB
     *
     * -20 dB => 101 = 0x65.
     *
     * Bit 8 is OUT1VU, which updates the channels.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_LOUT1_VOLUME, 0x165),
        TAG,
        "Left headphone volume failed"
    );

    ESP_RETURN_ON_ERROR(
        write_checked(REG_ROUT1_VOLUME, 0x165),
        TAG,
        "Right headphone volume failed"
    );


    ESP_LOGI(
        TAG,
        "WM8960 headphone playback configured"
    );

    return ESP_OK;
}

esp_err_t configure_microphone_capture()
{
    ESP_LOGI(
        TAG,
        "Configuring WM8960 microphone capture"
    );

    /*
     * Start from a known state.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_RESET, 0x000),
        TAG,
        "Reset failed"
    );

    vTaskDelay(pdMS_TO_TICKS(10));


    /*
     * Clock setup.
     *
     * Same 24 MHz -> PLL -> 44.1 kHz clock setup
     * that worked during H1.
     */

    ESP_RETURN_ON_ERROR(
        write_checked(REG_PLL_N, 0x037),
        TAG,
        "PLL N failed"
    );

    ESP_RETURN_ON_ERROR(
        write_checked(REG_PLL_K_1, 0x086),
        TAG,
        "PLL K1 failed"
    );

    ESP_RETURN_ON_ERROR(
        write_checked(REG_PLL_K_2, 0x0C2),
        TAG,
        "PLL K2 failed"
    );

    ESP_RETURN_ON_ERROR(
        write_checked(REG_PLL_K_3, 0x026),
        TAG,
        "PLL K3 failed"
    );


    /*
     * Power Management 1:
     *
     * VMID      enabled, 2 x 50k
     * VREF      enabled
     * AINL      enabled
     * Left ADC  enabled
     * MICBIAS   enabled
     *
     * 0x0C0
     * + 0x020 AINL
     * + 0x008 ADC left
     * + 0x002 MICBIAS
     *
     * = 0x0EA
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_PWR_MGMT_1, 0x0EA),
        TAG,
        "PWR1 microphone config failed"
    );

    vTaskDelay(pdMS_TO_TICKS(50));


    /*
     * Power Management 2:
     *
     * PLL enabled.
     *
     * We don't need DAC/headphone power for H2.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_PWR_MGMT_2, 0x001),
        TAG,
        "PLL power failed"
    );


    /*
     * Power Management 3:
     *
     * Left microphone PGA enabled.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_PWR_MGMT_3, 0x020),
        TAG,
        "Left mic PGA power failed"
    );


    /*
     * MICBIAS = 0.9 * AVDD.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_ADDITIONAL_CTRL_4, 0x001),
        TAG,
        "Mic bias voltage failed"
    );


    /*
     * Left input PGA volume.
     *
     * 0     = -17.25 dB
     * step  = 0.75 dB
     *
     * +24 dB:
     *
     * (24 - (-17.25)) / 0.75 = 55
     *
     * 55 = 0x37
     *
     * Bit 8 = input-volume update.
     *
     * Result = 0x137
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_LEFT_INPUT_VOLUME, 0x137),
        TAG,
        "Left PGA gain failed"
    );


    /*
     * Left ADC signal path.
     *
     * bit 8: connect LINPUT1 to PGA inverted input
     * bit 6: select LINPUT2 as non-inverting input
     * bits 5:4: mic boost = 0 dB
     * bit 3: connect PGA into input boost path
     *
     * = 0x148
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_ADCL_SIGNAL_PATH, 0x148),
        TAG,
        "Left ADC signal path failed"
    );


    /*
     * ADC digital volume.
     *
     * 195 = 0 dB
     * bit 8 = volume update
     *
     * 0x100 + 0xC3 = 0x1C3
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_LEFT_ADC_VOLUME, 0x1C3),
        TAG,
        "Left ADC volume failed"
    );


    /*
     * Audio interface:
     *
     * standard I2S
     * 16-bit samples
     * codec peripheral/slave
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_AUDIO_INTERFACE_1, 0x002),
        TAG,
        "Audio interface failed"
    );


    /*
     * Clocking:
     *
     * PLL selected.
     * SYSCLK / 2.
     * ADC divider appropriate for 44.1k.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_CLOCKING_1, 0x005),
        TAG,
        "Clocking 1 failed"
    );

    ESP_RETURN_ON_ERROR(
        write_checked(REG_CLOCKING_2, 0x1C4),
        TAG,
        "Clocking 2 failed"
    );


    ESP_LOGI(
        TAG,
        "Microphone configured: left channel, +24 dB PGA"
    );

    return ESP_OK;
}

esp_err_t configure_full_duplex()
{
    ESP_LOGI(TAG, "Configuring WM8960 full-duplex audio");

    ESP_RETURN_ON_ERROR(
        write_checked(REG_RESET, 0x000),
        TAG,
        "Reset failed"
    );

    vTaskDelay(pdMS_TO_TICKS(10));

    /*
    * 24 MHz MCLK
    * prescaled /2 -> 12 MHz PLL input
    *
    * 48 kHz family:
    *
    * SYSCLK = 12.288 MHz
    * PLL N  = 8
    * PLL K  = 0x3126E8
    */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_PLL_N, 0x038),
        TAG,
        "PLL N failed"
    );

    ESP_RETURN_ON_ERROR(
        write_checked(REG_PLL_K_1, 0x031),
        TAG,
        "PLL K1 failed"
    );

    ESP_RETURN_ON_ERROR(
        write_checked(REG_PLL_K_2, 0x026),
        TAG,
        "PLL K2 failed"
    );

    ESP_RETURN_ON_ERROR(
        write_checked(REG_PLL_K_3, 0x0E8),
        TAG,
        "PLL K3 failed"
    );

    /*
     * PWR1:
     *
     * VREF
     * VMID
     * left input
     * left ADC
     * MICBIAS
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_PWR_MGMT_1, 0x0EA),
        TAG,
        "PWR1 failed"
    );

    vTaskDelay(pdMS_TO_TICKS(50));

    /*
     * PWR2:
     *
     * PLL
     * left/right DAC
     * left/right headphone drivers
     * OUT3
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_PWR_MGMT_2, 0x1E3),
        TAG,
        "PWR2 failed"
    );

    /*
     * PWR3:
     *
     * left mic PGA
     * left output mixer
     * right output mixer
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_PWR_MGMT_3, 0x02C),
        TAG,
        "PWR3 failed"
    );

    /*
     * Mic bias: 0.9 * AVDD.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_ADDITIONAL_CTRL_4, 0x001),
        TAG,
        "Mic bias failed"
    );

    /*
     * Left microphone PGA: +24 dB.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_LEFT_INPUT_VOLUME, 0x12F),
        TAG,
        "Mic PGA failed"
    );

    /*
     * LINPUT1 = inverted input
     * LINPUT2 = non-inverting input
     * PGA -> ADC path
     * boost = 0 dB
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_ADCL_SIGNAL_PATH, 0x148),
        TAG,
        "Mic signal path failed"
    );

    /*
     * Left ADC digital gain: 0 dB.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_LEFT_ADC_VOLUME, 0x1C3),
        TAG,
        "ADC volume failed"
    );

    /*
     * Left DAC -> left mixer.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_LEFT_OUT_MIX_1, 0x150),
        TAG,
        "Left output mixer failed"
    );

    /*
     * Right DAC -> right mixer.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_RIGHT_OUT_MIX_2, 0x150),
        TAG,
        "Right output mixer failed"
    );

    /*
     * I2S, 16-bit, peripheral mode.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_AUDIO_INTERFACE_1, 0x002),
        TAG,
        "Audio interface failed"
    );

    /*
     * Clock configuration.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_CLOCKING_1, 0x005),
        TAG,
        "Clocking 1 failed"
    );

    ESP_RETURN_ON_ERROR(
        write_checked(REG_CLOCKING_2, 0x1C4),
        TAG,
        "Clocking 2 failed"
    );

    /*
     * DAC unmuted.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_ADC_DAC_CTRL_1, 0x000),
        TAG,
        "DAC unmute failed"
    );

    /*
     * Keep headphone gain conservative for loopback.
     *
     * -20 dB.
     */
    ESP_RETURN_ON_ERROR(
        write_checked(REG_LOUT1_VOLUME, 0x165),
        TAG,
        "Left headphone gain failed"
    );

    ESP_RETURN_ON_ERROR(
        write_checked(REG_ROUT1_VOLUME, 0x165),
        TAG,
        "Right headphone gain failed"
    );

    ESP_LOGI(TAG, "WM8960 full-duplex configuration complete");

    return ESP_OK;
}

}