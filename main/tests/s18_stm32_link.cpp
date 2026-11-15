#include "s18_stm32_link.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "driver/i2s_std.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr const char *TAG = "S18_LINK";
constexpr i2s_port_t I2S_PORT = I2S_NUM_0;
constexpr uart_port_t CTRL_UART = UART_NUM_2;
constexpr int GPIO_BCLK = 26;   // STM PC10
constexpr int GPIO_WS = 25;     // STM PA4
constexpr int GPIO_DATA = 22;   // STM PC12
constexpr int GPIO_UART_RX = 16;// STM PA2
constexpr int GPIO_UART_TX = 17;// STM PA3

constexpr uint16_t SYNC = 0xA55A;
constexpr uint16_t VERSION = 0x1801;
constexpr uint16_t FRAME_BYTES = 160;
constexpr uint32_t SAMPLES_PER_SLOT = 640; // 20 ms * 16kHz * 2 I2S channels
constexpr size_t RX_BUFFER_WORDS = 512;
// Buffer holds one packet of 8 header + 80 compressed-data words.
constexpr size_t FRAME_WORDS = 88;
constexpr TickType_t READ_TIMEOUT = pdMS_TO_TICKS(150);
i2s_chan_handle_t rx_chan = nullptr;

struct Counters {
    uint32_t good = 0;
    uint32_t crc_errors = 0;
    uint32_t header_errors = 0;
    uint32_t sequence_gaps = 0;
    uint32_t repeated = 0;
    uint32_t read_timeouts = 0;
    uint32_t i2s_errors = 0;
    uint32_t rx_bytes = 0;
    uint32_t sent_start = 0;
    uint32_t received_acks = 0;
    uint16_t last_sequence = 0;
    bool have_sequence = false;
};
Counters counts;

uint16_t crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i=0; i<length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit=0; bit<8; ++bit)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) :
                                   static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

// Framing self-resynchronizes by searching for the two-word magic/version
// anywhere in the incoming stereo 16-bit word stream. A slot may start
// in the middle of an I2S DMA read, so no DMA-read alignment is assumed.
uint16_t frame[FRAME_WORDS] = {};
size_t frame_fill = 0;
bool have_magic = false;
bool swapped_words = false;

void consume_word(uint16_t word) {
    if (!frame_fill) {
        if (word == SYNC || static_cast<uint16_t>((word >> 8) | (word << 8)) == SYNC) {
            swapped_words = (word != SYNC);
            frame[frame_fill++] = SYNC;
        }
        return;
    }
    if (swapped_words)
        word = static_cast<uint16_t>((word >> 8) | (word << 8));
    if (frame_fill == 1) {
        if (word == VERSION) { frame[frame_fill++] = word; return; }
        counts.header_errors++;
        frame_fill = 0;
        // A coincident second magic word is handled next DMA sample.
        return;
    }
    frame[frame_fill++] = word;
    if (frame_fill == 4 && frame[3] != FRAME_BYTES) {
        counts.header_errors++;
        frame_fill = 0;
        return;
    }
    if (frame_fill != FRAME_WORDS) return;

    uint8_t compressed[FRAME_BYTES];
    for (size_t i=0; i<FRAME_BYTES/2; ++i) {
        compressed[2*i] = static_cast<uint8_t>(frame[8+i] >> 8);
        compressed[2*i+1] = static_cast<uint8_t>(frame[8+i]);
    }
    const uint16_t actual = crc16(compressed,sizeof(compressed));
    if (frame[4] == actual) {
        ++counts.good;
        const uint16_t seq = frame[2];
        if (counts.have_sequence) {
            const uint16_t next = static_cast<uint16_t>(counts.last_sequence + 1U);
            if (seq == counts.last_sequence) ++counts.repeated;
            else if (seq != next) {
                const uint16_t forward = static_cast<uint16_t>(seq - next);
                counts.sequence_gaps += (forward < 0x8000U) ? forward : 1U;
            }
        }
        counts.last_sequence = seq;
        counts.have_sequence = true;
        // S18 deliberately verifies compressed data only, no radio or
        // G.722 decoding: STM still handles microphone/headphone DSP.
    } else ++counts.crc_errors;
    frame_fill = 0;
}

void i2s_task(void *) {
    uint16_t words[RX_BUFFER_WORDS] = {};
    int64_t last_report = esp_timer_get_time();
    uint32_t prev_good = 0, prev_bytes = 0;
    while (true) {
        size_t got = 0;
        const esp_err_t err = i2s_channel_read(rx_chan,words,sizeof(words),&got,READ_TIMEOUT);
        if (err == ESP_ERR_TIMEOUT) ++counts.read_timeouts;
        else if (err != ESP_OK) {
            ++counts.i2s_errors;
            ESP_LOGW(TAG,"I2S read failed: %s",esp_err_to_name(err));
        }
        if (got) {
            counts.rx_bytes += static_cast<uint32_t>(got);
            for(size_t i=0; i<got/sizeof(uint16_t); ++i)
                consume_word(words[i]);
        }
        const int64_t now = esp_timer_get_time();
        if (now-last_report >= 1000000) {
            const float seconds = static_cast<float>(now-last_report)/1000000.0f;
            const uint32_t good_delta = counts.good-prev_good;
            const uint32_t byte_delta = counts.rx_bytes-prev_bytes;
            prev_good=counts.good;prev_bytes=counts.rx_bytes;
            last_report=now;
            ESP_LOGI(TAG,
                "RX good=%lu (+%.1f/s) seq=%u gaps=%lu repeat=%lu crc=%lu badhdr=%lu "
                "I2Sbytes/s=%.0f timeout=%lu ioerr=%lu",
                static_cast<unsigned long>(counts.good),
                static_cast<double>(good_delta/seconds),
                static_cast<unsigned>(counts.last_sequence),
                static_cast<unsigned long>(counts.sequence_gaps),
                static_cast<unsigned long>(counts.repeated),
                static_cast<unsigned long>(counts.crc_errors),
                static_cast<unsigned long>(counts.header_errors),
                static_cast<double>(byte_delta/seconds),
                static_cast<unsigned long>(counts.read_timeouts),
                static_cast<unsigned long>(counts.i2s_errors));
        }
    }
}

void uart_task(void *) {
    uint8_t incoming[96];
    char line[192] = {};
    size_t fill = 0;
    bool started = false;
    int64_t last_status = 0;
    // Let I2S RX start before requesting voice TX.
    vTaskDelay(pdMS_TO_TICKS(500));
    while(true) {
        if (!started) {
            const char command[] = "START\n";
            uart_write_bytes(CTRL_UART,command,sizeof(command)-1);
            counts.sent_start++;
            started = true;
            ESP_LOGI(TAG,"UART -> STM: START");
        }
        const int64_t now = esp_timer_get_time();
        if(now-last_status >= 3000000) {
            last_status=now;
            const char command[]="STATUS\n";
            uart_write_bytes(CTRL_UART,command,sizeof(command)-1);
        }
        const int n=uart_read_bytes(CTRL_UART,incoming,sizeof(incoming),pdMS_TO_TICKS(50));
        for(int i=0;i<n;i++) {
            const uint8_t ch=incoming[i];
            if(ch=='\r') continue;
            if(ch=='\n') {
                if(fill) {
                    line[fill]=0;
                    ESP_LOGI(TAG,"STM UART: %s",line);
                    counts.received_acks++;
                    fill=0;
                }
            }else if(fill+1<sizeof(line)) line[fill++]=static_cast<char>(ch);
            else fill=0;
        }
    }
}

esp_err_t init_uart() {
    const uart_config_t cfg = {
        .baud_rate=115200,
        .data_bits=UART_DATA_8_BITS,
        .parity=UART_PARITY_DISABLE,
        .stop_bits=UART_STOP_BITS_1,
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh=0,
        .source_clk=UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(CTRL_UART,2048,0,0,nullptr,0),TAG,"uart install");
    ESP_RETURN_ON_ERROR(uart_param_config(CTRL_UART,&cfg),TAG,"uart config");
    ESP_RETURN_ON_ERROR(uart_set_pin(CTRL_UART,GPIO_UART_TX,GPIO_UART_RX,
                                   UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE),TAG,"uart pins");
    return ESP_OK;
}
esp_err_t init_i2s() {
    i2s_chan_config_t channels=I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT,I2S_ROLE_SLAVE);
    channels.dma_desc_num=6;
    channels.dma_frame_num=256;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&channels,nullptr,&rx_chan),TAG,"new i2s RX");
    i2s_std_config_t cfg = {};
    cfg.clk_cfg=I2S_STD_CLK_DEFAULT_CONFIG(16000);
    cfg.slot_cfg=I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_16BIT,I2S_SLOT_MODE_STEREO);
    cfg.gpio_cfg.mclk=I2S_GPIO_UNUSED;
    cfg.gpio_cfg.bclk=static_cast<gpio_num_t>(GPIO_BCLK);
    cfg.gpio_cfg.ws=static_cast<gpio_num_t>(GPIO_WS);
    cfg.gpio_cfg.dout=I2S_GPIO_UNUSED;
    cfg.gpio_cfg.din=static_cast<gpio_num_t>(GPIO_DATA);
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx_chan,&cfg),TAG,"i2s slave std");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx_chan),TAG,"i2s enable");
    return ESP_OK;
}
} // namespace

namespace s18 {
void run() {
    ESP_LOGI(TAG,"====================================");
    ESP_LOGI(TAG,"S18 STM32 G722 I2S slave + UART2 test");
    ESP_LOGI(TAG,"BCLK GPIO26 / WS GPIO25 / DIN GPIO22");
    ESP_LOGI(TAG,"UART2 RX GPIO16 / TX GPIO17 (115200)");
    ESP_LOGI(TAG,"No ESP-NOW or audio playback in this test");
    ESP_LOGI(TAG,"====================================");
    ESP_ERROR_CHECK(init_uart());
    ESP_ERROR_CHECK(init_i2s());
    xTaskCreate(i2s_task,"s18_i2s_rx",6144,nullptr,10,nullptr);
    xTaskCreate(uart_task,"s18_uart",4096,nullptr,6,nullptr);
}
}
