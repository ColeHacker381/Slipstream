#ifndef SLIPSTREAM_S18_LINK_H
#define SLIPSTREAM_S18_LINK_H
#include <stdint.h>
#define S18_UART_BAUD 460800U
#define S18_FRAME_SAMPLES 320U
#define S18_PAYLOAD_BYTES 160U
#define S18_HEADER_BYTES 12U
#define S18_TRAILER_BYTES 2U
#define S18_WIRE_BYTES (S18_HEADER_BYTES+S18_PAYLOAD_BYTES+S18_TRAILER_BYTES)
typedef struct {
    uint32_t frames_produced, frames_encoded, frames_sent, tx_busy_drops, queue_drops;
    uint32_t tx_errors, uart_errors, tx_dma_completed;
    uint32_t codec_avg_us, codec_max_us, max_tx_submit_us, bytes_sent;
    uint32_t seq_last, last_timestamp_ms, stop, stop_reason;
} S18_LinkStats;
int s18_link_init(void);
void s18_link_push(const int16_t *pcm160);
void s18_link_run_one(void);
void s18_link_stop(uint32_t reason);
void s18_link_get_stats(S18_LinkStats *out);
#endif
