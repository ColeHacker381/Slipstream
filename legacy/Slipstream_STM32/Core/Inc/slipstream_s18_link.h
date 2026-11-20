#ifndef SLIPSTREAM_S18_LINK_H
#define SLIPSTREAM_S18_LINK_H
#include <stdint.h>
#define S18_PAYLOAD_BYTES 160U
#define S18_I2S_WORDS_PER_20MS 640U
typedef struct {
 uint32_t made,encoded,queued,sent,dropped,idle,dma_errors;
 uint32_t encode_avg_us,encode_max_us,queue_high,uart_commands,uart_errors;
 uint32_t active;
} S18_LinkStats;
int s18_link_init(void);
int s18_link_start(void);
void s18_link_push(const int16_t *pcm160);
void s18_link_run_one(void);
void s18_link_service(void);
void s18_link_get_stats(S18_LinkStats *out);
void s18_link_i2s_error(void);
#endif
