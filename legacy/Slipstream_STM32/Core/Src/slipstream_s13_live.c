/* Opt-in S13 sustained live-DSP Opus stress test.
 * This does NOT play decoded audio and is NOT a production scheduler.
 * All functions run in main context, outside DMA interrupts.
 */
#include "slipstream_s13_live.h"
#include "opus.h"
#include "main.h"
#include <stdlib.h>
#include <string.h>

#ifndef S13_OPUS_COMPLEXITY
#define S13_OPUS_COMPLEXITY 0
#endif
#ifndef S13_OPUS_LIVE_DECODE
#define S13_OPUS_LIVE_DECODE 1
#endif
#define S13_FRAME_SIZE 320
#define S13_PACKET_SIZE 512
static S13_FrameAssembler assembler;
static OpusEncoder *enc;
static OpusDecoder *dec;
static int16_t source[S13_FRAME_SIZE], decoded[S13_FRAME_SIZE];
static uint8_t packet[S13_PACKET_SIZE];
static S13_LiveStats stats;
static uint8_t enabled;
static uint64_t enc_cycles_total, dec_cycles_total;

static uint32_t us(uint32_t cycles)
{
    return SystemCoreClock ? (uint32_t)(((uint64_t)cycles*1000000ULL)/SystemCoreClock) : 0;
}

int s13_live_init(void)
{
    s13_frames_reset(&assembler);
    memset(&stats, 0, sizeof(stats));
    enc_cycles_total=dec_cycles_total=0;
    int es = opus_encoder_get_size(1), ds = opus_decoder_get_size(1);
    if (es<=0 || ds<=0) return 0;
    enc=(OpusEncoder *)malloc((size_t)es);
    dec=(OpusDecoder *)malloc((size_t)ds);
    if (!enc || !dec) { free(enc); free(dec); enc=NULL; dec=NULL; return 0; }
    int r=opus_encoder_init(enc,16000,1,OPUS_APPLICATION_VOIP);
    if(r==OPUS_OK) r=opus_decoder_init(dec,16000,1);
    if(r==OPUS_OK) r=opus_encoder_ctl(enc,OPUS_SET_BITRATE(20000));
    if(r==OPUS_OK) r=opus_encoder_ctl(enc,OPUS_SET_COMPLEXITY(S13_OPUS_COMPLEXITY));
    if(r==OPUS_OK) r=opus_encoder_ctl(enc,OPUS_SET_VBR(1));
    if(r==OPUS_OK) r=opus_encoder_ctl(enc,OPUS_SET_DTX(0));
    if(r==OPUS_OK) r=opus_encoder_ctl(enc,OPUS_SET_INBAND_FEC(0));
    if(r!=OPUS_OK) { free(enc); free(dec); enc=NULL; dec=NULL; return 0; }
    enabled=1;
    return 1;
}
void s13_live_stop(uint32_t reason)
{
    if(enabled) { enabled=0; stats.stopped=1; stats.stop_reason=reason; }
}
void s13_live_push(const int16_t *pcm160)
{
    if(!enabled) return;
    if(s13_frames_push(&assembler,pcm160)) stats.produced++;
    stats.dropped=assembler.dropped_oldest;
    if(assembler.count>stats.high_water) stats.high_water=assembler.count;
}
void s13_live_run_one(void)
{
    if(!enabled || !s13_frames_take(&assembler,source)) return;
    stats.consumed++;
    uint32_t start=DWT->CYCCNT;
    int n=opus_encode(enc,source,S13_FRAME_SIZE,packet,S13_PACKET_SIZE);
    uint32_t ec=DWT->CYCCNT-start;
    if(n<=0 || n>S13_PACKET_SIZE) {stats.errors++;s13_live_stop(2);return;}
    if((uint32_t)n>stats.max_encoded_bytes) stats.max_encoded_bytes=(uint32_t)n;
    start=DWT->CYCCNT;
    uint32_t dc=0;
#if S13_OPUS_LIVE_DECODE
    int d=opus_decode(dec,packet,n,decoded,S13_FRAME_SIZE,0);
    dc=DWT->CYCCNT-start;
    if(d!=S13_FRAME_SIZE) {stats.errors++;s13_live_stop(3);return;}
    stats.decoded++;
#endif
    stats.encoded++;
    enc_cycles_total+=ec;dec_cycles_total+=dc;
    uint32_t a=us(ec),b=us(dc);
    if(a>stats.enc_max_us) stats.enc_max_us=a;
    if(b>stats.dec_max_us) stats.dec_max_us=b;
    if(a+b>stats.max_run_us) stats.max_run_us=a+b;
}
void s13_live_get_stats(S13_LiveStats *out)
{
    if(!out) return;
    *out=stats;
    out->dropped=assembler.dropped_oldest;
    if(stats.encoded) {
        out->enc_avg_us=us((uint32_t)(enc_cycles_total/stats.encoded));
        out->dec_avg_us=us((uint32_t)(dec_cycles_total/stats.encoded));
    }
}
