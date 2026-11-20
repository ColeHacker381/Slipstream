/* S17 G.722 live microphone DSP -> bounded 20ms frame queue -> codec.
 * No actual radio transmission; no decoded audio playback. Foreground
 * scheduler only (like S13), auto-stop is controlled by main.c.
 */
#include "slipstream_s17_g722_live.h"
#include "main.h"
#include "g722.h"
#include <string.h>

#define S17_FRAME_SAMPLES 320
#define S17_PACKET_BYTES 160
#define S17_RADIO_HEADER_BYTES 12
static S13_FrameAssembler frames;
static g722_encode_state_t encoder;
static g722_decode_state_t decoder;
static int16_t input[S17_FRAME_SAMPLES], output[S17_FRAME_SAMPLES];
static uint8_t packet[S17_PACKET_BYTES];
static S17_G722LiveStats stats;
static uint64_t total_enc_cycles, total_dec_cycles;
static uint8_t enabled;

static uint32_t cycles_to_us(uint32_t cycles) {
    return SystemCoreClock ? (uint32_t)((uint64_t)cycles*1000000ULL/SystemCoreClock) : 0U;
}
int s17_g722_live_init(void) {
    s13_frames_reset(&frames);
    memset(&stats,0,sizeof(stats));
    total_enc_cycles=total_dec_cycles=0U;
    enabled=0U;
    if(!g722_encode_init(&encoder,64000,0) ||
       !g722_decode_init(&decoder,64000,0)) {
        stats.errors++;
        return 0;
    }
    stats.packet_bytes=S17_PACKET_BYTES;
    stats.modeled_overhead_bytes=S17_RADIO_HEADER_BYTES;
    enabled=1U;
    return 1;
}
void s17_g722_live_stop(uint32_t reason) {
    if(enabled) {
        enabled=0U;
        stats.stopped=1U;
        stats.reason=reason;
    }
}
void s17_g722_live_push(const int16_t *block160) {
    if(!enabled) return;
    if(s13_frames_push(&frames,block160)) stats.produced++;
    stats.dropped=frames.dropped_oldest;
    if(frames.count>stats.queue_high)stats.queue_high=frames.count;
    if(stats.dropped) s17_g722_live_stop(5U);
}
void s17_g722_live_run_one(void) {
    if(!enabled || !s13_frames_take(&frames,input))return;
    stats.consumed++;
    uint32_t begin=DWT->CYCCNT;
    int n=g722_encode(&encoder,packet,input,S17_FRAME_SAMPLES);
    uint32_t ec=DWT->CYCCNT-begin;
    if(n!=S17_PACKET_BYTES) {
        stats.errors++;
        s17_g722_live_stop(2U);
        return;
    }
    stats.encoded++;
    stats.bytes_total+=(uint32_t)n;
    begin=DWT->CYCCNT;
#if S17_G722_LIVE_DECODE
    int samples=g722_decode(&decoder,output,packet,n);
    uint32_t dc=DWT->CYCCNT-begin;
    if(samples!=S17_FRAME_SAMPLES) {
        stats.errors++;
        s17_g722_live_stop(3U);
        return;
    }
    stats.decoded++;
#else
    uint32_t dc=0U;
#endif
    total_enc_cycles+=ec;
    total_dec_cycles+=dc;
    uint32_t e=cycles_to_us(ec),d=cycles_to_us(dc);
    if(e>stats.enc_max_us)stats.enc_max_us=e;
    if(d>stats.dec_max_us)stats.dec_max_us=d;
    if(e+d>stats.run_max_us)stats.run_max_us=e+d;
}
void s17_g722_live_stats(S17_G722LiveStats *out) {
    if(!out)return;
    *out=stats;
    out->dropped=frames.dropped_oldest;
    if(stats.encoded) {
        out->enc_avg_us=cycles_to_us((uint32_t)(total_enc_cycles/stats.encoded));
        out->dec_avg_us=cycles_to_us((uint32_t)(total_dec_cycles/stats.encoded));
    }
    /* Rates are the nominal continuous 50 packets/sec for a stream.
       This is a modeled RF workload, NOT measured wireless throughput.
       Header budget is configurable only in the future ESP32 transport. */
    out->payload_bps=50U*S17_PACKET_BYTES*8U;
    out->modeled_wire_bps=50U*(S17_PACKET_BYTES+S17_RADIO_HEADER_BYTES)*8U;
}
