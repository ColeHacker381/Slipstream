/* S16 Codec 2 3200bps real library benchmark, startup-only.
 * Full 16kHz/20ms pipeline: FIR downsample to 8kHz, 160-sample codec
 * encode/decode, FIR upsample to 16kHz. Timings include resampling.
 * This does not intercept or change microphone DMA or speaker playback.
 */
#include "slipstream_s16_codec2_bench.h"
#include "main.h"
#include "codec2.h"
#include <math.h>
#include <string.h>

#define S16_TAPS 31
#define S16_MEASURE 50U
#define S16_WARMUP 5U
static float coeff[S16_TAPS];
static int16_t source16[320], source8[160], result8[160], result16[320];
static float down_work[350], up_work[350];
static unsigned char packet[64];
static uint32_t usec(uint32_t cycles) {
    return SystemCoreClock ? (uint32_t)((uint64_t)cycles*1000000ULL/SystemCoreClock) : 0U;
}
static int16_t clip16(float f) {
    if(f>32767.f) return 32767;
    if(f< -32768.f) return -32768;
    return (int16_t)f;
}
static void prepare_filter(void) {
    const float pi=3.14159265358979323846f;
    const float fc=3500.f/16000.f; /* Anti-alias below 8kHz Nyquist (4kHz) */
    float sum=0.f;
    for(int j=0;j<S16_TAPS;j++) {
        int k=j-(S16_TAPS-1)/2;
        float sinc=k==0 ? 2.f*fc : sinf(2.f*pi*fc*k)/(pi*k);
        float window=.54f-.46f*cosf(2.f*pi*j/(S16_TAPS-1));
        coeff[j]=sinc*window;
        sum+=coeff[j];
    }
    for(int j=0;j<S16_TAPS;j++) coeff[j]/=sum;
}
static void downsample(void) {
    for(int i=0;i<320;i++) down_work[30+i]=(float)source16[i];
    for(int i=0;i<160;i++) {
        float sum=0;
        for(int k=0;k<31;k++) sum+=coeff[k]*down_work[30+i*2-k];
        source8[i]=clip16(sum);
    }
    memcpy(down_work,down_work+320,30*sizeof(float));
}
static void upsample(void) {
    for(int i=0;i<160;i++) {
        up_work[30+i*2]=(float)result8[i];
        up_work[30+i*2+1]=0.f;
    }
    for(int i=0;i<320;i++) {
        float sum=0;
        for(int k=0;k<31;k++) sum+=coeff[k]*up_work[30+i-k];
        result16[i]=clip16(2.f*sum);
    }
    memcpy(up_work,up_work+320,30*sizeof(float));
}
int s16_codec2_benchmark(S16_Codec2BenchResult *out) {
    if(!out) return 0;
    memset(out,0,sizeof(*out));
    memset(down_work,0,sizeof(down_work));
    memset(up_work,0,sizeof(up_work));
    prepare_filter();
    struct CODEC2 *codec=codec2_create(CODEC2_MODE_3200);
    if(!codec) {out->errors++;return 0;}
    int n=codec2_samples_per_frame(codec);
    int bytes=codec2_bytes_per_frame(codec);
    out->samples8k=(uint32_t)n;
    out->packet_bytes=(uint32_t)bytes;
    if(n!=160 || bytes!=8 || bytes>(int)sizeof(packet)) {
        out->errors++;
        codec2_destroy(codec);
        return 0;
    }
    uint64_t sum_down=0,sum_enc=0,sum_dec=0,sum_up=0,sum_all=0;
    for(uint32_t f=0;f<S16_WARMUP+S16_MEASURE;f++) {
        /* Identical deterministic stimulus to S14/S15 tests. */
        for(uint32_t i=0;i<320;i++) {
            uint32_t t=f*320U+i;
            int32_t triangle=(int32_t)(t%83U)-41;
            source16[i]=(int16_t)(triangle*80/(((t/320U)%5U)==0U?3:1));
        }
        uint32_t t=DWT->CYCCNT;
        downsample();
        uint32_t d0=DWT->CYCCNT-t;
        t=DWT->CYCCNT;
        codec2_encode(codec,packet,source8);
        uint32_t e=DWT->CYCCNT-t;
        t=DWT->CYCCNT;
        codec2_decode(codec,result8,packet);
        uint32_t d=DWT->CYCCNT-t;
        t=DWT->CYCCNT;
        upsample();
        uint32_t u=DWT->CYCCNT-t;
        if(f<S16_WARMUP)continue;
        uint32_t total=d0+e+d+u;
        sum_down+=d0;sum_enc+=e;sum_dec+=d;sum_up+=u;sum_all+=total;
        if(usec(d0)>out->down_max_us)out->down_max_us=usec(d0);
        if(usec(e)>out->enc_max_us)out->enc_max_us=usec(e);
        if(usec(d)>out->dec_max_us)out->dec_max_us=usec(d);
        if(usec(u)>out->up_max_us)out->up_max_us=usec(u);
        if(usec(total)>out->total_max_us)out->total_max_us=usec(total);
        out->frames++;
    }
    codec2_destroy(codec);
    if(out->frames) {
        out->down_avg_us=usec((uint32_t)(sum_down/out->frames));
        out->enc_avg_us=usec((uint32_t)(sum_enc/out->frames));
        out->dec_avg_us=usec((uint32_t)(sum_dec/out->frames));
        out->up_avg_us=usec((uint32_t)(sum_up/out->frames));
        out->total_avg_us=usec((uint32_t)(sum_all/out->frames));
    }
    return out->frames==S16_MEASURE && out->errors==0;
}
