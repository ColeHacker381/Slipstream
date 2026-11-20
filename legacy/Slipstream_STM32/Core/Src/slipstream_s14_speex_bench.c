/* Real Speex 1.2.1 fixed-point wideband benchmark, startup only.
 * Matches Opus's 16kHz mono, 320-sample frames and measured cycle timing.
 * Does not start DMA or change any WM8960/DSP audio routing.
 */
#include "slipstream_s14_speex_bench.h"
#include "speex/speex.h"
#include "speex/speex_bits.h"
#include "main.h"
#include <string.h>

#define S14_N 50U
#define S14_WARMUP 5U
#define S14_PACKET_CAP 512U

static int16_t s14_input[320];
static int16_t s14_output[320];
static char s14_packet[S14_PACKET_CAP];

static uint32_t s14_us(uint32_t cycles)
{
    return SystemCoreClock ?
        (uint32_t)(((uint64_t)cycles * 1000000ULL)/SystemCoreClock) : 0U;
}

int s14_speex_benchmark(S14_SpeexBenchResult *result)
{
    if (!result) return 0;
    memset(result, 0, sizeof(*result));

    void *enc=speex_encoder_init(&speex_wb_mode);
    void *dec=speex_decoder_init(&speex_wb_mode);
    SpeexBits ebits, dbits;
    if (!enc || !dec) {
        if(enc) speex_encoder_destroy(enc);
        if(dec) speex_decoder_destroy(dec);
        result->errors++;
        return 0;
    }
    speex_bits_init(&ebits);
    speex_bits_init(&dbits);

    int frame=0, rate=16000, quality=S14_SPEEX_QUALITY;
    int complexity=S14_SPEEX_COMPLEXITY, bitrate=20000, vbr=0, dtx=0, enh=0;
    int good=1;
    if(speex_encoder_ctl(enc,SPEEX_SET_SAMPLING_RATE,&rate)!=0)good=0;
    if(speex_decoder_ctl(dec,SPEEX_SET_SAMPLING_RATE,&rate)!=0)good=0;
    if(speex_encoder_ctl(enc,SPEEX_SET_QUALITY,&quality)!=0)good=0;
    if(speex_encoder_ctl(enc,SPEEX_SET_COMPLEXITY,&complexity)!=0)good=0;
    if(speex_encoder_ctl(enc,SPEEX_SET_VBR,&vbr)!=0)good=0;
    if(speex_encoder_ctl(enc,SPEEX_SET_DTX,&dtx)!=0)good=0;
    if(speex_encoder_ctl(enc,SPEEX_SET_BITRATE,&bitrate)!=0)good=0;
    if(speex_encoder_ctl(enc,SPEEX_GET_BITRATE,&bitrate)!=0)good=0;
    if(speex_decoder_ctl(dec,SPEEX_SET_ENH,&enh)!=0)good=0;
    if(speex_encoder_ctl(enc,SPEEX_GET_FRAME_SIZE,&frame)!=0 || frame!=320)good=0;
    result->frame_samples=(uint32_t)frame;
    result->target_bitrate=(uint32_t)bitrate;

    uint64_t ec_sum=0, dc_sum=0, total_sum=0, bytes_sum=0;
    if(good) for(uint32_t f=0; f<S14_N+S14_WARMUP; ++f)
    {
        /* Same lightweight deterministic voice-like variable stimulus each run.
           Not a substitute for real microphone speech quality benchmarks. */
        for(uint32_t i=0;i<320;i++) {
            uint32_t t=f*320U+i;
            int32_t triangle=(int32_t)(t%83U)-41;
            int32_t envelope=((t/320U)%5U==0U)?3:1;
            s14_input[i]=(int16_t)(triangle*80/envelope);
        }
        speex_bits_reset(&ebits);
        uint32_t start=DWT->CYCCNT;
        int er=speex_encode_int(enc,s14_input,&ebits);
        uint32_t ec=DWT->CYCCNT-start;
        if(er<0) {result->errors++;break;}
        int size=speex_bits_nbytes(&ebits);
        if(size<=0 || size>(int)S14_PACKET_CAP) {result->errors++;break;}
        int n=speex_bits_write(&ebits,s14_packet,(int)sizeof(s14_packet));
        if(n!=size) {result->errors++;break;}
        speex_bits_read_from(&dbits,s14_packet,n);
        start=DWT->CYCCNT;
        int dr=speex_decode_int(dec,&dbits,s14_output);
        uint32_t dc=DWT->CYCCNT-start;
        if(dr!=0) {result->errors++;break;}
        if(f<S14_WARMUP)continue;
        ec_sum+=ec; dc_sum+=dc; total_sum+=(uint64_t)ec+dc;
        bytes_sum+=(uint32_t)n;
        uint32_t eu=s14_us(ec),du=s14_us(dc),tu=s14_us(ec+dc);
        if(eu>result->encode_max_us)result->encode_max_us=eu;
        if(du>result->decode_max_us)result->decode_max_us=du;
        if(tu>result->total_max_us)result->total_max_us=tu;
        if((uint32_t)n>result->packet_max_bytes)result->packet_max_bytes=(uint32_t)n;
        result->frames++;
    }
    if(result->frames){
        result->encode_avg_us=s14_us((uint32_t)(ec_sum/result->frames));
        result->decode_avg_us=s14_us((uint32_t)(dc_sum/result->frames));
        result->total_avg_us=s14_us((uint32_t)(total_sum/result->frames));
        result->packet_avg_bytes=(uint32_t)(bytes_sum/result->frames);
    }
    result->ok=good && (result->frames==S14_N) && !result->errors;
    speex_bits_destroy(&ebits);
    speex_bits_destroy(&dbits);
    speex_encoder_destroy(enc);
    speex_decoder_destroy(dec);
    return (int)result->ok;
}
