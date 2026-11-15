/* S15 startup-only 16kHz/64kbps G.722 wideband encode+decode benchmark */
#include "slipstream_s15_g722_bench.h"
#include "main.h"
#include "g722.h"
#include <string.h>
static g722_encode_state_t g722_e;
static g722_decode_state_t g722_d;
static int16_t source[320], restored[320];
static uint8_t compressed[160];
static uint32_t usec(uint32_t cyc)
{
  return SystemCoreClock ? (uint32_t)((uint64_t)cyc*1000000ULL/SystemCoreClock) : 0;
}
int s15_g722_benchmark(S15_G722Result *out)
{
  if (!out) return 0;
  memset(out,0,sizeof(*out));
  if (!g722_encode_init(&g722_e,64000,0) ||
      !g722_decode_init(&g722_d,64000,0)) {out->errors++;return 0;}
  uint64_t es=0,ds=0,ts=0,bs=0;
  for(uint32_t f=0;f<55;f++){
    /* Deterministic voice-like stimulus. Separate audio samples/packet
       quality evaluation is required for a fair codec comparison. */
    for(uint32_t i=0;i<320;i++){
      uint32_t t=f*320U+i;
      int32_t triangle=(int32_t)(t%83U)-41;
      source[i]=(int16_t)(triangle*80/(((t/320U)%5U)==0U?3:1));
    }
    uint32_t st=DWT->CYCCNT;
    int n=g722_encode(&g722_e,compressed,source,320);
    uint32_t e=DWT->CYCCNT-st;
    if(n!=160){out->errors++;break;}
    st=DWT->CYCCNT;
    int m=g722_decode(&g722_d,restored,compressed,n);
    uint32_t d=DWT->CYCCNT-st;
    if(m!=320){out->errors++;break;}
    if(f<5)continue;
    es+=e;ds+=d;ts+=(uint64_t)e+d;bs+=(uint32_t)n;
    uint32_t eu=usec(e),du=usec(d),tu=usec(e+d);
    if(eu>out->enc_max_us)out->enc_max_us=eu;
    if(du>out->dec_max_us)out->dec_max_us=du;
    if(tu>out->total_max_us)out->total_max_us=tu;
    if((uint32_t)n>out->max_bytes)out->max_bytes=(uint32_t)n;
    out->frames++;
  }
  if(out->frames){
    out->enc_avg_us=usec((uint32_t)(es/out->frames));
    out->dec_avg_us=usec((uint32_t)(ds/out->frames));
    out->total_avg_us=usec((uint32_t)(ts/out->frames));
    out->avg_bytes=(uint32_t)(bs/out->frames);
  }
  /* Upstream release() calls free(). These states are static, so NEVER
     release them; they are reset by init() on the next boot. */
  return out->frames==50 && out->errors==0;
}
