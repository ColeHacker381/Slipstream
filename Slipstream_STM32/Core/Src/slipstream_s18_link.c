/* S18 STM32 prototype. I2S3 is a binary compressed-audio carrier, NOT PCM.
 * SPI2/I2S2 keeps WM8960 audio unchanged. USART2: ASCII control/status only.
 * Master: SPI3 I2S @ nominal 16kHz, stereo, 16 bits; ESP32 I2S RX slave.
 * DMA 1280 halfwords, each half = one 20ms wire frame (640 halfwords).
 * Each frame contains 8 header words, 80 G.722 words, zero padding.
 */
#include "slipstream_s18_link.h"
#include "slipstream_s13_frames.h"
#include "main.h"
#include "g722.h"
#include <stdio.h>
#include <string.h>
#define W 640U
#define Q 3U
I2S_HandleTypeDef s18_i2s;
DMA_HandleTypeDef s18_dma;
UART_HandleTypeDef s18_uart;
static uint16_t dma_words[W*2U];
static uint16_t queued[Q][88];
static volatile uint8_t q_head,q_tail,q_count;
static uint16_t seq;
static g722_encode_state_t encoder;
static S13_FrameAssembler assembler;
static int16_t pcm[320];
static uint8_t bytes[160];
static volatile S18_LinkStats st;
static uint64_t sum_enc_cycles;
static uint8_t uart_rx_byte,uart_line[48];
static volatile uint8_t uart_len,uart_ready;
static uint8_t uart_out[192];
static volatile uint8_t uart_tx_busy;
static uint32_t last_status;

static uint32_t us(uint32_t cycles) {
 return SystemCoreClock?(uint32_t)((uint64_t)cycles*1000000ULL/SystemCoreClock):0U;
}
static uint16_t crc16(const uint8_t *buf,unsigned n) {
 uint16_t c=0xffff;
 for(unsigned i=0;i<n;i++){c^=(uint16_t)buf[i]<<8;
  for(unsigned b=0;b<8;b++)c=(uint16_t)((c&0x8000U)?((c<<1)^0x1021U):(c<<1));}
 return c;
}
int s18_link_init(void) {
 GPIO_InitTypeDef gpio={0};
 __HAL_RCC_GPIOA_CLK_ENABLE();
 __HAL_RCC_GPIOC_CLK_ENABLE();
 __HAL_RCC_USART2_CLK_ENABLE();
 __HAL_RCC_SPI3_CLK_ENABLE();
 __HAL_RCC_DMA1_CLK_ENABLE();
 /* PA4 WS, PC10 CK, PC12 SD. 3.3V logic, no MCLK. */
 gpio.Pin=GPIO_PIN_4;gpio.Mode=GPIO_MODE_AF_PP;gpio.Pull=GPIO_NOPULL;
 gpio.Speed=GPIO_SPEED_FREQ_VERY_HIGH;gpio.Alternate=GPIO_AF6_SPI3;
 HAL_GPIO_Init(GPIOA,&gpio);
 gpio.Pin=GPIO_PIN_10|GPIO_PIN_12;HAL_GPIO_Init(GPIOC,&gpio);
 gpio.Pin=GPIO_PIN_2|GPIO_PIN_3;gpio.Alternate=GPIO_AF7_USART2;
 gpio.Pull=GPIO_PULLUP;HAL_GPIO_Init(GPIOA,&gpio);
 memset(&s18_i2s,0,sizeof(s18_i2s));
 s18_i2s.Instance=SPI3;
 s18_i2s.Init.Mode=I2S_MODE_MASTER_TX;
 s18_i2s.Init.Standard=I2S_STANDARD_PHILIPS;
 s18_i2s.Init.DataFormat=I2S_DATAFORMAT_16B;
 s18_i2s.Init.MCLKOutput=I2S_MCLKOUTPUT_DISABLE;
 s18_i2s.Init.AudioFreq=I2S_AUDIOFREQ_16K;
 s18_i2s.Init.CPOL=I2S_CPOL_LOW;
 s18_i2s.Init.ClockSource=I2S_CLOCK_PLL;
 s18_i2s.Init.FullDuplexMode=I2S_FULLDUPLEXMODE_DISABLE;
 if(HAL_I2S_Init(&s18_i2s)!=HAL_OK)return 0;
 memset(&s18_dma,0,sizeof(s18_dma));
 s18_dma.Instance=DMA1_Stream5;s18_dma.Init.Channel=DMA_CHANNEL_0;
 s18_dma.Init.Direction=DMA_MEMORY_TO_PERIPH;
 s18_dma.Init.PeriphInc=DMA_PINC_DISABLE;s18_dma.Init.MemInc=DMA_MINC_ENABLE;
 s18_dma.Init.PeriphDataAlignment=DMA_PDATAALIGN_HALFWORD;
 s18_dma.Init.MemDataAlignment=DMA_MDATAALIGN_HALFWORD;
 s18_dma.Init.Mode=DMA_CIRCULAR;s18_dma.Init.Priority=DMA_PRIORITY_MEDIUM;
 s18_dma.Init.FIFOMode=DMA_FIFOMODE_DISABLE;
 if(HAL_DMA_Init(&s18_dma)!=HAL_OK)return 0;
 __HAL_LINKDMA(&s18_i2s,hdmatx,s18_dma);
 memset(&s18_uart,0,sizeof(s18_uart));s18_uart.Instance=USART2;
 s18_uart.Init.BaudRate=115200;s18_uart.Init.WordLength=UART_WORDLENGTH_8B;
 s18_uart.Init.StopBits=UART_STOPBITS_1;s18_uart.Init.Parity=UART_PARITY_NONE;
 s18_uart.Init.Mode=UART_MODE_TX_RX;s18_uart.Init.HwFlowCtl=UART_HWCONTROL_NONE;
 s18_uart.Init.OverSampling=UART_OVERSAMPLING_16;
 if(HAL_UART_Init(&s18_uart)!=HAL_OK)return 0;
 HAL_NVIC_SetPriority(DMA1_Stream5_IRQn,4,0);HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
 HAL_NVIC_SetPriority(USART2_IRQn,6,0);HAL_NVIC_EnableIRQ(USART2_IRQn);
 memset((void *)&st,0,sizeof(st));memset(dma_words,0,sizeof(dma_words));
 q_head=q_tail=q_count=0;seq=0;sum_enc_cycles=0;last_status=HAL_GetTick();
 s13_frames_reset(&assembler);
 if(!g722_encode_init(&encoder,64000,0))return 0;
 /* Streaming OFF until ESP issues START on UART control channel. */
 st.active=0U;
 return HAL_UART_Receive_IT(&s18_uart,&uart_rx_byte,1U)==HAL_OK;
}
int s18_link_start(void) {
 return HAL_I2S_Transmit_DMA(&s18_i2s,dma_words,2U*W)==HAL_OK;
}
void s18_link_push(const int16_t *pcm160) {
 if(!st.active)return;
 if(s13_frames_push(&assembler,pcm160))st.made++;
 if(assembler.dropped_oldest)st.dropped=assembler.dropped_oldest;
}
void s18_link_run_one(void) {
 if(!st.active || !s13_frames_take(&assembler,pcm))return;
 uint32_t start=DWT->CYCCNT;
 int n=g722_encode(&encoder,bytes,pcm,320);
 uint32_t cycles=DWT->CYCCNT-start;
 if(n!=160){st.dropped++;return;}
 sum_enc_cycles+=cycles;st.encoded++;
 uint32_t duration=us(cycles);
 if(duration>st.encode_max_us)st.encode_max_us=duration;
 uint16_t frame[88];
 frame[0]=0xA55AU;frame[1]=0x1801U;frame[2]=seq++;
 frame[3]=160U;frame[4]=crc16(bytes,160U);
 frame[5]=0U;frame[6]=0U;frame[7]=0U;
 for(unsigned i=0;i<80;i++)frame[8+i]=(uint16_t)((uint16_t)bytes[2*i]<<8|bytes[2*i+1]);
 /* 176 bytes copied with IRQs masked to keep DMA consumer atomic. */
 uint32_t was=__get_PRIMASK();__disable_irq();
 if(q_count==Q)st.dropped++;
 else{
  memcpy(queued[q_head],frame,sizeof(frame));
  q_head=(uint8_t)((q_head+1U)%Q);q_count++;st.queued++;
  if(q_count>st.queue_high)st.queue_high=q_count;
 }
 if(!was)__enable_irq();
}
/* DMA completed one 20ms half; it is now free for its next revolution.
 * Always output a zero-padded 20ms slot, so the ESP can scan for sync.
 */
static void refill(unsigned half) {
 uint16_t *dst=&dma_words[half*W];memset(dst,0,W*sizeof(uint16_t));
 if(q_count && st.active) {
  memcpy(dst,queued[q_tail],88U*sizeof(uint16_t));
  q_tail=(uint8_t)((q_tail+1U)%Q);q_count--;st.sent++;
 }else st.idle++;
}
void DMA1_Stream5_IRQHandler(void){HAL_DMA_IRQHandler(&s18_dma);}
void USART2_IRQHandler(void){HAL_UART_IRQHandler(&s18_uart);}
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *h){
 if(h->Instance==SPI3)refill(0U);
}
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *h){
 if(h->Instance==SPI3)refill(1U);
}
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *h){
 if(h->Instance==SPI3)st.dma_errors++;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h){
 if(h->Instance!=USART2)return;
 uint8_t b=uart_rx_byte;
 if((b=='\n'||b=='\r') && uart_len && !uart_ready){
  uart_line[uart_len]=0;uart_ready=1;st.uart_commands++;
 }else if(!uart_ready && b!='\r' && b!='\n'){
  if(uart_len<sizeof(uart_line)-1U)uart_line[uart_len++]=b;
  else{uart_len=0;st.uart_errors++;}
 }
 (void)HAL_UART_Receive_IT(&s18_uart,&uart_rx_byte,1U);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h){
 if(h->Instance==USART2)uart_tx_busy=0U;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h){
 if(h->Instance==USART2) {
  st.uart_errors++;uart_tx_busy=0U;
  (void)HAL_UART_Receive_IT(&s18_uart,&uart_rx_byte,1U);
 }
}
void s18_link_get_stats(S18_LinkStats *out){
 if(!out)return;
 *out=st;
 if(st.encoded)out->encode_avg_us=us((uint32_t)(sum_enc_cycles/st.encoded));
}
static void report(void) {
 if(uart_tx_busy || s18_uart.gState!=HAL_UART_STATE_READY)return;
 S18_LinkStats v;s18_link_get_stats(&v);
 int n=snprintf((char*)uart_out,sizeof(uart_out),
 "S18 active=%lu made=%lu enc=%lu queued=%lu sent=%lu drop=%lu idle=%lu dma=%lu enc_us=%lu/%lu qhi=%lu cmds=%lu errors=%lu\n",
 (unsigned long)v.active,(unsigned long)v.made,(unsigned long)v.encoded,
 (unsigned long)v.queued,(unsigned long)v.sent,(unsigned long)v.dropped,
 (unsigned long)v.idle,(unsigned long)v.dma_errors,
 (unsigned long)v.encode_avg_us,(unsigned long)v.encode_max_us,
 (unsigned long)v.queue_high,(unsigned long)v.uart_commands,(unsigned long)v.uart_errors);
 if(n>0 && n<(int)sizeof(uart_out)){
  uart_tx_busy=1U;
  if(HAL_UART_Transmit_IT(&s18_uart,uart_out,(uint16_t)n)!=HAL_OK)uart_tx_busy=0;
 }
}
void s18_link_service(void){
 if(uart_ready){
  char cmd[48];
  uint32_t was=__get_PRIMASK();__disable_irq();
  memcpy(cmd,uart_line,sizeof(cmd));uart_ready=0;uart_len=0;
  if(!was)__enable_irq();
  if(strcmp(cmd,"START")==0)st.active=1U;
  else if(strcmp(cmd,"STOP")==0){
   st.active=0U;s13_frames_reset(&assembler);
   was=__get_PRIMASK();__disable_irq();
   q_head=q_tail=q_count=0;
   if(!was)__enable_irq();
  }else if(strcmp(cmd,"STATUS")!=0 && strcmp(cmd,"PING")!=0)st.uart_errors++;
  report();
 }
 if((uint32_t)(HAL_GetTick()-last_status)>=1000U){
  last_status=HAL_GetTick();report();
 }
}
