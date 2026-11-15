/* S18 STM32 -> ESP32 radio-UART physical transport (TX first).
 * Dedicated USART2 PA2 TX / PA3 RX, DMA1 Stream6 Channel4 TX.
 * Independent of USART3 ST-LINK debug and I2S DMA Stream3/4.
 *
 * Wire frame (174 bytes, all integer fields little endian):
 *   [0:2]   magic 'S','L'
 *   [2]     protocol version 1
 *   [3]     type 1 = G.722 16kHz 20ms mono
 *   [4:6]   sequence number
 *   [6:10]  HAL_GetTick milliseconds at packet submission
 *   [10:12] payload length 160
 *   [12:172] G.722 encoded bytes
 *   [172:174] CRC16-CCITT-FALSE over first 172 bytes (init FFFF, poly 1021)
 *
 * No real RF has been integrated. The downstream ESP32 must implement
 * matching framing/CRC/sequence checks; USART2 RX is reserved for S18 RX.
 */
#include "slipstream_s18_link.h"
#include "slipstream_s13_frames.h"
#include "main.h"
#include "g722.h"
#include <string.h>
#include <stdint.h>

static UART_HandleTypeDef link_uart;
static DMA_HandleTypeDef link_tx_dma;
static S13_FrameAssembler frame_queue;
static g722_encode_state_t encoder;
static int16_t pcm320[S18_FRAME_SAMPLES];
static uint8_t tx_packet[S18_WIRE_BYTES];
static S18_LinkStats counters;
static uint64_t codec_cycles_sum;
static uint16_t sequence;
static uint8_t enabled;

static uint32_t to_us(uint32_t cycles) {
    return SystemCoreClock ? (uint32_t)((uint64_t)cycles*1000000ULL/SystemCoreClock) : 0U;
}
static void le16(uint8_t *p,uint16_t v) {p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void le32(uint8_t *p,uint32_t v) {le16(p,(uint16_t)v);le16(p+2,(uint16_t)(v>>16));}
static uint16_t crc16(const uint8_t *p,uint32_t n) {
    uint16_t crc=0xffffU;
    for(uint32_t i=0;i<n;i++) {
        crc^=(uint16_t)((uint16_t)p[i]<<8);
        for(unsigned k=0;k<8;k++)crc=(uint16_t)((crc&0x8000U)?((crc<<1)^0x1021U):(crc<<1));
    }
    return crc;
}
static int configure_uart(void) {
    GPIO_InitTypeDef gpio={0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    gpio.Pin=GPIO_PIN_2|GPIO_PIN_3;
    gpio.Mode=GPIO_MODE_AF_PP;
    gpio.Pull=GPIO_PULLUP;
    gpio.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate=GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA,&gpio);

    /* HAL_UART_MspInit is Cube-generated for USART3 only. Manually
       configure USART2 and its DMA to keep S18 opt-in and independent. */
    memset(&link_uart,0,sizeof(link_uart));
    link_uart.Instance=USART2;
    link_uart.Init.BaudRate=S18_UART_BAUD;
    link_uart.Init.WordLength=UART_WORDLENGTH_8B;
    link_uart.Init.StopBits=UART_STOPBITS_1;
    link_uart.Init.Parity=UART_PARITY_NONE;
    link_uart.Init.Mode=UART_MODE_TX_RX;
    link_uart.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    link_uart.Init.OverSampling=UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&link_uart)!=HAL_OK)return 0;

    memset(&link_tx_dma,0,sizeof(link_tx_dma));
    link_tx_dma.Instance=DMA1_Stream6;
    link_tx_dma.Init.Channel=DMA_CHANNEL_4;
    link_tx_dma.Init.Direction=DMA_MEMORY_TO_PERIPH;
    link_tx_dma.Init.PeriphInc=DMA_PINC_DISABLE;
    link_tx_dma.Init.MemInc=DMA_MINC_ENABLE;
    link_tx_dma.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;
    link_tx_dma.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;
    link_tx_dma.Init.Mode=DMA_NORMAL;
    link_tx_dma.Init.Priority=DMA_PRIORITY_LOW;
    link_tx_dma.Init.FIFOMode=DMA_FIFOMODE_DISABLE;
    if(HAL_DMA_Init(&link_tx_dma)!=HAL_OK)return 0;
    __HAL_LINKDMA(&link_uart,hdmatx,link_tx_dma);
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn,6U,0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn,6U,0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    return 1;
}
int s18_link_init(void) {
    memset(&counters,0,sizeof(counters));
    s13_frames_reset(&frame_queue);
    codec_cycles_sum=0U;sequence=0U;enabled=0U;
    if(!g722_encode_init(&encoder,64000,0))return 0;
    if(!configure_uart())return 0;
    enabled=1U;
    return 1;
}
void s18_link_push(const int16_t *pcm160) {
    if(!enabled)return;
    if(s13_frames_push(&frame_queue,pcm160))counters.frames_produced++;
    counters.queue_drops=frame_queue.dropped_oldest;
}
void s18_link_stop(uint32_t reason) {
    if(!enabled)return;
    enabled=0;
    counters.stop=1;
    counters.stop_reason=reason;
    /* Allow an already-started UART DMA transfer to finish. */
}
void s18_link_run_one(void) {
    if(!enabled)return;
    /* Keep at most two assembled frames; never wait for TX or block DSP. */
    if(!s13_frames_take(&frame_queue,pcm320))return;
    uint32_t start=DWT->CYCCNT;
    uint8_t encoded[S18_PAYLOAD_BYTES];
    int size=g722_encode(&encoder,encoded,pcm320,S18_FRAME_SAMPLES);
    uint32_t elapsed=DWT->CYCCNT-start;
    if(size!=(int)S18_PAYLOAD_BYTES) {
        counters.tx_errors++;
        s18_link_stop(2U);
        return;
    }
    counters.frames_encoded++;
    codec_cycles_sum+=elapsed;
    uint32_t usec=to_us(elapsed);
    if(usec>counters.codec_max_us)counters.codec_max_us=usec;
    /* Drop the already-encoded frame if UART has not finished. No blocking
       or buffer mutation while DMA is using tx_packet. */
    if(link_uart.gState!=HAL_UART_STATE_READY) {
        counters.tx_busy_drops++;
        return;
    }
    uint32_t submit=DWT->CYCCNT;
    tx_packet[0]='S';tx_packet[1]='L';tx_packet[2]=1;tx_packet[3]=1;
    le16(&tx_packet[4],sequence);
    uint32_t timestamp=HAL_GetTick();
    le32(&tx_packet[6],timestamp);
    le16(&tx_packet[10],S18_PAYLOAD_BYTES);
    memcpy(&tx_packet[12],encoded,S18_PAYLOAD_BYTES);
    le16(&tx_packet[S18_HEADER_BYTES+S18_PAYLOAD_BYTES],
         crc16(tx_packet,S18_HEADER_BYTES+S18_PAYLOAD_BYTES));
    if(HAL_UART_Transmit_DMA(&link_uart,tx_packet,S18_WIRE_BYTES)!=HAL_OK) {
        counters.tx_errors++;
        return;
    }
    counters.seq_last=sequence;
    sequence++;
    counters.last_timestamp_ms=timestamp;
    counters.frames_sent++;
    counters.bytes_sent+=S18_WIRE_BYTES;
    uint32_t submit_us=to_us(DWT->CYCCNT-submit);
    if(submit_us>counters.max_tx_submit_us)counters.max_tx_submit_us=submit_us;
}
void s18_link_get_stats(S18_LinkStats *out) {
    if(!out)return;
    *out=counters;
    out->queue_drops=frame_queue.dropped_oldest;
    if(counters.frames_encoded)out->codec_avg_us=to_us((uint32_t)(codec_cycles_sum/counters.frames_encoded));
}
void DMA1_Stream6_IRQHandler(void) {HAL_DMA_IRQHandler(&link_tx_dma);}
void USART2_IRQHandler(void) {HAL_UART_IRQHandler(&link_uart);}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance==USART2)counters.tx_dma_completed++;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance==USART2)counters.uart_errors++;
}
