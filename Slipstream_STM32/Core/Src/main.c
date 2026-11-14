/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_i2s2_ext_rx;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE BEGIN 0 */


/*
 * Slipstream S7 — ESP32-compatible CMSIS-DSP port
 *
 * I2S:   48 kHz, 16-bit stereo
 * Voice: 16 kHz, 16-bit mono
 * DMA:   480 stereo frames per half
 *
 * S7_MODE:
 * 0 = direct 48 kHz loopback
 * 1 = original FIR resampler only
 * 2 = original FIR resampler + original HPF/LPF
 */
#define S7_MODE 2

#define WM8960_ADDR (0x1A << 1)

#define S5_FRAMES_PER_HALF 480U
#define S5_HALF_SAMPLES     960U
#define S5_DMA_SAMPLES     1920U

#define S7_VOICE_SAMPLES    160U
#define S7_FIR_TAPS          33U
#define S7_RATIO             3U

#define S7_PLAYBACK_GAIN    1.0f

static uint16_t s5_tx[S5_DMA_SAMPLES] = {0};
static uint16_t s5_rx[S5_DMA_SAMPLES] = {0};

static volatile uint32_t s5_half_count = 0;
static volatile uint32_t s5_full_count = 0;
static volatile uint32_t s5_peak = 0;
static volatile uint32_t s5_dma_errors = 0;
static volatile uint32_t s5_last_error = 0;

/*
 * Each counter advances when the relevant RX
 * DMA half has completed.
 *
 * The main loop processes the newest completed
 * half. Counts let us detect skipped blocks.
 */
static volatile uint32_t s7_ready_count[2] = {0, 0};
static uint32_t s7_done_count[2] = {0, 0};

static volatile uint32_t s7_irq_blocks = 0;

static uint32_t s7_processed_blocks = 0;
static uint32_t s7_missed_blocks = 0;
static uint32_t s7_deadline_misses = 0;
static uint32_t s7_voice_samples = 0;

static uint32_t s7_input_peak = 0;
static uint32_t s7_output_peak = 0;
static uint32_t s7_last_cycles = 0;
static uint32_t s7_max_cycles = 0;

/* -----------------------------------------
 * WM8960 — unchanged from working S5
 * ----------------------------------------- */

static HAL_StatusTypeDef wm8960_write(
    uint8_t reg,
    uint16_t value)
{
    uint8_t data[2];

    value &= 0x01FFU;

    data[0] = (uint8_t)(
        (reg << 1) | ((value >> 8) & 1U)
    );
    data[1] = (uint8_t)(value & 0xFFU);

    return HAL_I2C_Master_Transmit(
        &hi2c1,
        WM8960_ADDR,
        data,
        2,
        100
    );
}

static HAL_StatusTypeDef wm8960_headphone_init(void)
{
    HAL_StatusTypeDef status;

    #define WM_WRITE(reg, value)                \
        do {                                    \
            status = wm8960_write(reg, value);  \
            if (status != HAL_OK) return status;\
        } while (0)

    WM_WRITE(0x0F, 0x000);
    HAL_Delay(10);

    WM_WRITE(0x19, 0x0C0);
    HAL_Delay(50);

    WM_WRITE(0x34, 0x038);
    WM_WRITE(0x35, 0x031);
    WM_WRITE(0x36, 0x026);
    WM_WRITE(0x37, 0x0E8);

    WM_WRITE(0x04, 0x005);
    WM_WRITE(0x08, 0x1C4);
    WM_WRITE(0x07, 0x002);

    WM_WRITE(0x22, 0x150);
    WM_WRITE(0x25, 0x150);
    WM_WRITE(0x2F, 0x00C);

    WM_WRITE(0x1A, 0x1E3);
    WM_WRITE(0x05, 0x000);

    WM_WRITE(0x02, 0x165);
    WM_WRITE(0x03, 0x165);

    #undef WM_WRITE
    return HAL_OK;
}

static HAL_StatusTypeDef wm8960_enable_microphone(void)
{
    HAL_StatusTypeDef status;

    #define MIC_WRITE(reg, value)               \
        do {                                    \
            status = wm8960_write(reg, value);  \
            if (status != HAL_OK) return status;\
        } while (0)

    MIC_WRITE(0x19, 0x0EA);
    HAL_Delay(50);

    MIC_WRITE(0x2F, 0x02C);
    MIC_WRITE(0x30, 0x001);
    MIC_WRITE(0x00, 0x12F);
    MIC_WRITE(0x20, 0x148);
    MIC_WRITE(0x15, 0x1C3);

    #undef MIC_WRITE
    return HAL_OK;
}

/* -----------------------------------------
 * Original ESP32 FIR coefficients
 * ----------------------------------------- */

static const float32_t s7_fir[33] = {
     0.0012146106f,
     0.0023260760f,
     0.0015351378f,
    -0.0031749238f,
    -0.0100945219f,
    -0.0101718408f,
     0.0023477392f,
     0.0218961922f,
     0.0294725790f,
     0.0076512116f,
    -0.0375374010f,
    -0.0658048924f,
    -0.0323882860f,
     0.0736134608f,
     0.2064548135f,
     0.2988090563f,
     0.2988090563f,
     0.2988090563f,
     0.2064548135f,
     0.0736134608f,
    -0.0323882860f,
    -0.0658048924f,
    -0.0375374010f,
     0.0076512116f,
     0.0294725790f,
     0.0218961922f,
     0.0023477392f,
    -0.0101718408f,
    -0.0100945219f,
    -0.0031749238f,
     0.0015351378f,
     0.0023260760f,
     0.0012146106f
};

/*
 * Both CMSIS FIR implementations consume
 * time-reversed coefficients relative to
 * the conventional FIR representation.
 *
 * This particular FIR is symmetric, so the
 * same array is valid in either order.
 */
static arm_fir_decimate_instance_f32 s7_decimator;
static arm_fir_interpolate_instance_f32 s7_interpolator;

static float32_t s7_dec_state[
    S7_FIR_TAPS + S5_FRAMES_PER_HALF - 1U
];

static float32_t s7_int_state[
    (S7_FIR_TAPS / S7_RATIO)
    + S7_VOICE_SAMPLES - 1U
];

/* -----------------------------------------
 * Audio processing buffers
 * ----------------------------------------- */

static float32_t s7_input_48k[S5_FRAMES_PER_HALF];
static float32_t s7_decimated_16k[S7_VOICE_SAMPLES];
static float32_t s7_playback_48k[S5_FRAMES_PER_HALF];

static int16_t s7_voice_16k[S7_VOICE_SAMPLES];

/*
 * Keep a full stereo output staging block.
 * This is important: processing must finish
 * before we copy results into the DMA buffer.
 */
static uint16_t s7_tx_stage[S5_HALF_SAMPLES];

/* -----------------------------------------
 * Frozen voice biquads
 * ----------------------------------------- */

typedef struct
{
    float b0, b1, b2;
    float a1, a2;
    float x1, x2;
    float y1, y2;
} S7_Biquad;

static S7_Biquad s7_hpf = {
    .b0 =  0.9726138985f,
    .b1 = -1.9452277970f,
    .b2 =  0.9726138985f,
    .a1 = -1.9444776578f,
    .a2 =  0.9459779362f
};

static S7_Biquad s7_lpf = {
    .b0 = 0.5690355937f,
    .b1 = 1.1380711875f,
    .b2 = 0.5690355937f,
    .a1 = 0.9428090416f,
    .a2 = 0.3333333333f
};

static inline float s7_biquad_process(
    S7_Biquad *f,
    float x)
{
    float y =
        f->b0 * x +
        f->b1 * f->x1 +
        f->b2 * f->x2 -
        f->a1 * f->y1 -
        f->a2 * f->y2;

    f->x2 = f->x1;
    f->x1 = x;

    f->y2 = f->y1;
    f->y1 = y;

    return y;
}

static inline int16_t s7_clip_pcm(float x)
{
    if (x > 32767.0f) return 32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)x;
}

static inline uint32_t s7_abs_pcm(int32_t x)
{
    return (x < 0) ? (uint32_t)(-x) : (uint32_t)x;
}

/*
 * Initialize both CMSIS resamplers.
 *
 * Decimator:
 *   480 samples -> 160 samples
 *
 * Interpolator:
 *   160 samples -> 480 samples
 */
static int s7_resampler_init(void)
{
    arm_status dec_status;
    arm_status int_status;

    dec_status = arm_fir_decimate_init_f32(
        &s7_decimator,
        S7_FIR_TAPS,
        S7_RATIO,
        s7_fir,
        s7_dec_state,
        S5_FRAMES_PER_HALF
    );

    int_status = arm_fir_interpolate_init_f32(
        &s7_interpolator,
        S7_RATIO,
        S7_FIR_TAPS,
        s7_fir,
        s7_int_state,
        S7_VOICE_SAMPLES
    );

    return (
        dec_status == ARM_MATH_SUCCESS &&
        int_status == ARM_MATH_SUCCESS
    );
}

/*
 * Process a completed DMA half in main context.
 *
 * Returns 1 when the output was committed safely.
 * Returns 0 when processing missed the buffer window.
 *
 * Both DMA half callbacks simply signal readiness.
 */
static int s7_process_half(uint32_t half, uint32_t sequence)
{
    uint32_t offset = half * S5_HALF_SAMPLES;
    uint32_t start_cycles = DWT->CYCCNT;
    uint32_t irq_at_start = s7_irq_blocks;

    uint32_t input_peak = 0;
    uint32_t output_peak = 0;

    /*
     * Copy microphone data out of the completed
     * RX half immediately.
     */
    for (uint32_t i = 0; i < S5_FRAMES_PER_HALF; i++)
    {
        int16_t mic =
            (int16_t)s5_rx[offset + i * 2U];

        s7_input_48k[i] = (float32_t)mic;

        uint32_t magnitude = s7_abs_pcm(mic);
        if (magnitude > input_peak)
            input_peak = magnitude;
    }

#if S7_MODE == 0

    for (uint32_t i = 0; i < S5_FRAMES_PER_HALF; i++)
    {
        int16_t pcm = s7_clip_pcm(
            s7_input_48k[i] * S7_PLAYBACK_GAIN
        );

        s7_tx_stage[i * 2U] = (uint16_t)pcm;
        s7_tx_stage[i * 2U + 1U] = (uint16_t)pcm;

        uint32_t magnitude = s7_abs_pcm(pcm);
        if (magnitude > output_peak)
            output_peak = magnitude;
    }

#else

    /*
     * Exact 48 -> 16 kHz factor, using
     * the original ESP32 FIR coefficients.
     */
    arm_fir_decimate_f32(
        &s7_decimator,
        s7_input_48k,
        s7_decimated_16k,
        S5_FRAMES_PER_HALF
    );

    /*
     * Match the ESP32 conversion to int16
     * after downsampling.
     */
    for (uint32_t i = 0; i < S7_VOICE_SAMPLES; i++)
    {
        s7_voice_16k[i] =
            s7_clip_pcm(s7_decimated_16k[i]);
    }

#if S7_MODE == 2
    /*
     * Original ESP32 H6 filter order.
     */
    for (uint32_t i = 0; i < S7_VOICE_SAMPLES; i++)
    {
        float sample = (float)s7_voice_16k[i];

        sample = s7_biquad_process(&s7_hpf, sample);
        sample = s7_biquad_process(&s7_lpf, sample);

        s7_voice_16k[i] = s7_clip_pcm(sample);
    }
#endif

    /*
     * Match ESP32's 16-bit voice input into
     * the interpolation stage.
     */
    for (uint32_t i = 0; i < S7_VOICE_SAMPLES; i++)
    {
        s7_decimated_16k[i] =
            (float32_t)s7_voice_16k[i];
    }

    arm_fir_interpolate_f32(
        &s7_interpolator,
        s7_decimated_16k,
        s7_playback_48k,
        S7_VOICE_SAMPLES
    );

    /*
     * CMSIS interpolation performs zero-stuffing
     * and FIR filtering but does not supply the
     * interpolation gain compensation.
     *
     * Multiply by the factor of 3, as on ESP32.
     */
    for (uint32_t i = 0; i < S5_FRAMES_PER_HALF; i++)
    {
        int16_t pcm = s7_clip_pcm(
            s7_playback_48k[i] *
            (float)S7_RATIO *
            S7_PLAYBACK_GAIN
        );

        s7_tx_stage[i * 2U] = (uint16_t)pcm;
        s7_tx_stage[i * 2U + 1U] = (uint16_t)pcm;

        uint32_t magnitude = s7_abs_pcm(pcm);
        if (magnitude > output_peak)
            output_peak = magnitude;
    }

#endif

    uint32_t elapsed = DWT->CYCCNT - start_cycles;

    s7_last_cycles = elapsed;
    if (elapsed > s7_max_cycles)
        s7_max_cycles = elapsed;

    /*
     * One DMA half represents 10 ms.
     * Require margin for copying the completed
     * output to the transmit buffer.
     */
    uint32_t deadline_cycles = SystemCoreClock / 100U;

    if (elapsed >= (deadline_cycles * 95U) / 100U ||
        s7_ready_count[half] != sequence ||
        s7_irq_blocks != irq_at_start)
    {
        s7_deadline_misses++;
        return 0;
    }

    /*
     * Check that DMA hasn't already advanced
     * into the region we're about to overwrite.
     */
    uint32_t other_half = half ^ 1U;

    /*
     * A subsequent DMA half-completion event
     * means the writable window has closed.
     *
     * We use the aggregate callback count
     * recorded at processing start (below in
     * the main loop) for further diagnosis.
     */
    (void)other_half;

    memcpy(
        &s5_tx[offset],
        s7_tx_stage,
        sizeof(s7_tx_stage)
    );

    if (input_peak > s7_input_peak)
        s7_input_peak = input_peak;

    if (output_peak > s7_output_peak)
        s7_output_peak = output_peak;

    s7_processed_blocks++;

#if S7_MODE != 0
    s7_voice_samples += S7_VOICE_SAMPLES;
#endif

    return 1;
}


/* USER CODE END 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  /* The MCU's OTG-FS clock is not 48 MHz at 100 MHz SYSCLK.
     ST-LINK SWD and its USART3 virtual COM port are independent. */
#if !defined(SLIPSTREAM_100MHZ)
  MX_USB_OTG_FS_PCD_Init();
#endif
  MX_I2C1_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */

  const char *boot_msg =
      "\r\n[Slipstream] S7 CMSIS-DSP port\r\n";

  HAL_UART_Transmit(
      &huart3,
      (uint8_t *)boot_msg,
      strlen(boot_msg),
      100
  );

  /* Restore known-good codec initialization */
  HAL_StatusTypeDef status = wm8960_headphone_init();

  if (status != HAL_OK)
  {
      const char *msg = "[S7] Headphone init failed\r\n";

      HAL_UART_Transmit(
          &huart3,
          (uint8_t *)msg,
          strlen(msg),
          100
      );

      Error_Handler();
  }

  status = wm8960_enable_microphone();

  if (status != HAL_OK)
  {
      const char *msg = "[S7] Microphone init failed\r\n";

      HAL_UART_Transmit(
          &huart3,
          (uint8_t *)msg,
          strlen(msg),
          100
      );

      Error_Handler();
  }

  /* Initialize ARM CMSIS-DSP filters */
  if (!s7_resampler_init())
  {
      const char *msg =
          "[S7] CMSIS resampler init failed\r\n";

      HAL_UART_Transmit(
          &huart3,
          (uint8_t *)msg,
          strlen(msg),
          100
      );

      Error_Handler();
  }

  if (hi2s2.hdmatx == NULL || hi2s2.hdmarx == NULL)
  {
      const char *msg =
          "[S7] DMA handles missing\r\n";

      HAL_UART_Transmit(
          &huart3,
          (uint8_t *)msg,
          strlen(msg),
          100
      );

      Error_Handler();
  }

  /* Start core-cycle profiling */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Start continuous circular DMA */
  status = HAL_I2SEx_TransmitReceive_DMA(
      &hi2s2,
      s5_tx,
      s5_rx,
      S5_DMA_SAMPLES
  );

  if (status != HAL_OK)
  {
      const char *msg = "[S7] DMA start failed\r\n";

      HAL_UART_Transmit(
          &huart3,
          (uint8_t *)msg,
          strlen(msg),
          100
      );

      Error_Handler();
  }

  char ready_msg[120];

  int ready_len = snprintf(
      ready_msg,
      sizeof(ready_msg),
      "[S7] Running mode=%d CPU=%lu Hz\r\n",
      S7_MODE,
      (unsigned long)SystemCoreClock
  );

  if (ready_len > 0)
  {
      HAL_UART_Transmit(
          &huart3,
          (uint8_t *)ready_msg,
          (uint16_t)ready_len,
          100
      );
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    static uint32_t last_report = 0;
    static uint32_t previous_irqs = 0;
    static uint32_t previous_voice = 0;

    uint32_t processed_this_iteration = 0;

    /*
    * Service completed DMA halves.
    *
    * One half becomes ready every 10 ms.
    * Never deliberately sleep while audio is
    * waiting to be processed.
    */
    for (uint32_t half = 0; half < 2U; half++)
    {
        uint32_t ready = s7_ready_count[half];
        uint32_t done = s7_done_count[half];

        if (ready == done)
            continue;

        /*
        * If this half has completed more than once
        * without processing, an audio block was lost.
        */
        if (ready > done + 1U)
        {
            s7_missed_blocks += ready - done - 1U;
        }

        /*
        * Only process the newest available block.
        */
        s7_done_count[half] = ready;

        (void)s7_process_half(half, ready);

        processed_this_iteration++;
    }

    /*
    * UART report once per HAL second.
    * DMA callbacks are now short, so SysTick
    * should no longer be heavily starved.
    */
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_report) >= 1000U)
    {
        last_report = now;

        uint32_t irq_total = s7_irq_blocks;
        uint32_t irqs_per_second =
            irq_total - previous_irqs;
        previous_irqs = irq_total;

        uint32_t voice_total = s7_voice_samples;
        uint32_t voice_per_second =
            voice_total - previous_voice;
        previous_voice = voice_total;

        uint32_t processing_us = 0;
        uint32_t max_processing_us = 0;

        if (SystemCoreClock != 0U)
        {
            processing_us = (uint32_t)(
                ((uint64_t)s7_last_cycles * 1000000ULL) /
                SystemCoreClock
            );

            max_processing_us = (uint32_t)(
                ((uint64_t)s7_max_cycles * 1000000ULL) /
                SystemCoreClock
            );
        }

        char msg[220];

        int len = snprintf(
            msg,
            sizeof(msg),
            "[S7] Mode=%d IRQ/s=%lu Voice/s=%lu "
            "In=%lu Out=%lu "
            "Proc=%luus Max=%luus "
            "Missed=%lu Late=%lu DMAErr=%lu\r\n",
            S7_MODE,
            (unsigned long)irqs_per_second,
            (unsigned long)voice_per_second,
            (unsigned long)s7_input_peak,
            (unsigned long)s7_output_peak,
            (unsigned long)processing_us,
            (unsigned long)max_processing_us,
            (unsigned long)s7_missed_blocks,
            (unsigned long)s7_deadline_misses,
            (unsigned long)s5_dma_errors
        );

        if (len > 0)
        {
            uint16_t tx_len = (uint16_t)(
                (len < (int)sizeof(msg))
                ? len
                : (int)sizeof(msg) - 1
            );

            HAL_UART_Transmit(
                &huart3,
                (uint8_t *)msg,
                tx_len,
                50
            );
        }

        s7_input_peak = 0;
        s7_output_peak = 0;
        s7_max_cycles = 0;
    }

    /* Continue looping immediately. */
    (void)processed_this_iteration;

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  /* Keep 72 MHz Debug unchanged; use 100 MHz only in Performance100. */
#if defined(SLIPSTREAM_100MHZ)
  RCC_OscInitStruct.PLL.PLLN = 100;
#else
  RCC_OscInitStruct.PLL.PLLN = 72;
#endif
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  /* 100 MHz PLL: VCO=200 MHz, PLLQ=4 gives 50 MHz (NOT USB-valid).
     Target MCU OTG-FS is intentionally not initialized in this build. */
#if defined(SLIPSTREAM_100MHZ)
  RCC_OscInitStruct.PLL.PLLQ = 4;
#else
  RCC_OscInitStruct.PLL.PLLQ = 3;
#endif
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  /* RM0402: at VDD=3.3 V, 100 MHz requires 3 Flash wait states;
     72 MHz remains at the existing 2 wait states. */
#if defined(SLIPSTREAM_100MHZ)
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
#else
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
#endif
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/*
 * First half of DMA buffer completed.
 */
void HAL_I2SEx_TxRxHalfCpltCallback(
    I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance != SPI2)
        return;

    s5_half_count++;
    s7_ready_count[0]++;
    s7_irq_blocks++;
}

/*
 * Second half of DMA buffer completed.
 */
void HAL_I2SEx_TxRxCpltCallback(
    I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance != SPI2)
        return;

    s5_full_count++;
    s7_ready_count[1]++;
    s7_irq_blocks++;
}

/*
 * Audio hardware error reporting.
 */
void HAL_I2S_ErrorCallback(
    I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance != SPI2)
        return;

    s5_dma_errors++;
    s5_last_error = hi2s->ErrorCode;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
