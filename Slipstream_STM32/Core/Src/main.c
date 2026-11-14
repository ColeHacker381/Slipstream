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
 * SLIPSTREAM S5
 *
 * 48 kHz stereo, signed 16-bit PCM
 * 10 ms per DMA half
 * 20 ms total circular DMA buffer
 *
 * Microphone: left ADC channel
 * Playback: both DAC channels
 */

#define WM8960_ADDR         (0x1A << 1)

#define S5_FRAMES_PER_HALF  480U
#define S5_CHANNELS         2U
#define S5_HALF_SAMPLES     (S5_FRAMES_PER_HALF * S5_CHANNELS)
#define S5_DMA_SAMPLES      (S5_HALF_SAMPLES * 2U)

#define S5_LOOPBACK_DIVISOR 1

static uint16_t s5_tx[S5_DMA_SAMPLES] = {0};
static uint16_t s5_rx[S5_DMA_SAMPLES] = {0};

/* Updated from DMA interrupt callbacks. */
static volatile uint32_t s5_half_count = 0;
static volatile uint32_t s5_full_count = 0;
static volatile uint32_t s5_peak = 0;
static volatile uint32_t s5_dma_errors = 0;
static volatile uint32_t s5_last_error = 0;

/*
 * WM8960 register writes.
 *
 * Registers contain 9-bit values.
 * The codec uses a 7-bit I2C address.
 */
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

/*
 * Preserves the headphone setup from S2/S3.
 */
static HAL_StatusTypeDef wm8960_headphone_init(void)
{
    HAL_StatusTypeDef status;

    #define WM_WRITE(reg, value)                   \
        do {                                       \
            status = wm8960_write(reg, value);     \
            if (status != HAL_OK) return status;   \
        } while (0)

    /* Reset codec */
    WM_WRITE(0x0F, 0x000);
    HAL_Delay(10);

    /* VREF + VMID */
    WM_WRITE(0x19, 0x0C0);
    HAL_Delay(50);

    /* Existing 48 kHz codec PLL settings */
    WM_WRITE(0x34, 0x038);
    WM_WRITE(0x35, 0x031);
    WM_WRITE(0x36, 0x026);
    WM_WRITE(0x37, 0x0E8);

    /* Clocking */
    WM_WRITE(0x04, 0x005);
    WM_WRITE(0x08, 0x1C4);

    /* I2S, 16-bit, codec slave */
    WM_WRITE(0x07, 0x002);

    /* DAC output mixers */
    WM_WRITE(0x22, 0x150);
    WM_WRITE(0x25, 0x150);
    WM_WRITE(0x2F, 0x00C);

    /* DACs, headphone drivers, PLL */
    WM_WRITE(0x1A, 0x1E3);

    /* DAC unmute */
    WM_WRITE(0x05, 0x000);

    /* Conservative headphone volume */
    WM_WRITE(0x02, 0x165);
    WM_WRITE(0x03, 0x165);

    #undef WM_WRITE

    return HAL_OK;
}

/*
 * Adds microphone functionality without resetting
 * the headphone configuration.
 */
static HAL_StatusTypeDef wm8960_enable_microphone(void)
{
    HAL_StatusTypeDef status;

    #define MIC_WRITE(reg, value)                  \
        do {                                       \
            status = wm8960_write(reg, value);     \
            if (status != HAL_OK) return status;   \
        } while (0)

    /* Left analog input + ADC + mic bias */
    MIC_WRITE(0x19, 0x0EA);
    HAL_Delay(50);

    /* Left microphone PGA + output mixers */
    MIC_WRITE(0x2F, 0x02C);

    /* Microphone bias */
    MIC_WRITE(0x30, 0x001);

    /* Microphone PGA */
    MIC_WRITE(0x00, 0x12F);

    /* Microphone signal routing */
    MIC_WRITE(0x20, 0x148);

    /* ADC digital volume */
    MIC_WRITE(0x15, 0x1C3);

    #undef MIC_WRITE

    return HAL_OK;
}

/*
 * Process one completed 10 ms DMA half.
 *
 * The RX DMA has just finished writing this region.
 * The TX DMA has moved beyond this region.
 *
 * We prepare it for the next DMA cycle.
 *
 * No UART, delays, or blocking HAL calls here.
 */
static void s5_process_half(uint32_t offset)
{
    uint32_t peak = 0;

    for (uint32_t i = 0; i < S5_FRAMES_PER_HALF; i++)
    {
        uint32_t index = offset + (i * 2U);

        /* Capture the left microphone sample. */
        int32_t mic = (int16_t)s5_rx[index];

        /* Initial low-gain loopback. */
        int16_t output = (int16_t)(
            mic / S5_LOOPBACK_DIVISOR
        );

        /* Copy microphone to both headphones. */
        s5_tx[index] = (uint16_t)output;
        s5_tx[index + 1U] = (uint16_t)output;

        /* Capture microphone peak for diagnostics. */
        uint32_t magnitude = (mic < 0)
            ? (uint32_t)(-mic)
            : (uint32_t)mic;

        if (magnitude > peak)
        {
            peak = magnitude;
        }
    }

    if (peak > s5_peak)
    {
        s5_peak = peak;
    }
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
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */

  const char *boot_msg =
      "\r\n[Slipstream] Starting S5 DMA audio\r\n";

  HAL_UART_Transmit(
      &huart3,
      (uint8_t *)boot_msg,
      strlen(boot_msg),
      HAL_MAX_DELAY
  );

  /* Initialize headphone audio path. */
  HAL_StatusTypeDef status = wm8960_headphone_init();

  if (status != HAL_OK)
  {
      const char *msg =
          "[S5] ERROR: WM8960 headphone init\r\n";

      HAL_UART_Transmit(
          &huart3,
          (uint8_t *)msg,
          strlen(msg),
          HAL_MAX_DELAY
      );

      Error_Handler();
  }

  /* Enable microphone audio path. */
  status = wm8960_enable_microphone();

  if (status != HAL_OK)
  {
      const char *msg =
          "[S5] ERROR: WM8960 microphone init\r\n";

      HAL_UART_Transmit(
          &huart3,
          (uint8_t *)msg,
          strlen(msg),
          HAL_MAX_DELAY
      );

      Error_Handler();
  }

  /* Verify that CubeMX linked both DMA handles. */
  if (hi2s2.hdmatx == NULL || hi2s2.hdmarx == NULL)
  {
      const char *msg =
          "[S5] ERROR: I2S DMA handles missing\r\n";

      HAL_UART_Transmit(
          &huart3,
          (uint8_t *)msg,
          strlen(msg),
          HAL_MAX_DELAY
      );

      Error_Handler();
  }

  /* Start continuous full-duplex audio transfer. */
  status = HAL_I2SEx_TransmitReceive_DMA(
      &hi2s2,
      s5_tx,
      s5_rx,
      S5_DMA_SAMPLES
  );

  if (status != HAL_OK)
  {
      char msg[120];

      int len = snprintf(
          msg,
          sizeof(msg),
          "[S5] DMA start failed: HAL=%d I2S=0x%08lX\r\n",
          (int)status,
          (unsigned long)hi2s2.ErrorCode
      );

      if (len > 0)
      {
          HAL_UART_Transmit(
              &huart3,
              (uint8_t *)msg,
              (uint16_t)len,
              HAL_MAX_DELAY
          );
      }

      Error_Handler();
  }

  const char *ready_msg =
      "[S5] DMA started; circular loopback active\r\n";

  HAL_UART_Transmit(
      &huart3,
      (uint8_t *)ready_msg,
      strlen(ready_msg),
      HAL_MAX_DELAY
  );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    static uint32_t last_report = 0;
    static uint32_t previous_blocks = 0;

    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_report) >= 1000U)
    {
        last_report = now;

        uint32_t half_count = s5_half_count;
        uint32_t full_count = s5_full_count;
        uint32_t peak = s5_peak;
        uint32_t errors = s5_dma_errors;
        uint32_t error_code = s5_last_error;

        uint32_t total_blocks = half_count + full_count;
        uint32_t blocks_per_second =
            total_blocks - previous_blocks;

        previous_blocks = total_blocks;
        s5_peak = 0;

        char msg[180];

        int len = snprintf(
            msg,
            sizeof(msg),
            "[S5] Blocks/s=%lu Peak=%lu "
            "Half=%lu Full=%lu Errors=%lu "
            "LastError=0x%08lX\r\n",
            (unsigned long)blocks_per_second,
            (unsigned long)peak,
            (unsigned long)half_count,
            (unsigned long)full_count,
            (unsigned long)errors,
            (unsigned long)error_code
        );

        if (len > 0)
        {
            HAL_UART_Transmit(
                &huart3,
                (uint8_t *)msg,
                (uint16_t)((len < (int)sizeof(msg))
                    ? len : (int)sizeof(msg) - 1),
                100
            );
        }

        if (blocks_per_second == 0U)
        {
            const char *warning =
                "[S5] WARNING: No DMA progress\r\n";

            HAL_UART_Transmit(
                &huart3,
                (uint8_t *)warning,
                strlen(warning),
                100
            );
        }
    }

    HAL_Delay(10);

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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
 * The RX DMA completed the first 10 ms.
 * Process the first half while DMA advances
 * through the second half.
 */
void HAL_I2SEx_TxRxHalfCpltCallback(
    I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance != SPI2)
    {
        return;
    }

    s5_process_half(0U);

    s5_half_count++;
}

/*
 * The RX DMA completed the second 10 ms.
 * Process the second half while DMA
 * wraps around to the first half.
 */
void HAL_I2SEx_TxRxCpltCallback(
    I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance != SPI2)
    {
        return;
    }

    s5_process_half(S5_HALF_SAMPLES);

    s5_full_count++;
}

/*
 * Record hardware errors.
 * Never print to UART from an ISR.
 */
void HAL_I2S_ErrorCallback(
    I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance != SPI2)
    {
        return;
    }

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
