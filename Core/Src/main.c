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
#include <math.h>
#include <string.h>
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
SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockA2;

/* USER CODE BEGIN PV */
int32_t rx_audio_buf[AUDIO_BUFFER_SIZE];
int32_t tx_audio_buf[AUDIO_BUFFER_SIZE];

volatile MicSelection_t current_mic_sel = MIC_SEL_MIC1;
volatile float live_mic_gain = 2.0f;          /* Default +6 dB linear gain */
volatile uint8_t switch_mute_frames = 0;       /* Anti-pop transient mute counter */

/* Tone Generation DSP Channel States */
ChannelState ch_stim;
ChannelState ch_mask;

/* Standard Audiometric Test Frequencies in Hz */
const float AUDIOMETER_FREQUENCIES_HZ[11] = {
  125.0f, 250.0f, 500.0f, 750.0f, 1000.0f, 1500.0f,
  2000.0f, 3000.0f, 4000.0f, 6000.0f, 8000.0f
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
void MX_SAI1_Init(void);
void MX_SAI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* Configure the peripherals common clocks (SAI1 & SAI2 audio clock) */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SAI1_Init();
  MX_SAI2_Init();
  /* USER CODE BEGIN 2 */

  /* 1. Turn on ALL transducers & outputs (AC Left/Right, Insert Earphone, Bone Conductor, Free Field) */
  HAL_GPIO_WritePin(GPIOE, AC_Left_EN_Pin | AC_Right_EN_Pin | INSERT_EP_EN_Pin | BC_EN_Pin | BC_L_R_EN_Pin | FF_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(INTERNAL_SPK_EN_GPIO_Port, INTERNAL_SPK_EN_Pin, GPIO_PIN_SET);

  /* 2. Turn on Stimulus presentation switches (PA6, PB16) */
  HAL_GPIO_WritePin(STIMULUS1_GPIO_Port, STIMULUS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(STIMULUS2_GPIO_Port, STIMULUS2_Pin, GPIO_PIN_SET);

  /* 3. Enable OPA headphone amplifier */
  HAL_GPIO_WritePin(OPA_EN_GPIO_Port, OPA_EN_Pin, GPIO_PIN_SET);

  /* 4. Select PCM DAC mode (PA9 = LOW) */
  HAL_GPIO_WritePin(PCM_MIC_Control_GPIO_Port, PCM_MIC_Control_Pin, GPIO_PIN_RESET);

  /* 5. Un-mute PGA2311 (PB13 = HIGH) and disable Zero-Cross delay (PB14 = LOW for instant update) */
  HAL_GPIO_WritePin(PGA_MUTE_1_GPIO_Port, PGA_MUTE_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ZCEN_GPIO_Port, ZCEN_Pin, GPIO_PIN_RESET);

  /* 6. Set volume of PGA2311 (code 200 = ~ +4 dB clean audible volume) */
  PGA2311_SetVolume(200, 200);

  /* USER CODE END 2 */

  /* Configure PCM5102 DAC Audio Pins as High-Speed Outputs:
   * PD11 = LRCK (Frame Sync)
   * PD12 = DIN  (Serial Audio Data)
   * PE5  = BCK  (Bit Clock)
   * PE2  = MCLK (Master Clock)
   */
  GPIO_InitTypeDef GPIO_DAC_InitStruct = {0};
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_DAC_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_5;
  GPIO_DAC_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_DAC_InitStruct.Pull = GPIO_NOPULL;
  GPIO_DAC_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_DAC_InitStruct);

  GPIO_DAC_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_DAC_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_DAC_InitStruct.Pull = GPIO_NOPULL;
  GPIO_DAC_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_DAC_InitStruct);

  /* Set initial pin states: LRCK High (idle), BCK Low */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);

  /* Infinite loop - Direct High-Precision 48 kHz / 1 kHz Sine I2S Engine */
  /* USER CODE BEGIN WHILE */
  Audio_Channel_Init(&ch_stim, TONE_MODE_SINE, 1000.0f, 123456789);

  while (1)
  {
    /* Generate 1000 Hz Sine Sample (16-bit MSB-aligned in 32-bit slot) */
    int16_t sample = (int16_t)(Audio_Process_Channel(&ch_stim) * 0.8f);
    uint32_t data_word = (uint32_t)(((uint32_t)sample) << 16);

    /* =========================================================================
     * LEFT CHANNEL: LRCK (PD11) = LOW
     * Standard I2S: 32 bits, MSB first, 1-bit delay
     * ========================================================================= */
    /* Bit 0: LRCK transitions to LOW 1 bit before MSB */
    GPIOD->BSRR = (uint32_t)GPIO_PIN_11 << 16U; /* LRCK LOW */
    if (data_word & (1UL << 31)) GPIOD->BSRR = GPIO_PIN_12; else GPIOD->BSRR = (uint32_t)GPIO_PIN_12 << 16U;
    for (volatile int d = 0; d < 8; d++) __NOP();
    GPIOE->BSRR = GPIO_PIN_5; /* BCK HIGH */
    for (volatile int d = 0; d < 8; d++) __NOP();
    GPIOE->BSRR = (uint32_t)GPIO_PIN_5 << 16U; /* BCK LOW */

    /* Bits 1 to 31 */
    for (int8_t b = 30; b >= 0; b--)
    {
      if (data_word & (1UL << b)) GPIOD->BSRR = GPIO_PIN_12; else GPIOD->BSRR = (uint32_t)GPIO_PIN_12 << 16U;
      for (volatile int d = 0; d < 8; d++) __NOP();
      GPIOE->BSRR = GPIO_PIN_5; /* BCK HIGH */
      for (volatile int d = 0; d < 8; d++) __NOP();
      GPIOE->BSRR = (uint32_t)GPIO_PIN_5 << 16U; /* BCK LOW */
    }

    /* =========================================================================
     * RIGHT CHANNEL: LRCK (PD11) = HIGH
     * Standard I2S: 32 bits, MSB first, 1-bit delay
     * ========================================================================= */
    /* Bit 0: LRCK transitions to HIGH 1 bit before MSB */
    GPIOD->BSRR = GPIO_PIN_11; /* LRCK HIGH */
    if (data_word & (1UL << 31)) GPIOD->BSRR = GPIO_PIN_12; else GPIOD->BSRR = (uint32_t)GPIO_PIN_12 << 16U;
    for (volatile int d = 0; d < 8; d++) __NOP();
    GPIOE->BSRR = GPIO_PIN_5; /* BCK HIGH */
    for (volatile int d = 0; d < 8; d++) __NOP();
    GPIOE->BSRR = (uint32_t)GPIO_PIN_5 << 16U; /* BCK LOW */

    /* Bits 1 to 31 */
    for (int8_t b = 30; b >= 0; b--)
    {
      if (data_word & (1UL << b)) GPIOD->BSRR = GPIO_PIN_12; else GPIOD->BSRR = (uint32_t)GPIO_PIN_12 << 16U;
      for (volatile int d = 0; d < 8; d++) __NOP();
      GPIOE->BSRR = GPIO_PIN_5; /* BCK HIGH */
      for (volatile int d = 0; d < 8; d++) __NOP();
      GPIOE->BSRR = (uint32_t)GPIO_PIN_5 << 16U; /* BCK LOW */
    }
  }
  /* USER CODE END 3 */
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

  /** First attempt: Try External High Speed Oscillator (HSE 8MHz) with PLL
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* HSE crystal failed to start or timed out -> Fallback seamlessly to Internal 16MHz RC (HSI) */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    RCC_OscInitStruct.PLL.PLLR = 2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      /* If PLL fails, run directly on raw HSI without PLL */
      RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
      HAL_RCC_OscConfig(&RCC_OscInitStruct);
    }
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  
  if ((RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLRDY)
  {
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
  }
  else
  {
    /* Fallback directly to HSI 16MHz */
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
  }
}

/**
  * @brief Peripherals Common Clock Configuration (SAI1 & SAI2 Audio Clock)
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1 | RCC_PERIPHCLK_SAI2;

  /* If running on HSI (16MHz), PLLSAIM = 16 (16MHz / 16 = 1MHz VCO input)
   * If running on HSE (8MHz),  PLLSAIM = 8  (8MHz / 8 = 1MHz VCO input) */
  if ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI)
  {
    PeriphClkInitStruct.PLLSAI.PLLSAIM = 16;
  }
  else
  {
    PeriphClkInitStruct.PLLSAI.PLLSAIM = 8;
  }

  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;

  /* Configure SAI1 & SAI2 peripheral clock */
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
}

/**
  * @brief SAI1 Initialization Function (Configured for PCM5102A Stereo DAC Transmitter - PE2/PE5/PE4/PE6)
  * @param None
  * @retval None
  */
void MX_SAI1_Init(void)
{
  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */

  /* Configure SAI1 Block A as Master Transmitter for PCM5102A */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_32;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;

  /* Standard Philips I2S Frame: Total Frame Length = 64 (32 bits per channel)
   * Active Frame Length = 32 (Half frame for Left channel)
   * Channel Identification: FS Low for Left, High for Right
   * 1-bit delay (Before first data bit)
   */
  hsai_BlockA1.FrameInit.FrameLength = 64;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 32;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;

  /* Slot Configuration: 2 Slots (Left = Slot 0, Right = Slot 1), 32-bit slot width */
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
  hsai_BlockA1.SlotInit.SlotNumber = 2;
  hsai_BlockA1.SlotInit.SlotActive = SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1;

  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */
}

/**
  * @brief SAI2 Initialization Function (Configured for PCM5102A Stereo DAC Transmitter - PD11/PD12)
  * @param None
  * @retval None
  */
void MX_SAI2_Init(void)
{
  /* Configure SAI2 Block A as Master Transmitter for PCM5102A on PD11 (SD) / PD12 (FS) */
  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA2.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA2.Init.DataSize = SAI_DATASIZE_32;
  hsai_BlockA2.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;

  hsai_BlockA2.FrameInit.FrameLength = 64;
  hsai_BlockA2.FrameInit.ActiveFrameLength = 32;
  hsai_BlockA2.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockA2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA2.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;

  hsai_BlockA2.SlotInit.FirstBitOffset = 0;
  hsai_BlockA2.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
  hsai_BlockA2.SlotInit.SlotNumber = 2;
  hsai_BlockA2.SlotInit.SlotActive = SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1;

  if (HAL_SAI_Init(&hsai_BlockA2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Configure GPIO pin Output Level: Default Low */
  HAL_GPIO_WritePin(GPIOE, BC_EN_Pin|WN_EN_Pin|FF_EN_Pin|BC_L_R_EN_Pin
                          |INSERT_EP_EN_Pin|AC_Left_EN_Pin|AC_Right_EN_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOA, STIMULUS1_Pin|PCM_MIC_Control_Pin|INTERNAL_SPK_EN_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, RTC_EN_Pin|OPA_EN_Pin|RTC_CLK_Pin|RTC_IO_Pin
                          |PGA_SCLK_Pin|PGA_MUTE_1_Pin|ZCEN_Pin|STIMULUS2_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOD, TALKOVER_EN_Pin|MIC_AUX_EN_Pin|MIC_EN_Pin|PCM1808_DATA_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, PGA_SDI_Pin, GPIO_PIN_RESET);

  /* Configure GPIO pin Output Level: Chip Selects Default High (Inactive) */
  HAL_GPIO_WritePin(GPIOC, FRAM_CS_Pin|SD_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, PGA_CS1_Pin, GPIO_PIN_SET);

  /* Configure Storage Chip Selects: FRAM_CS (PC13), SD_CS (PC12) */
  GPIO_InitStruct.Pin = FRAM_CS_Pin|SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure Audio Route Enables: PE7, PE8, PE9, PE10, PE12, PE13, PE14 */
  GPIO_InitStruct.Pin = BC_EN_Pin|WN_EN_Pin|FF_EN_Pin|BC_L_R_EN_Pin
                          |INSERT_EP_EN_Pin|AC_Left_EN_Pin|AC_Right_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* Configure Audio Route Enables & Controls on Port D: PD4, PD5, PD6, PD10 */
  GPIO_InitStruct.Pin = TALKOVER_EN_Pin|MIC_AUX_EN_Pin|MIC_EN_Pin|PCM1808_DATA_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Configure GPIO Outputs on Port A: PA6 (STIMULUS1), PA9 (PCM/MIC), PA15 (INTERNAL_SPK_EN) */
  GPIO_InitStruct.Pin = STIMULUS1_Pin|PCM_MIC_Control_Pin|INTERNAL_SPK_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure GPIO Outputs on Port B: RTC, OPA, PGA controls, Stimulus2 */
  GPIO_InitStruct.Pin = RTC_EN_Pin|OPA_EN_Pin|RTC_CLK_Pin|RTC_IO_Pin
                          |PGA_SCLK_Pin|PGA_CS1_Pin|PGA_MUTE_1_Pin|ZCEN_Pin|STIMULUS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure PGA_SDI (PC1) Output */
  GPIO_InitStruct.Pin = PGA_SDI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure PGA_SDO (PC2) Input */
  GPIO_InitStruct.Pin = PGA_SDO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure POWER_DETECTION (PA1) Input */
  GPIO_InitStruct.Pin = POWER_DETECTION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure Patient Response Switch (PB3) as Interrupt/Input */
  GPIO_InitStruct.Pin = PAT_RESPONSE_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PAT_RESPONSE_SW_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Selects active microphone source by controlling hardware switch (PD10)
  *         and configuring the software demuxer.
  * @param  mic: Target microphone selection (MIC_SEL_MIC1..MIC4 or STEREO)
  */
void Audio_Select_Microphone(MicSelection_t mic)
{
  current_mic_sel = mic;

  if (mic == MIC_SEL_MIC1 || mic == MIC_SEL_MIC2 || mic == MIC_SEL_IC1_STEREO)
  {
    /* Select IC1 (U8: MIC1=Left, MIC2=Right) -> PCM1808_DATA_EN HIGH */
    HAL_GPIO_WritePin(PCM1808_DATA_EN_GPIO_Port, PCM1808_DATA_EN_Pin, GPIO_PIN_SET);
  }
  else
  {
    /* Select IC2 (U12: MIC3=Left, MIC4=Right) -> PCM1808_DATA_EN LOW */
    HAL_GPIO_WritePin(PCM1808_DATA_EN_GPIO_Port, PCM1808_DATA_EN_Pin, GPIO_PIN_RESET);
  }

  /* Mute for 2 DMA buffer cycles to suppress hardware switching transients */
  switch_mute_frames = 2;
}

/**
  * @brief  Sets linear digital gain.
  * @param  gain: Linear multiplier (e.g. 1.0 = 0dB, 2.0 = +6dB, 4.0 = +12dB)
  */
void Audio_Set_Gain_Linear(float gain)
{
  if (gain < 0.0f) gain = 0.0f;
  live_mic_gain = gain;
}

/**
  * @brief  Sets digital amplification in decibels (dB).
  * @param  gain_db: Gain in dB (e.g. 0.0dB to +40.0dB)
  */
void Audio_Set_Gain_dB(float gain_db)
{
  live_mic_gain = powf(10.0f, gain_db / 20.0f);
}

/**
  * @brief  Gets current digital amplification in decibels (dB).
  * @retval Current gain in dB
  */
float Audio_Get_Gain_dB(void)
{
  if (live_mic_gain <= 0.00001f) return -100.0f;
  return 20.0f * log10f(live_mic_gain);
}

/**
  * @brief  Demultiplexes selected Mic, applies gain, and routes to DAC buffer.
  * @param  pSrc: Interleaved raw ADC buffer from PCM1808 (32-bit slots)
  * @param  pDst: Interleaved DAC buffer for PCM5102 (32-bit slots)
  * @param  length: Total number of 32-bit words (Left + Right slots)
  * @param  gain: Linear gain factor
  */
void Process_Mic_To_DAC(int32_t *pSrc, int32_t *pDst, uint16_t length, float gain)
{
  /* If switching ICs, output silence to prevent switching clicks */
  if (switch_mute_frames > 0)
  {
    memset(pDst, 0, length * sizeof(int32_t));
    switch_mute_frames--;
    return;
  }

  for (uint16_t i = 0; i < length; i += 2)
  {
    /* Extract 24-bit samples (PCM1808 transmits MSB-aligned in upper 24 bits) */
    int32_t raw_left  = pSrc[i]     >> 8; /* Left channel sample (MIC1 or MIC3) */
    int32_t raw_right = pSrc[i + 1] >> 8; /* Right channel sample (MIC2 or MIC4) */

    float out_left_f  = 0.0f;
    float out_right_f = 0.0f;

    switch (current_mic_sel)
    {
      case MIC_SEL_MIC1: /* IC1 Left (MIC1) -> Mono on both DAC channels */
      case MIC_SEL_MIC3: /* IC2 Left (MIC3) -> Mono on both DAC channels */
        out_left_f  = (float)raw_left * gain;
        out_right_f = out_left_f;
        break;

      case MIC_SEL_MIC2: /* IC1 Right (MIC2) -> Mono on both DAC channels */
      case MIC_SEL_MIC4: /* IC2 Right (MIC4) -> Mono on both DAC channels */
        out_left_f  = (float)raw_right * gain;
        out_right_f = out_left_f;
        break;

      case MIC_SEL_IC1_STEREO: /* IC1 Stereo: MIC1 -> L, MIC2 -> R */
      case MIC_SEL_IC2_STEREO: /* IC2 Stereo: MIC3 -> L, MIC4 -> R */
        out_left_f  = (float)raw_left  * gain;
        out_right_f = (float)raw_right * gain;
        break;
    }

    /* Soft Saturation / Anti-clipping Limiter (24-bit range: -8388608 to +8388607) */
    if (out_left_f > 8388607.0f)        out_left_f = 8388607.0f;
    else if (out_left_f < -8388608.0f)  out_left_f = -8388608.0f;

    if (out_right_f > 8388607.0f)       out_right_f = 8388607.0f;
    else if (out_right_f < -8388608.0f) out_right_f = -8388608.0f;

    /* Pack back into 32-bit slot for PCM5102 DAC */
    pDst[i]     = ((int32_t)out_left_f)  << 8;
    pDst[i + 1] = ((int32_t)out_right_f) << 8;
  }
}

/**
  * @brief  SAI Rx Half-Transfer Complete callback (Ping buffer ready).
  * @param  hsai: SAI handle pointer
  */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  Process_Mic_To_DAC(&rx_audio_buf[0], &tx_audio_buf[0], HALF_BUFFER_SIZE, live_mic_gain);
}

/**
  * @brief  SAI Rx Transfer Complete callback (Pong buffer ready).
  * @param  hsai: SAI handle pointer
  */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  Process_Mic_To_DAC(&rx_audio_buf[HALF_BUFFER_SIZE], &tx_audio_buf[HALF_BUFFER_SIZE], HALF_BUFFER_SIZE, live_mic_gain);
}

/* ==============================================================================
 * Tone & Noise Synthesis Engine (Pure Tone, Warble Tone, NBN, White Noise)
 * Audio Transmission via SAI1 Block A (PCM5102A Stereo DAC @ 48 kHz)
 * ============================================================================== */

/* 256-point Sine Lookup Table (16-bit signed values: -32768 to 32767) */
static const int16_t SINE_LUT[256] = {
  0, 804, 1607, 2410, 3211, 4011, 4807, 5601, 6392, 7179, 7961, 8739, 9511, 10278, 11038, 11792,
  12539, 13278, 14009, 14732, 15446, 16150, 16845, 17530, 18204, 18867, 19519, 20159, 20787, 21402, 22005, 22594,
  23169, 23731, 24278, 24811, 25329, 25831, 26318, 26789, 27244, 27683, 28105, 28510, 28897, 29268, 29621, 29955,
  30272, 30571, 30851, 31113, 31356, 31580, 31785, 31970, 32137, 32284, 32412, 32520, 32609, 32678, 32727, 32757,
  32767, 32757, 32727, 32678, 32609, 32520, 32412, 32284, 32137, 31970, 31785, 31580, 31356, 31113, 30851, 30571,
  30272, 29955, 29621, 29268, 28897, 28510, 28105, 27683, 27244, 26789, 26318, 25831, 25329, 24811, 24278, 23731,
  23169, 22594, 22005, 21402, 20787, 20159, 19519, 18867, 18204, 17530, 16845, 16150, 15446, 14732, 14009, 13278,
  12539, 11792, 11038, 10278, 9511, 8739, 7961, 7179, 6392, 5601, 4807, 4011, 3211, 2410, 1607, 804,
  0, -804, -1607, -2410, -3211, -4011, -4807, -5601, -6392, -7179, -7961, -8739, -9511, -10278, -11038, -11792,
  -12539, -13278, -14009, -14732, -15446, -16150, -16845, -17530, -18204, -18867, -19519, -20159, -20787, -21402, -22005, -22594,
  -23169, -23731, -24278, -24811, -25329, -25831, -26318, -26789, -27244, -27683, -28105, -28510, -28897, -29268, -29621, -29955,
  -30272, -30571, -30851, -31113, -31356, -31580, -31785, -31970, -32137, -32284, -32412, -32520, -32609, -32678, -32727, -32757,
  -32767, -32757, -32727, -32678, -32609, -32520, -32412, -32284, -32137, -31970, -31785, -31580, -31356, -31113, -30851, -30571,
  -30272, -29955, -29621, -29268, -28897, -28510, -28105, -27683, -27244, -26789, -26318, -25831, -25329, -24811, -24278, -23731,
  -23169, -22594, -22005, -21402, -20787, -20159, -19519, -18867, -18204, -17530, -16845, -16150, -15446, -14732, -14009, -13278,
  -12539, -11792, -11038, -10278, -9511, -8739, -7961, -7179, -6392, -5601, -4807, -4011, -3211, -2410, -1607, -804
};

/**
  * @brief  Fast 32-bit XORshift pseudo-random number generator for white noise.
  * @param  state: Pointer to PRNG state variable
  * @retval 32-bit pseudo-random value
  */
static inline uint32_t xorshift32(uint32_t *state)
{
  uint32_t x = *state;
  if (x == 0) x = 0x98765432;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  *state = x;
  return x;
}

/**
  * @brief  Initializes a synthesizer channel state for specified mode and frequency.
  * @param  ch: Pointer to ChannelState structure
  * @param  mode: TONE_MODE_SINE, TONE_MODE_WARBLE, TONE_MODE_WHITE_NOISE, TONE_MODE_NBN, TONE_MODE_OFF
  * @param  freq_hz: Frequency in Hertz (e.g. 125.0f to 8000.0f)
  * @param  seed: PRNG seed for noise modes
  */
void Audio_Channel_Init(ChannelState *ch, int mode, float freq_hz, uint32_t seed)
{
  if (ch == NULL) return;

  if (mode == TONE_MODE_OFF) {
    ch->mode = TONE_MODE_OFF;
    ch->amp = 0.0f;
    return;
  }

  /* Clamp frequency range */
  if (freq_hz > 12000.0f) freq_hz = 12000.0f;
  if (freq_hz < 0.0f) freq_hz = 0.0f;

  ch->mode = mode;
  ch->freq = freq_hz;
  ch->amp = 1.0f;

  ch->acc = 0;
  ch->inc = (uint32_t)(freq_hz * 4294967296.0f / AUDIO_SAMPLE_RATE_HZ);

  ch->cp = 0.0f;
  ch->mp = 0.0f;
  ch->mp_acc = 0;
  ch->warble_rate = 5.0f; /* 5 Hz audiometric standard modulation rate */
  ch->warble_dev = 5.0f;  /* 5% audiometric standard frequency deviation */

  ch->noise_seed = (seed == 0) ? 0x12345678 : seed;

  ch->bp_x1 = 0.0f; ch->bp_x2 = 0.0f;
  ch->bp_y1 = 0.0f; ch->bp_y2 = 0.0f;

  if (mode == TONE_MODE_NBN) {
    /* Constant Q for 0.333 (1/3) octave bandwidth per IEC 60645-1 standard */
    float q = 4.31847f;
    float w = 2.0f * 3.14159265f * freq_hz / AUDIO_SAMPLE_RATE_HZ;
    float alpha = sinf(w) / (2.0f * q);
    float a0 = 1.0f + alpha;

    ch->b0 = alpha / a0;
    ch->b2 = -alpha / a0;
    ch->a1 = (-2.0f * cosf(w)) / a0;
    ch->a2 = (1.0f - alpha) / a0;
  }
}

/**
  * @brief  Computes the next 16-bit audio sample for a given channel state.
  * @param  ch: Pointer to ChannelState structure
  * @retval 16-bit signed audio sample (-32768 to 32767)
  */
int16_t Audio_Process_Channel(ChannelState *ch)
{
  if (ch == NULL || ch->mode == TONE_MODE_OFF) return 0;

  if (ch->mode == TONE_MODE_SINE)
  {
    /* Pure Tone: Direct Phase Accumulation using 256-point Sine LUT */
    ch->acc += ch->inc;
    uint8_t idx = (uint8_t)(ch->acc >> 24);
    return (int16_t)(SINE_LUT[idx] * ch->amp);
  }
  else if (ch->mode == TONE_MODE_WARBLE)
  {
    /* Warble Tone: Frequency Modulated Sine Wave
     * 5 Hz modulation rate at 48 kHz: Modulator step = 5 * 2^32 / 48000 = 447392 */
    ch->mp_acc += 447392;

    uint8_t mod_idx = (uint8_t)(ch->mp_acc >> 24);
    int32_t mod_val = (int32_t)SINE_LUT[mod_idx]; /* -32768 to 32767 */

    /* 5% frequency deviation: inst_step = inc + mod_val * deviation_step / 32768 */
    int64_t dev_step = ((int64_t)ch->inc * 5) / 100;
    int32_t inst_step = ch->inc + (int32_t)((mod_val * dev_step) >> 15);

    ch->acc += inst_step;
    uint8_t car_idx = (uint8_t)(ch->acc >> 24);
    return (int16_t)(SINE_LUT[car_idx] * ch->amp);
  }
  else if (ch->mode == TONE_MODE_WHITE_NOISE)
  {
    /* Broadband White Noise: 32-bit xorshift PRNG scaled to 16-bit */
    uint32_t rnd = xorshift32(&ch->noise_seed);
    int16_t noise = (int16_t)(rnd >> 16);
    return (int16_t)(noise * ch->amp);
  }
  else if (ch->mode == TONE_MODE_NBN)
  {
    /* Narrow Band Noise: 2nd-order IIR Biquad Bandpass Filter fed with white noise */
    uint32_t rnd = xorshift32(&ch->noise_seed);
    float noise = (float)((int16_t)(rnd >> 16)) / 32768.0f;

    float v = ch->b0 * noise + ch->b2 * ch->bp_x2 - ch->a1 * ch->bp_y1 - ch->a2 * ch->bp_y2;
    ch->bp_x2 = ch->bp_x1;
    ch->bp_x1 = noise;
    ch->bp_y2 = ch->bp_y1;
    ch->bp_y1 = v;

    /* Soft Limiter */
    if (v > 1.0f) v = 1.0f;
    else if (v < -1.0f) v = -1.0f;

    return (int16_t)(v * 32767.0f * ch->amp);
  }

  return 0;
}

/**
  * @brief  Generates an interleaved Left and Right sample pair routed according to ear selection.
  * @param  ch_stim: Pointer to stimulus channel (Tone or Warble)
  * @param  ch_mask: Pointer to masking channel (NBN, White Noise, or OFF)
  * @param  stim_env: Stimulus envelope volume multiplier (0.0f to 1.0f)
  * @param  mask_env: Masking envelope volume multiplier (0.0f to 1.0f)
  * @param  active_ear: AUDIO_EAR_LEFT, AUDIO_EAR_RIGHT, or AUDIO_EAR_BOTH
  * @param  out_left: Output pointer for Left channel sample
  * @param  out_right: Output pointer for Right channel sample
  */
void Audio_Generate_Sample_Pair_SAI(ChannelState *ch_stim, ChannelState *ch_mask,
                                    float stim_env, float mask_env, uint8_t active_ear,
                                    int16_t *out_left, int16_t *out_right)
{
  int16_t sample_stim = (ch_stim != NULL) ? Audio_Process_Channel(ch_stim) : 0;
  int16_t sample_mask = (ch_mask != NULL) ? Audio_Process_Channel(ch_mask) : 0;

  if (active_ear == AUDIO_EAR_LEFT)
  {
    /* Left Ear = Stimulus, Right Ear = Contralateral Masking */
    *out_left  = (int16_t)((float)sample_stim * stim_env);
    *out_right = (int16_t)((float)sample_mask * mask_env);
  }
  else if (active_ear == AUDIO_EAR_RIGHT)
  {
    /* Right Ear = Stimulus, Left Ear = Contralateral Masking */
    *out_left  = (int16_t)((float)sample_mask * mask_env);
    *out_right = (int16_t)((float)sample_stim * stim_env);
  }
  else /* AUDIO_EAR_BOTH / Binaural */
  {
    *out_left  = (int16_t)((float)sample_stim * stim_env);
    *out_right = (int16_t)((float)sample_stim * stim_env);
  }
}

/**
  * @brief  Plays a stimulus tone and optional masking noise over SAI Block A to the PCM5102 DAC.
  *         Includes 10 ms smooth attack/release anti-pop envelope ramps.
  * @param  freq_hz: Frequency in Hertz (125 Hz - 8000 Hz)
  * @param  duration_ms: Duration of playback in milliseconds
  * @param  stim_mode: TONE_MODE_SINE or TONE_MODE_WARBLE
  * @param  mask_mode: TONE_MODE_OFF, TONE_MODE_NBN, or TONE_MODE_WHITE_NOISE
  * @param  vol_stim: Volume for stimulus (0.0f silent to 1.0f full scale)
  * @param  vol_mask: Volume for masking noise (0.0f silent to 1.0f full scale)
  * @param  ear: AUDIO_EAR_LEFT, AUDIO_EAR_RIGHT, or AUDIO_EAR_BOTH
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Audio_Play_Tone_SAI(float freq_hz, uint32_t duration_ms,
                                      int stim_mode, int mask_mode,
                                      float vol_stim, float vol_mask,
                                      uint8_t ear)
{
  if (duration_ms == 0) return HAL_OK;

  /* Initialize DSP synthesis channels */
  Audio_Channel_Init(&ch_stim, stim_mode, freq_hz, 123456789);
  Audio_Channel_Init(&ch_mask, mask_mode, freq_hz, 987654321);

  uint32_t sample_rate = (uint32_t)AUDIO_SAMPLE_RATE_HZ;
  uint32_t total_samples = (sample_rate * duration_ms) / 1000;
  uint32_t fade_samples = (sample_rate * 10) / 1000; /* 10 ms anti-pop fade ramp */
  if (fade_samples > total_samples / 2)
  {
    fade_samples = total_samples / 2;
  }

  /* Ensure SAI1 Block A transmitter is enabled (pre-fill FIFO to avoid startup underrun) */
  if ((hsai_BlockA1.Instance->CR1 & SAI_xCR1_SAIEN) == RESET)
  {
    for (int k = 0; k < 4; k++)
    {
      hsai_BlockA1.Instance->DR = 0;
    }
    __HAL_SAI_ENABLE(&hsai_BlockA1);
  }

  for (uint32_t i = 0; i < total_samples; i++)
  {
    /* Calculate dynamic envelope: 10 ms smooth fade-in and fade-out */
    float env_factor = 1.0f;
    if (i < fade_samples)
    {
      env_factor = (float)i / (float)fade_samples;
    }
    else if (i >= (total_samples - fade_samples))
    {
      env_factor = (float)(total_samples - i) / (float)fade_samples;
    }

    float current_stim_env = vol_stim * env_factor;
    float current_mask_env = vol_mask * env_factor;

    int16_t left_sample = 0;
    int16_t right_sample = 0;

    Audio_Generate_Sample_Pair_SAI(&ch_stim, &ch_mask,
                                   current_stim_env, current_mask_env, ear,
                                   &left_sample, &right_sample);

    /* 32-bit MSB alignment for standard Philips I2S 32-bit slot:
     * Audio sample is placed in upper 16 bits (bits 31..16), lower 16 bits are 0 */
    uint32_t left_word  = (uint32_t)(((int32_t)left_sample)  << 16);
    uint32_t right_word = (uint32_t)(((int32_t)right_sample) << 16);

    uint32_t timeout = 50000;
    while ((hsai_BlockA1.Instance->SR & SAI_xSR_FLVL) == SAI_FIFOSTATUS_FULL)
    {
      if (--timeout == 0) break;
    }
    hsai_BlockA1.Instance->DR = left_word;

    timeout = 50000;
    while ((hsai_BlockA1.Instance->SR & SAI_xSR_FLVL) == SAI_FIFOSTATUS_FULL)
    {
      if (--timeout == 0) break;
    }
    hsai_BlockA1.Instance->DR = right_word;
  }

  return HAL_OK;
}

/**
  * @brief  Plays a Pure Tone (Sine) via SAI.
  * @param  freq_hz: Frequency in Hertz (125 - 8000 Hz)
  * @param  duration_ms: Duration in milliseconds
  * @param  volume: Linear volume (0.0f to 1.0f)
  * @param  ear: AUDIO_EAR_LEFT, AUDIO_EAR_RIGHT, or AUDIO_EAR_BOTH
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Audio_Play_PureTone_SAI(float freq_hz, uint32_t duration_ms, float volume, uint8_t ear)
{
  return Audio_Play_Tone_SAI(freq_hz, duration_ms, TONE_MODE_SINE, TONE_MODE_OFF, volume, 0.0f, ear);
}

/**
  * @brief  Plays a Warble Tone (FM modulated, 5 Hz rate, 5% dev) via SAI.
  * @param  freq_hz: Carrier frequency in Hertz
  * @param  duration_ms: Duration in milliseconds
  * @param  volume: Linear volume (0.0f to 1.0f)
  * @param  ear: AUDIO_EAR_LEFT, AUDIO_EAR_RIGHT, or AUDIO_EAR_BOTH
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Audio_Play_Warble_SAI(float freq_hz, uint32_t duration_ms, float volume, uint8_t ear)
{
  return Audio_Play_Tone_SAI(freq_hz, duration_ms, TONE_MODE_WARBLE, TONE_MODE_OFF, volume, 0.0f, ear);
}

/**
  * @brief  Plays Narrow Band Noise (NBN, 1/3 octave bandpass filtered) via SAI.
  * @param  freq_hz: Center frequency in Hertz
  * @param  duration_ms: Duration in milliseconds
  * @param  volume: Linear volume (0.0f to 1.0f)
  * @param  ear: AUDIO_EAR_LEFT, AUDIO_EAR_RIGHT, or AUDIO_EAR_BOTH
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Audio_Play_NBN_SAI(float freq_hz, uint32_t duration_ms, float volume, uint8_t ear)
{
  return Audio_Play_Tone_SAI(freq_hz, duration_ms, TONE_MODE_NBN, TONE_MODE_OFF, volume, 0.0f, ear);
}

/**
  * @brief  Plays Broadband White Noise via SAI.
  * @param  duration_ms: Duration in milliseconds
  * @param  volume: Linear volume (0.0f to 1.0f)
  * @param  ear: AUDIO_EAR_LEFT, AUDIO_EAR_RIGHT, or AUDIO_EAR_BOTH
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Audio_Play_WhiteNoise_SAI(uint32_t duration_ms, float volume, uint8_t ear)
{
  return Audio_Play_Tone_SAI(1000.0f, duration_ms, TONE_MODE_WHITE_NOISE, TONE_MODE_OFF, volume, 0.0f, ear);
}

/**
  * @brief  Sets volume on PGA2311 (Stereo Audio Volume Control) via GPIO bit-banging.
  * @param  left_gain: Gain code for Left channel (0 = Mute, 1 = -95.5 dB, 192 = 0 dB, 255 = +31.5 dB Full)
  * @param  right_gain: Gain code for Right channel (0 = Mute, 1 = -95.5 dB, 192 = 0 dB, 255 = +31.5 dB Full)
  */
void PGA2311_SetVolume(uint8_t left_gain, uint8_t right_gain)
{
  /* PGA2311 shifts in 16 bits MSB first:
   * DB15..DB8: Right channel gain
   * DB7..DB0:   Left channel gain
   */
  uint16_t data = ((uint16_t)right_gain << 8) | (uint16_t)left_gain;

  /* Ensure ZCEN (PB14) is held LOW to disable zero-cross waiting for immediate gain change */
  HAL_GPIO_WritePin(ZCEN_GPIO_Port, ZCEN_Pin, GPIO_PIN_RESET);

  /* Ensure SCLK starts LOW */
  HAL_GPIO_WritePin(PGA_SCLK_GPIO_Port, PGA_SCLK_Pin, GPIO_PIN_RESET);

  /* Assert CS LOW to begin transfer */
  HAL_GPIO_WritePin(PGA_CS1_GPIO_Port, PGA_CS1_Pin, GPIO_PIN_RESET);
  for (volatile int d = 0; d < 50; d++) __NOP();

  for (int8_t i = 15; i >= 0; i--)
  {
    /* Set SDI data bit */
    if (data & (1U << i))
    {
      HAL_GPIO_WritePin(PGA_SDI_GPIO_Port, PGA_SDI_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(PGA_SDI_GPIO_Port, PGA_SDI_Pin, GPIO_PIN_RESET);
    }
    for (volatile int d = 0; d < 50; d++) __NOP();

    /* Rising edge on SCLK samples SDI */
    HAL_GPIO_WritePin(PGA_SCLK_GPIO_Port, PGA_SCLK_Pin, GPIO_PIN_SET);
    for (volatile int d = 0; d < 50; d++) __NOP();

    /* Falling edge on SCLK */
    HAL_GPIO_WritePin(PGA_SCLK_GPIO_Port, PGA_SCLK_Pin, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 50; d++) __NOP();
  }

  /* Deassert CS HIGH to latch the 16-bit volume setting */
  HAL_GPIO_WritePin(PGA_CS1_GPIO_Port, PGA_CS1_Pin, GPIO_PIN_SET);
  for (volatile int d = 0; d < 50; d++) __NOP();
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

#ifdef  USE_FULL_ASSERT
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
