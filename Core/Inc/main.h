/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
  MIC_SEL_MIC1 = 0,   /* IC1 Left Channel  */
  MIC_SEL_MIC2,       /* IC1 Right Channel */
  MIC_SEL_MIC3,       /* IC2 Left Channel  */
  MIC_SEL_MIC4,       /* IC2 Right Channel */
  MIC_SEL_IC1_STEREO, /* IC1 Stereo (MIC1 -> DAC L, MIC2 -> DAC R) */
  MIC_SEL_IC2_STEREO  /* IC2 Stereo (MIC3 -> DAC L, MIC4 -> DAC R) */
} MicSelection_t;

/* Tone / Noise Generator Modes */
typedef enum {
  TONE_MODE_SINE = 0,        /* Pure Tone (Sinusoid) */
  TONE_MODE_WARBLE = 1,      /* Frequency Modulated (Warble) Tone: 5 Hz, 5% dev */
  TONE_MODE_WHITE_NOISE = 2, /* Broadband White Noise (PRNG) */
  TONE_MODE_NBN = 3,         /* Narrow Band Noise (2nd-order IIR BPF, Q=4.318, 1/3 octave) */
  TONE_MODE_OFF = 4          /* Muted / Silence */
} ToneMode_t;

/* Audio Output Ear Routing */
typedef enum {
  AUDIO_EAR_LEFT = 0,
  AUDIO_EAR_RIGHT = 1,
  AUDIO_EAR_BOTH = 2
} AudioEar_t;

/* Channel Synthesizer DSP State */
typedef struct {
  int mode;                  /* ToneMode_t */
  float freq;                /* Center or Carrier Frequency in Hz */
  float amp;                 /* Channel amplitude scalar (0.0 to 1.0) */

  uint32_t acc;              /* 32-bit Phase Accumulator */
  uint32_t inc;              /* Phase increment per sample */

  float cp;
  float mp;
  uint32_t mp_acc;           /* Modulation phase accumulator (Warble) */
  float warble_rate;         /* Modulation rate (typically 5.0 Hz) */
  float warble_dev;          /* Modulation deviation percentage (typically 5.0%) */

  uint32_t noise_seed;       /* 32-bit PRNG seed */

  /* Narrow Band Noise (NBN) Biquad Bandpass Filter State */
  float b0, b2, a1, a2;
  float bp_x1, bp_x2;
  float bp_y1, bp_y2;
} ChannelState;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define AUDIO_BUFFER_SIZE   128  /* Number of 32-bit audio slots (64 Left + 64 Right) */
#define HALF_BUFFER_SIZE    (AUDIO_BUFFER_SIZE / 2)
#define AUDIO_SAMPLE_RATE_HZ 48000.0f
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void PeriphCommonClock_Config(void);
void MX_SAI1_Init(void);
void MX_I2S1_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

extern I2S_HandleTypeDef hi2s1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN EFP */
void Audio_Select_Microphone(MicSelection_t mic);
void Audio_Set_Gain_Linear(float gain);
void Audio_Set_Gain_dB(float gain_db);
float Audio_Get_Gain_dB(void);
void Process_Mic_To_DAC(int32_t *pSrc, int32_t *pDst, uint16_t length, float gain);

/* Tone & Noise Generation APIs (Hardware I2S1 Protocol - PA4/PA5/PA7) */
void Audio_Channel_Init(ChannelState *ch, int mode, float freq_hz, uint32_t seed);
int16_t Audio_Process_Channel(ChannelState *ch);
HAL_StatusTypeDef Play_SineWave_I2S(float freq_left_hz, float freq_right_hz, uint32_t duration_ms, float volume_left, float volume_right);
HAL_StatusTypeDef Audio_Play_Tone_I2S(float freq_hz, uint32_t duration_ms,
                                      int stim_mode, int mask_mode,
                                      float vol_stim, float vol_mask,
                                      uint8_t ear);
HAL_StatusTypeDef Audio_Play_PureTone_I2S(float freq_hz, uint32_t duration_ms, float volume, uint8_t ear);
HAL_StatusTypeDef Audio_Play_Warble_I2S(float freq_hz, uint32_t duration_ms, float volume, uint8_t ear);
HAL_StatusTypeDef Audio_Play_NBN_I2S(float freq_hz, uint32_t duration_ms, float volume, uint8_t ear);
HAL_StatusTypeDef Audio_Play_WhiteNoise_I2S(uint32_t duration_ms, float volume, uint8_t ear);

/* PGA2311 Volume Control */
void PGA2311_SetVolume(uint8_t left_gain, uint8_t right_gain);

/* WebUI Serial Protocol Handling */
void WebUI_UartRxByte(uint8_t byte);
void WebUI_Poll(void);
void Audio_I2S_ISR_Handler(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* Hardware I2S1 PCM5102 DAC Pins (PA4=LRCK, PA5=BCK, PA7=DIN) */
#define I2S_LRCK_Pin               GPIO_PIN_4
#define I2S_LRCK_GPIO_Port         GPIOA
#define I2S_BCK_Pin                GPIO_PIN_5
#define I2S_BCK_GPIO_Port          GPIOA
#define I2S_DIN_Pin                GPIO_PIN_7
#define I2S_DIN_GPIO_Port          GPIOA

/* SAI PCM5102 DAC Pins (Alternate provision) */
#define SAI_PCM5102_MCLK_Pin       GPIO_PIN_2
#define SAI_PCM5102_MCLK_GPIO_Port GPIOE
#define SAI_PCM5102_SCK_Pin        GPIO_PIN_5
#define SAI_PCM5102_SCK_GPIO_Port  GPIOE
#define SAI_PCM5102_FS_Pin         GPIO_PIN_11
#define SAI_PCM5102_FS_GPIO_Port   GPIOD
#define SAI_PCM5102_SD_Pin         GPIO_PIN_12
#define SAI_PCM5102_SD_GPIO_Port   GPIOD

/* Audio Routing & Output Enables */
#define BC_EN_Pin                 GPIO_PIN_7
#define BC_EN_GPIO_Port           GPIOE
#define WN_EN_Pin                 GPIO_PIN_8
#define WN_EN_GPIO_Port           GPIOE
#define FF_EN_Pin                 GPIO_PIN_9
#define FF_EN_GPIO_Port           GPIOE
#define BC_L_R_EN_Pin             GPIO_PIN_10
#define BC_L_R_EN_GPIO_Port       GPIOE
#define INSERT_EP_EN_Pin          GPIO_PIN_12
#define INSERT_EP_EN_GPIO_Port    GPIOE
#define AC_Left_EN_Pin            GPIO_PIN_13
#define AC_Left_EN_GPIO_Port      GPIOE
#define AC_Right_EN_Pin           GPIO_PIN_14
#define AC_Right_EN_GPIO_Port     GPIOE
#define INTERNAL_SPK_EN_Pin       GPIO_PIN_15
#define INTERNAL_SPK_EN_GPIO_Port GPIOA

#define TALKOVER_EN_Pin           GPIO_PIN_4
#define TALKOVER_EN_GPIO_Port     GPIOD
#define MIC_AUX_EN_Pin            GPIO_PIN_5
#define MIC_AUX_EN_GPIO_Port      GPIOD
#define MIC_EN_Pin                GPIO_PIN_6
#define MIC_EN_GPIO_Port          GPIOD
#define OPA_EN_Pin                GPIO_PIN_2
#define OPA_EN_GPIO_Port          GPIOB
#define PCM_MIC_Control_Pin       GPIO_PIN_9
#define PCM_MIC_Control_GPIO_Port GPIOA

#define STIMULUS1_Pin             GPIO_PIN_6
#define STIMULUS1_GPIO_Port       GPIOA
#define STIMULUS2_Pin             GPIO_PIN_15
#define STIMULUS2_GPIO_Port       GPIOB

/* Codec Controls */
#define PCM1808_DATA_EN_Pin       GPIO_PIN_10
#define PCM1808_DATA_EN_GPIO_Port GPIOD

/* PGA (Programmable Gain Amplifier) & Zero Cross */
#define PGA_SDI_Pin               GPIO_PIN_1
#define PGA_SDI_GPIO_Port         GPIOC
#define PGA_SDO_Pin               GPIO_PIN_2
#define PGA_SDO_GPIO_Port         GPIOC
#define PGA_SCLK_Pin              GPIO_PIN_10
#define PGA_SCLK_GPIO_Port        GPIOB
#define PGA_CS1_Pin               GPIO_PIN_12
#define PGA_CS1_GPIO_Port         GPIOB
#define PGA_MUTE_1_Pin            GPIO_PIN_13
#define PGA_MUTE_1_GPIO_Port      GPIOB
#define ZCEN_Pin                  GPIO_PIN_14
#define ZCEN_GPIO_Port            GPIOB

/* RTC (DS1302SN+ 3-Wire Interface) */
#define RTC_EN_Pin                GPIO_PIN_1
#define RTC_EN_GPIO_Port          GPIOB
#define RTC_CLK_Pin               GPIO_PIN_6
#define RTC_CLK_GPIO_Port         GPIOB
#define RTC_IO_Pin                GPIO_PIN_9
#define RTC_IO_GPIO_Port          GPIOB

/* Storage Chip Selects */
#define FRAM_CS_Pin               GPIO_PIN_13
#define FRAM_CS_GPIO_Port         GPIOC
#define SD_CS_Pin                 GPIO_PIN_12
#define SD_CS_GPIO_Port           GPIOC

/* Power & Patient Response Inputs */
#define BAT_READ_Pin              GPIO_PIN_0
#define BAT_READ_GPIO_Port        GPIOA
#define POWER_DETECTION_Pin       GPIO_PIN_1
#define POWER_DETECTION_GPIO_Port GPIOA
#define PAT_RESPONSE_SW_Pin       GPIO_PIN_3
#define PAT_RESPONSE_SW_GPIO_Port GPIOB
#define PAT_RESPONSE_SW_EXTI_IRQn EXTI3_IRQn

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
