/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
  * Initializes the SAI MSP.
  */
void HAL_SAI_MspInit(SAI_HandleTypeDef* hsai)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  if (hsai->Instance == SAI1_Block_A)
  {
    /* Configure PLLI2S clock for SAI1 */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
    if ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI)
    {
      PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
    }
    else
    {
      PeriphClkInitStruct.PLLI2S.PLLI2SM = 8;
    }
    PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
    PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
    PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
    PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
    PeriphClkInitStruct.PLLI2SDivQ = 1;
    PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLI2S;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_SAI1_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /** PCM1808 Audio ADC GPIO Configuration
      PE2     ------> SAI1_MCLK_A (SCKI master clock for PCM1808: 256*Fs = 12.288MHz)
      PE4     ------> SAI1_FS_A   (LRCK frame sync: 48kHz)
      PE5     ------> SAI1_SCK_A  (BCK bit clock: 64*Fs = 3.072MHz)
      PE6     ------> SAI1_SD_A   (DOUT serial data in from PCM1808)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /** PCM1808_DATA_EN Pin Configuration:
      PD10    ------> PCM1808_DATA_EN (HIGH = IC1, LOW = IC2)
    */
    GPIO_InitStruct.Pin = PCM1808_DATA_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PCM1808_DATA_EN_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(PCM1808_DATA_EN_GPIO_Port, PCM1808_DATA_EN_Pin, GPIO_PIN_SET); /* Default: IC1 */

    /* Configure DMA2 Stream 1 Channel 0 for SAI1_Block_A RX */
    extern DMA_HandleTypeDef hdma_sai1_a;
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_sai1_a.Instance = DMA2_Stream1;
    hdma_sai1_a.Init.Channel = DMA_CHANNEL_0;
    hdma_sai1_a.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_sai1_a.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sai1_a.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sai1_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sai1_a.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_sai1_a.Init.Mode = DMA_CIRCULAR;
    hdma_sai1_a.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_sai1_a.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_sai1_a) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hsai, hdmarx, hdma_sai1_a);

    /* DMA2 Stream 1 Interrupt Init */
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  }
}

/**
  * De-Initializes the SAI MSP.
  */
void HAL_SAI_MspDeInit(SAI_HandleTypeDef* hsai)
{
  if (hsai->Instance == SAI1_Block_A)
  {
    /* Disable DMA IRQ and de-init DMA */
    HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
    HAL_DMA_DeInit(hsai->hdmarx);

    /* Peripheral clock disable */
    __HAL_RCC_SAI1_CLK_DISABLE();

    /** SAI1_Block_A GPIO Configuration
      PE2     ------> SAI1_MCLK_A
      PE4     ------> SAI1_FS_A
      PE5     ------> SAI1_SCK_A
      PE6     ------> SAI1_SD_A
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
  }
}

/* USER CODE BEGIN 1 */

/**
* @brief I2S MSP Initialization
* Configures PLLI2S clock and GPIO pins for I2S1:
*   PA4 ------> I2S1_WS (I2S_LRCK_PCM5102 via R4 0R, R21 22R to PCM5102 Pin 15)
*   PA5 ------> I2S1_CK (I2S_BCK_PCM5102 via R5 0R, R23 22R to PCM5102 Pin 13)
*   PA7 ------> I2S1_SD (I2S_DIN_PCM5102 via R14 0R, R22 22R to PCM5102 Pin 14)
* @param hi2s: I2S handle pointer
* @retval None
*/
void HAL_I2S_MspInit(I2S_HandleTypeDef* hi2s)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  if (hi2s->Instance == SPI1)
  {
    /* Configure PLLI2S for I2S1 on APB2 and SAI1 */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB2 | RCC_PERIPHCLK_SAI1;
    if ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI)
    {
      PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
    }
    else
    {
      PeriphClkInitStruct.PLLI2S.PLLI2SM = 8;
    }
    PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
    PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
    PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
    PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
    PeriphClkInitStruct.PLLI2SDivQ = 1;
    PeriphClkInitStruct.I2sApb2ClockSelection = RCC_I2SAPB2CLKSOURCE_PLLI2S;
    PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLI2S;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Enable Peripheral Clocks */
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure I2S1 GPIO Pins: PA4 (WS), PA5 (CK), PA7 (SD) */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* I2S1 / SPI1 Interrupt Init */
    HAL_NVIC_SetPriority(SPI1_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);

    /* I2S1 DMA Init (DMA2 Stream 3 Channel 3 for continuous glitch-free stereo streaming) */
    extern DMA_HandleTypeDef hdma_spi1_tx;
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_spi1_tx.Instance = DMA2_Stream3;
    hdma_spi1_tx.Init.Channel = DMA_CHANNEL_3;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi1_tx.Init.Mode = DMA_CIRCULAR;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hi2s, hdmatx, hdma_spi1_tx);

    /* DMA2 Stream 3 Interrupt Init */
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  }
}

/**
* @brief I2S MSP De-Initialization
* @param hi2s: I2S handle pointer
* @retval None
*/
void HAL_I2S_MspDeInit(I2S_HandleTypeDef* hi2s)
{
  if (hi2s->Instance == SPI1)
  {
    /* Disable SPI1 Interrupt */
    HAL_NVIC_DisableIRQ(SPI1_IRQn);

    /* De-Initialize DMA */
    HAL_DMA_DeInit(hi2s->hdmatx);

    __HAL_RCC_SPI1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7);
  }
}

/**
* @brief UART MSP Initialization
* Configures USART2 on PA2 (TX) and PA3 (RX) for Display / HMI interface.
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (huart->Instance == USART2)
  {
    /* Enable USART2 & GPIOA Clocks */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /** USART2 GPIO Configuration
      PA2     ------> USART2_TX
      PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
  else if (huart->Instance == USART3)
  {
    /* Enable USART3, GPIOD & GPIOC Clocks */
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /** USART3 GPIO Configuration
      PD8     ------> USART3_TX
      PC5     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  }
}

/**
* @brief UART MSP De-Initialization
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if (huart->Instance == USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
  else if (huart->Instance == USART3)
  {
    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_5);
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  }
}

/* USER CODE END 1 */

