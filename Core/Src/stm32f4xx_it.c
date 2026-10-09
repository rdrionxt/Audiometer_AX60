/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
 }

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern void WebUI_UartRxByte(uint8_t byte);

/**
  * @brief This function handles USART2 global interrupt (Dedicated Display Interface).
  */
void USART2_IRQHandler(void)
{
  uint32_t sr = huart2.Instance->SR;
  if (sr & (USART_SR_RXNE | USART_SR_ORE))
  {
    /* Read DR to clear RXNE and ORE flags.
     * Note: Display frames (Proculus DGUS / UnicView AD) belong to dedicated display parser,
     * not WebUI protocol. Do not inject into WebUI buffer. */
    volatile uint8_t dummy = (uint8_t)(huart2.Instance->DR & 0xFF);
    (void)dummy;
  }
}

/**
  * @brief This function handles USART3 global interrupt for WebUI serial protocol.
  */
void USART3_IRQHandler(void)
{
  uint32_t sr = huart3.Instance->SR;
  if (sr & (USART_SR_RXNE | USART_SR_ORE))
  {
    uint8_t ch = (uint8_t)(huart3.Instance->DR & 0xFF);
    if (sr & USART_SR_RXNE)
    {
      WebUI_UartRxByte(ch);
    }
  }
}

extern I2S_HandleTypeDef hi2s1;
extern DMA_HandleTypeDef hdma_spi1_tx;

/**
  * @brief This function handles SPI1 / I2S1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  HAL_I2S_IRQHandler(&hi2s1);
}

/**
  * @brief This function handles DMA2 Stream 3 global interrupt for continuous I2S1 TX streaming.
  */
void DMA2_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

extern SAI_HandleTypeDef hsai_BlockA1;
extern DMA_HandleTypeDef hdma_sai1_a;

/**
  * @brief This function handles SAI1 global interrupt.
  */
void SAI1_IRQHandler(void)
{
  HAL_SAI_IRQHandler(&hsai_BlockA1);
}

/**
  * @brief This function handles DMA2 Stream 1 global interrupt for continuous SAI1 RX streaming.
  */
void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_sai1_a);
}

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/**
  * @brief This function handles USB On-The-Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

/* USER CODE END 1 */
