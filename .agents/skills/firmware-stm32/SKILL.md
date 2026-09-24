---
name: firmware-stm32
description: >-
  Expert guidelines for STM32 embedded C firmware, STM32Cube HAL/LL drivers,
  clock tree setup, SAI audio streaming, DMA, EXTI interrupts, and peripheral drivers.
  Use when writing, debugging, refactoring, or optimizing microcontroller firmware for STM32.
---

# STM32 Firmware Development Skill

## Overview
This skill provides best practices, peripheral patterns, and safety routines tailored for STM32F4 series microcontrollers (ARM Cortex-M4), specifically the STM32F446 on the Audiometer AX60 platform.

---

## 1. STM32CubeMX Code Integrity
- **USER CODE blocks**: All custom user definitions, includes, functions, variables, and loop logic must reside strictly inside `USER CODE BEGIN` and `USER CODE END` blocks.
- **Clock Initialization**: Verify HSE/PLL configuration if modifying audio sampling rates (SAI clock must be an exact integer multiple of $256 \times F_s$ or $512 \times F_s$ to minimize jitter).

---

## 2. Audio Subsystem (SAI & PCM1808)
- **SAI Configuration**:
  - `SAI1_Block_A` configured as Master/Slave Receiver depending on ADC master clock generator.
  - Continuous DMA streaming must use double buffering or circular DMA (`DMA_CIRCULAR`) to prevent underrun/overrun.
- **DMA Callback Discipline**:
  - Implement `HAL_SAI_RxHalfCpltCallback()` and `HAL_SAI_RxCpltCallback()` for ping-pong buffer processing.
  - Never execute blocking calls or floating-point transforms directly within the SAI DMA ISR; queue buffers for processing in the main loop or dedicated FreeRTOS task.

---

## 3. Programmable Gain Amplifier (PGA) & Attenuation Control
- **Pop/Click Suppression Sequence**:
  Whenever switching channels or attenuator levels (dB HL):
  1. Assert `PGA_MUTE_1` (High / Active).
  2. Small settling delay ($2\text{--}5\text{ ms}$) or wait for Zero Crossing (`ZCEN`).
  3. Update PGA gain word via SPI/bitbang (`PGA_SCLK`, `PGA_SDI`, `PGA_CS1`).
  4. Toggle output multiplexer pins (`AC_Left_EN`, `AC_Right_EN`, `BC_EN`, etc.).
  5. Deassert `PGA_MUTE_1`.

---

## 4. Interrupts & Patient Response Handling
- **Patient Response Switch (PB3 / EXTI3)**:
  - Configure falling/rising edge trigger with debounce handling.
  - Software debounce: Check timestamp against `HAL_GetTick()` ($> 50\text{ ms}$ refractory window) or dedicated hardware timer to eliminate switch bounce.
  - Set volatile flag `volatile uint8_t patient_pressed` and notify host/WebUI immediately.

---

## 5. Non-Volatile Storage (FRAM & SD Card)
- **Calibration Data**: Store audiometer transducer calibration curves (RETPL - Reference Equivalent Threshold Sound Pressure Levels) in SPI FRAM (`FRAM_CS` on PC13) for instant read/write cycles without EEPROM wear-out.
- **Patient Records / Test Logs**: Write bulk audiometric logs to FATFS on the SPI SD card (`SD_CS` on PC12).
