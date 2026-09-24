# Audiometer AX60 Project Rules & Guidelines

## 1. Project Overview
- **Device**: Clinical Audiometer AX60
- **MCU**: STM32F446VET6 (ARM Cortex-M4 @ 180MHz, 512KB Flash, 128KB SRAM, LQFP100)
- **Primary Toolchain**: STM32CubeIDE / GCC ARM Embedded / STM32CubeMX v6.8.1 (HAL v1.27.1)
- **Frontend / Control Panel**: Modern HTML5/JS WebUI with Web Serial API communication (`WebUI/index.html`)

---

## 2. Hardware Architecture & Key Pin Mappings

### Audio Engine & Signal Chain
- **Audio ADC**: PCM1808 connected via **SAI1 Block A**
  - `SAI1_MCLK_A` (PE2), `SAI1_FS_A` (PE4), `SAI1_SCK_A` (PE5), `SAI1_SD_A` (PE6)
  - Data control: `PCM1808_DATA_EN` (PD10), `PCM_MIC_Control` (PA9)
- **Programmable Gain Amplifier (PGA)**:
  - `PGA_SCLK` (PB10), `PGA_CS1` (PB12), `PGA_MUTE_1` (PB13), `ZCEN` (Zero Crossing Enable - PB14)
  - `PGA_SDI` (PC1), `PGA_SDO` (PC2)
- **Output Channel Switching & Amplification**:
  - Air Conduction: `AC_Left_EN` (PE13), `AC_Right_EN` (PE14)
  - Bone Conduction: `BC_EN` (PE7), `BC_L_R_EN` (PE10)
  - Insert Earphone: `INSERT_EP_EN` (PE12)
  - Free Field: `FF_EN` (PE9)
  - Masking Noise: `WN_EN` (White Noise Enable - PE8)
  - Internal Speaker: `INTERNAL_SPK_EN` (PA15)
  - Analog Op-Amp: `OPA_EN` (PB2)
- **Microphone & Talkover**:
  - `MIC_EN` (PD6), `MIC_AUX_EN` (PD5), `TALKOVER_EN` (PD4)

### User Inputs & Indicators
- **Patient Response Switch**: `pat_response_switch` on **PB3** configured with `EXTI3`
- **Stimulus Indicators**: `STIMULUS1` (PA6), `STIMULUS2` (PB15)

### Peripherals & Storage
- **SPI3**: `SD_CS` (PC12), `FRAM_CS` (PC13), `SCK` (PC10), `MISO` (PC11), `MOSI` (PB5)
- **I2C3**: `I2C3_SCL` (PA8), `I2C3_SDA` (PC9)
- **RTC**: `RTC_EN` (PB1), `RTC_CLK` (PB6), `RTC_IO` (PB9)
- **Power Monitoring**: `BAT_READ` (ADC1_IN0 PA0), `POWER_DETECTION` (PA1)

### Host Communication
- **USART2**: TX (PA2), RX (PA3)
- **USART3**: TX (PD8), RX (PC5)
- **USB OTG FS**: ID (PA10), DM (PA11), DP (PA12)

---

## 3. Firmware Coding Standards & Safe Editing Boundaries
1. **STM32CubeMX Regeneration Safety**:
   - Always place application code strictly inside `/* USER CODE BEGIN ... */` and `/* USER CODE END ... */` comments in all CubeMX generated files (`main.c`, `stm32f4xx_it.c`, `stm32f4xx_hal_msp.c`).
   - Never remove or alter `USER CODE` markers.
2. **Interrupt Safety (NVIC)**:
   - Keep EXTI (`PB3` patient response) and DMA callbacks fast and deterministic. Never execute blocking delays (`HAL_Delay`) inside ISRs.
   - Defer heavy processing to the main loop using atomic/volatile flags or RTOS notifications.
3. **PGA & Audio Path Protection**:
   - Always mute (`PGA_MUTE_1` = active) before toggling output channel relays (`AC_Left_EN`, `BC_EN`, etc.) or stepping up PGA gain to prevent loud audio clicks/pops in patient headphones.
   - Respect zero-crossing (`ZCEN`) timing during gain transitions.

---

## 4. WebUI & Hardware Interface Standards
1. **Transport**: Web Serial API (`navigator.serial`) at 115200 baud (or configurable).
2. **Protocol Framing**:
   - Use structured commands with start/end markers (e.g. `$<COMMAND>:<PARAMS>*<CHECKSUM>\n`).
   - Implement acknowledgement (`ACK`/`NACK`) handshakes for frequency and attenuation changes.
3. **Safety Interlocks**:
   - WebUI must disarm tone stimulus if connection heartbeat fails.
