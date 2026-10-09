# AX60 Clinical Audiometer — Hardware & Feature GPIO Control Specification

This document is the authoritative hardware and firmware reference for controlling every feature and audio routing mode on the **AX60 Audiometer (STM32F446VET6)**.

---

## 1. Master GPIO Pin Mapping Table

| Pin Name | Port / Pin | Alternate / Mode | Active Level | Default State | Description / Hardware Destination |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **AC_Left_EN** | `PE13` | GPIO Output | HIGH | LOW | Air Conduction (AC) Left Earphone Relay (TDH-39 / DD45) |
| **AC_Right_EN** | `PE14` | GPIO Output | HIGH | LOW | Air Conduction (AC) Right Earphone Relay |
| **BC_EN** | `PE7` | GPIO Output | HIGH | LOW | Bone Conduction (BC) Transducer Relay (Radioear B71 / B81) |
| **INSERT_EP_EN** | `PE12` | GPIO Output | HIGH | LOW | Insert Earphone Relay (ER-3A / ER-5A) |
| **FF_EN** | `PE9` | GPIO Output | HIGH | LOW | Free Field (FF) Loudspeaker Output Relay |
| **WN_EN / MH_EN** | `PE8` | GPIO Output | HIGH | LOW | White Noise / Monitor Headphone Enable Relay |
| **INTERNAL_SPK_EN**| `PA15` | GPIO Output | HIGH | LOW | Internal Cabinet Speaker Relay (Examiner Talkback) |
| **OPA_EN** | `PB2` | GPIO Output | HIGH | HIGH | High-Current Analog Output Op-Amp Enable |
| **TALKOVER_EN** | `PD4` | GPIO Output | HIGH=Ext, LOW=Int | LOW | Talkover Mode: Mic Selector (HIGH=External, LOW=Internal) |
| **MIC_AUX_EN** | `PD5` | GPIO Output | HIGH | LOW | Auxiliary 3.5mm Stereo Audio Input Buffer Enable |
| **MIC_EN** | `PD6` | GPIO Output | HIGH | LOW | Patient Talkback Microphone Pre-Amp Enable |
| **PCM_MIC_Control** | `PA9` | GPIO Output | LOW=DAC, HIGH=Mux | LOW | Fixed LOW in Digital Loopback mode (PCM5102 DAC feeds PGA2311) |
| **PCM1808_DATA_EN**| `PD10` | GPIO Output | HIGH=IC1, LOW=IC2 | HIGH | Audio ADC Select: HIGH=IC1 (Mic 1 L / Mic 2 R), LOW=IC2 (Mic 3 L / Mic 4 R) |
| **PGA_MUTE_1** | `PB13` | GPIO Output | LOW=Mute, HIGH=Active| LOW (Muted) | PGA2311 Hardware Output Mute Control |
| **ZCEN** | `PB14` | GPIO Output | HIGH=Zero Cross, LOW=Imm | LOW | PGA2311 Zero-Crossing Gain Step Synchronizer |
| **PGA_CS1** | `PB12` | GPIO Output | Active LOW | HIGH | PGA2311 SPI Chip Select |
| **PGA_SCLK** | `PB10` | GPIO Output | Rising Edge Clock | LOW | PGA2311 SPI Serial Clock |
| **PGA_SDI** | `PC1` | GPIO Output | Data In (MOSI) | LOW | PGA2311 Serial Data Input |
| **PGA_SDO** | `PC2` | GPIO Input | Data Out (MISO) | High-Z | PGA2311 Serial Data Output |
| **PAT_RESPONSE_SW**| `PB3` | EXTI3 Input | Active LOW | HIGH (Pull-up) | Patient Push-Button Response Handswitch |
| **BAT_READ** | `PA0` | ADC1_IN0 | Analog | Analog In | Battery Voltage Divider Sensing |
| **POWER_DETECTION**| `PA1` | GPIO Input | HIGH=Connected | In | DC Mains / Wall Adapter Power Detector |
| **RTC_EN (CE)** | `PB1` | GPIO Output | Active HIGH | LOW | DS1302 Real-Time Clock Chip Enable |
| **RTC_CLK (SCLK)** | `PB6` | GPIO Output | Rising Edge | LOW | DS1302 Serial Clock |
| **RTC_IO (DAT)** | `PB9` | GPIO Bidirectional| Data Line | In/Out | DS1302 Bidirectional Data Pin |
| **FRAM_CS** | `PC13` | GPIO Output | Active LOW | HIGH | SPI3 Non-Volatile FRAM Chip Select (Calibration Tables) |
| **SD_CS** | `PC12` | GPIO Output | Active LOW | HIGH | SPI3 MicroSD Card Chip Select (Patient Records) |
| **USART2 TX / RX** | `PA2 / PA3` | AF7 (`USART2`) | Serial TTL | -- | Proculus 7-inch Intelligent Display (115200 8N1) |
| **USART3 TX / RX** | `PD8 / PC5` | AF7 (`USART3`) | Serial TTL | -- | CH340G USB Serial (WebUI / Future Thermal Printer) |

---

## 2. Feature-by-Feature GPIO Control Matrix

### Quick Reference Summary Table

| Feature Mode | `PCM1808_DATA_EN` (PD10) | `PCM_MIC_Control` (PA9) | `TALKOVER_EN` (PD4) | `MIC_EN` (PD6) | `MIC_AUX_EN` (PD5) | `AC_Left` (PE13) | `AC_Right` (PE14) | `BC_EN` (PE7) | `INSERT_EP` (PE12) | `FF_EN` (PE9) | `MH_EN` (PE8) | `INTERNAL_SPK` (PA15) | `OPA_EN` (PB2) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1. Talkover (Ext Mic)** | **SET (1: IC1)** | **0** | **SET (1)** | 0 | 0 | **1** | **1** | 0 | 0 | 0 | 0 | 0 | **1** |
| **2. Talkover (Int Mic)** | **RESET (0: IC2)**| **0** | **RESET (0)**| 0 | 0 | **1** | **1** | 0 | 0 | 0 | 0 | 0 | **1** |
| **3. Talkback (Cabinet Spk)**| **SET (1: IC1)**| **0** | 0 | **SET (1)** | 0 | 0 | 0 | 0 | 0 | 0 | 0 | **SET (1)** | **1** |
| **4. Talkback (Headphones)** | **SET (1: IC1)**| **0** | 0 | **SET (1)** | 0 | 0 | 0 | 0 | 0 | 0 | **SET (1)** | 0 | **1** |
| **5. AUX Audio Input** | **RESET (0: IC2)**| **0** | 0 | 0 | **SET (1)** | **1** | **1** | 0 | 0 | 0 | 0 | 0 | **1** |
| **6. AC Pure Tone (Left)** | **SET (1: IC1)** | **0**| 0 | 0 | 0 | **1** | **1\*** | 0 | 0 | 0 | 0 | 0 | **1** |
| **7. AC Pure Tone (Right)**| **SET (1: IC1)** | **0**| 0 | 0 | 0 | **1\*** | **1** | 0 | 0 | 0 | 0 | 0 | **1** |
| **8. Bone Conduction (BC)**| **SET (1: IC1)** | **0**| 0 | 0 | 0 | 0 | 0 | **1** | **1\*\*** | 0 | 0 | 0 | **1** |
| **9. Insert Earphone** | **SET (1: IC1)** | **0**| 0 | 0 | 0 | 0 | 0 | 0 | **1** | 0 | 0 | 0 | **1** |
| **10. Free Field (Spk)** | **SET (1: IC1)** | **0**| 0 | 0 | 0 | 0 | 0 | 0 | 0 | **1** | 0 | 0 | **1** |
| **11. Standby / Mute** | **SET (1: IC1)** | **0**| 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | **1** |

*\*Note: In AC testing, opposite ear relay is turned ON when contralateral masking noise is active.*  
*\*\*Note: In BC testing, INSERT_EP_EN is turned ON to deliver contralateral narrowband masking noise.*

---

## 3. Detailed Operation & Code Implementation for Each Feature

### Feature 1: Talkover (Examiner → Patient)
* **Goal**: Examiner speaks into the console mic to communicate instructions directly into the patient's headphones.
* **GPIO States**:
  - `PCM_MIC_Control` (`PA9`) → **HIGH (1)** (Bypasses DAC tone generator; routes analog mic pre-amp)
  - `TALKOVER_EN` (`PD4`):
    - **HIGH (1)** = External Gooseneck / Headset Microphone
    - **LOW (0)** = Internal Front-Panel Console Microphone
  - `MIC_EN` (`PD6`) → **LOW (0)** (Patient mic disabled)
  - `MIC_AUX_EN` (`PD5`) → **LOW (0)** (Auxiliary line disabled)
  - `AC_Left_EN` (`PE13`) & `AC_Right_EN` (`PE14`) → **HIGH (1)** (Both patient headphone sides enabled)
  - `BC_EN` (`PE7`), `INSERT_EP_EN` (`PE12`), `FF_EN` (`PE9`) → **LOW (0)**
  - `MH_EN` (`PE8`), `INTERNAL_SPK_EN` (`PA15`) → **LOW (0)** (Examiner speakers muted)
  - `OPA_EN` (`PB2`) → **HIGH (1)** (Output op-amp active)
  - **PGA Volume**: Set to unity gain (`192` = 0 dB) via `PGA2311_SetVolume(192, 192)`.

```c
void Feature_Enable_Talkover(uint8_t use_external_mic)
{
    /* 1. Mute PGA to prevent audio pop */
    PGA2311_Mute();

    /* 2. Select Mic over DAC */
    HAL_GPIO_WritePin(GPIOA, PCM_MIC_Control_Pin, GPIO_PIN_SET);

    /* 3. Choose External vs Internal Examiner Mic */
    HAL_GPIO_WritePin(GPIOD, TALKOVER_EN_Pin, use_external_mic ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, MIC_EN_Pin | MIC_AUX_EN_Pin, GPIO_PIN_RESET);

    /* 4. Route audio strictly to patient headphones */
    HAL_GPIO_WritePin(GPIOE, AC_Left_EN_Pin | AC_Right_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, BC_EN_Pin | INSERT_EP_EN_Pin | FF_EN_Pin | MH_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, INTERNAL_SPK_EN_Pin, GPIO_PIN_RESET);

    /* 5. Set unity gain and unmute */
    PGA2311_SetVolume(192, 192);
    PGA2311_Unmute();
}
```

---

### Feature 2: Talkback (Patient → Examiner)
* **Goal**: Allows patient inside a soundproof booth to speak to the examiner. Prevents acoustic feedback howling by isolating patient transducers.
* **GPIO States**:
  - `PCM_MIC_Control` (`PA9`) → **HIGH (1)**
  - `MIC_EN` (`PD6`) → **HIGH (1)** (Activates patient booth mic amplifier)
  - `TALKOVER_EN` (`PD4`) → **LOW (0)**
  - `MIC_AUX_EN` (`PD5`) → **LOW (0)**
  - **Patient Isolation**: `AC_Left_EN`=0, `AC_Right_EN`=0, `BC_EN`=0, `INSERT_EP_EN`=0, `FF_EN`=0
  - **Examiner Destination**:
    - **Cabinet Speaker**: `INTERNAL_SPK_EN` (`PA15`) → **HIGH (1)**, `MH_EN` (`PE8`) → **LOW (0)**
    - **Monitor Headphones**: `MH_EN` (`PE8`) → **HIGH (1)**, `INTERNAL_SPK_EN` (`PA15`) → **LOW (0)**

```c
void Feature_Enable_Talkback(uint8_t destination_is_internal_speaker)
{
    PGA2311_Mute();

    /* Select Patient Mic */
    HAL_GPIO_WritePin(GPIOA, PCM_MIC_Control_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, MIC_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, TALKOVER_EN_Pin | MIC_AUX_EN_Pin, GPIO_PIN_RESET);

    /* Mute all patient transducers to prevent feedback loop */
    HAL_GPIO_WritePin(GPIOE, AC_Left_EN_Pin | AC_Right_EN_Pin | BC_EN_Pin | INSERT_EP_EN_Pin | FF_EN_Pin, GPIO_PIN_RESET);

    /* Route to Examiner Monitor */
    if (destination_is_internal_speaker) {
        HAL_GPIO_WritePin(GPIOA, INTERNAL_SPK_EN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, MH_EN_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOE, MH_EN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, INTERNAL_SPK_EN_Pin, GPIO_PIN_RESET);
    }

    PGA2311_SetVolume(192, 192);
    PGA2311_Unmute();
}
```

---

### Feature 3: Auxiliary Audio Input (Speech Audiometry / CD / AUX)
* **Goal**: Feeds pre-recorded speech material from a 3.5mm input jack into the patient's headphones.
* **GPIO States**:
  - `PCM_MIC_Control` (`PA9`) → **HIGH (1)**
  - `MIC_AUX_EN` (`PD5`) → **HIGH (1)**
  - `TALKOVER_EN` (`PD4`) → **LOW (0)**, `MIC_EN` (`PD6`) → **LOW (0)**
  - `AC_Left_EN` (`PE13`) & `AC_Right_EN` (`PE14`) → **HIGH (1)**
  - `BC_EN`=0, `INSERT_EP_EN`=0, `FF_EN`=0, `MH_EN`=0, `INTERNAL_SPK_EN`=0
  - `OPA_EN` (`PB2`) → **HIGH (1)**

---

### Feature 4: Air Conduction Pure Tone & Warble (AC Left / Right)
* **Goal**: Delivers pure tones (125 Hz to 8000 Hz) or Warble tones at -10 dB to 120 dB HL.
* **GPIO States**:
  - `PCM_MIC_Control` (`PA9`) → **LOW (0)** (Selects I2S1 PCM5102 DAC output)
  - `TALKOVER_EN`=0, `MIC_EN`=0, `MIC_AUX_EN`=0
  - `AC_Left_EN` (`PE13`) → **HIGH (1)**
  - `AC_Right_EN` (`PE14`) → **HIGH (1)**
  - `BC_EN`=0, `INSERT_EP_EN`=0, `FF_EN`=0
  - **PGA Attenuator**: Gain calculated via `DbToPGA2311(db_level)`.

---

### Feature 5: Bone Conduction (BC with Contralateral Masking)
* **Goal**: Routes stimulus vibrations to Radioear B71/B81 transducer; routes Narrowband Noise masking to contralateral insert earphone.
* **GPIO States**:
  - `PCM_MIC_Control` (`PA9`) → **LOW (0)**
  - `BC_EN` (`PE7`) → **HIGH (1)** (Enables Bone Conductor power driver)
  - `INSERT_EP_EN` (`PE12`) → **HIGH (1)** (Delivers masking noise into opposite ear canal)
  - `AC_Left_EN`=0, `AC_Right_EN`=0, `FF_EN`=0

```c
void Feature_Enable_BoneConduction(uint8_t enable_insert_masking)
{
    PGA2311_Mute();

    HAL_GPIO_WritePin(GPIOA, PCM_MIC_Control_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, AC_Left_EN_Pin | AC_Right_EN_Pin | FF_EN_Pin, GPIO_PIN_RESET);

    /* Turn ON Bone Conductor */
    HAL_GPIO_WritePin(GPIOE, BC_EN_Pin, GPIO_PIN_SET);

    /* Enable Insert Earphone for contralateral masking noise */
    HAL_GPIO_WritePin(GPIOE, INSERT_EP_EN_Pin, enable_insert_masking ? GPIO_PIN_SET : GPIO_PIN_RESET);

    PGA2311_Unmute();
}
```

---

### Feature 6: Free Field Testing (FF)
* **Goal**: Directs audio to external sound room amplifier and calibrated speakers.
* **GPIO States**:
  - `PCM_MIC_Control` (`PA9`) → **LOW (0)**
  - `FF_EN` (`PE9`) → **HIGH (1)**
  - `AC_Left_EN`=0, `AC_Right_EN`=0, `BC_EN`=0, `INSERT_EP_EN`=0

---

## 4. PGA2311 Attenuation & Click-Free Switching Discipline

The PGA2311 offers 0.5 dB step resolution (0..255). Step 0 = Mute, Step 1 = -95.5 dB, Step 192 = 0 dB (Unity Gain), Step 255 = +31.5 dB.

### Safe Pop/Click Elimination Sequence
Always execute this exact 5-step sequence when changing transducers, modes, or stepping gain:

```
[1. Assert PGA_MUTE_1 = LOW] 
          │
[2. Settle Delay (2 to 5 ms)]
          │
[3. Toggle Relay GPIOs (AC_EN, BC_EN, etc.)]
          │
[4. Shift 16-bit Volume to PGA2311 via SPI]
          │
[5. Deassert PGA_MUTE_1 = HIGH (Unmute)]
```

---

## 5. Patient Response Switch & Power Monitoring

### Patient Handswitch (`PB3` / `EXTI3`)
- **Pin**: `PB3` configured as input with `GPIO_PULLUP`.
- **Logic**: Active LOW. Pressing the button pulls `PB3` to GND.
- **Handling**: Configured with `EXTI3_IRQHandler`. Fast software debounce window (> 50 ms). Transmits response packet `0xAA 0x50 0x01 0x55` immediately to WebUI and Proculus display.

### Battery & Mains Voltage Detection
- **`BAT_READ` (`PA0` / `ADC1_IN0`)**: Measures battery pack voltage through a resistor divider. Polled periodically, averaged, and passed through a 1% slew rate limiter in `battery.c`.
- **`POWER_DETECTION` (`PA1`)**: Digital input. HIGH indicates external 12V/15V DC charger is plugged in.

---

## 6. Communication Architecture

| Interface | Baud Rate | Frame Protocol | Primary Use Case |
| :--- | :--- | :--- | :--- |
| **USART2 (`PA2`/`PA3`)** | 115200 8N1 | Proculus DGUS / UnicView AD (`0x5A 0xA5 ...`) | **Proculus 7-inch Intelligent Display** |
| **USART3 (`PD8`/`PC5`)** | 115200 8N1 | Binary/ASCII Protocol (`0xAA ... 0x55`) | **CH340G USB Serial** (WebUI control / future Thermal Printer) |
| **USB OTG FS** | 12 Mbps | USB CDC Virtual COM Port | Direct PC USB connection |
