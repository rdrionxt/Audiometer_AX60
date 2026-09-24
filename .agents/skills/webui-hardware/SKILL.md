---
name: webui-hardware
description: >-
  Development guidelines for hardware control interfaces, Web Serial API communication,
  real-time audiogram plotting, responsive clinical control panels, and state synchronization.
  Use when designing, enhancing, or debugging Web UI and browser-to-MCU hardware communication.
---

# Web UI & Hardware Control Interface Skill

## Overview
Standards and best practices for developing the browser-based control panel (`WebUI/index.html`) connecting to the Audiometer AX60 via the Web Serial API.

---

## 1. Web Serial Communication Pipeline
- **Connection Management**:
  - Request port: `navigator.serial.requestPort()` with vendor/product ID filtering.
  - Recommended default baud rate: 115200 (8N1), matching STM32 `USART2`/`USART3` or virtual COM port.
  - Implement continuous read loop with `TextDecoderStream` or binary buffer parser.
  - Handle unexpected disconnects (`port.ondisconnect`) gracefully with UI state recovery.

- **Command Protocol Protocol Format**:
  - Outgoing commands: Format as clear delimiter-separated packets:
    `$CMD,<PARAM1>,<PARAM2>*<CRC>\n`
  - Examples:
    - Tone Stimulus: `$STIM,1,1000,40,L*4F` (Enable stimulus, 1000Hz, 40dB HL, Left AC)
    - Channel Select: `$CHAN,AC_L*12`
    - Mute/Stop: `$STIM,0*00`

---

## 2. Real-Time Audiogram & Display Rendering
- **Audiogram Standards (ISO / ANSI)**:
  - Frequencies (Hz on logarithmic X-axis): 125, 250, 500, 750, 1000, 1500, 2000, 3000, 4000, 6000, 8000 Hz.
  - Hearing Level (dB HL on inverted Y-axis): $-10\text{ dB}$ at the top down to $120\text{ dB}$ at the bottom.
  - Symbols:
    - Red Circle `O`: Right Air Conduction (unmasked)
    - Blue Cross `X`: Left Air Conduction (unmasked)
    - Red Triangle `Δ` / Bracket `[`: Right AC masked / Right BC
    - Blue Square `□` / Bracket `]`: Left AC masked / Left BC
- **Responsive Updates**:
  - Update virtual LCD and active button states immediately upon receiving response packets from the MCU.
  - Animate patient response indicator dynamically when `pat_response_switch` triggers an event.

---

## 3. Clinical Safety & Fail-Safe UI Logic
1. **Auto-Disarm on Timeout**:
   - If no heartbeat is received from the STM32 within 1 second during an active stimulus, disarm stimulus in the UI and warn the operator.
2. **High-Intensity Warning**:
   - For intensities exceeding $85\text{ dB HL}$, require operator confirmation or display a distinct visual warning icon to prevent acoustic trauma.
