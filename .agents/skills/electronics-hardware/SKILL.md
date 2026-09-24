---
name: electronics-hardware
description: >-
  Hardware design verification, schematic analysis, audio transducer impedance matching,
  op-amp circuit filtering, PGA gain scaling, and power rail integrity.
  Use when analyzing schematics, audio hardware circuitry, transducer routing, or board diagnostics.
---

# Electronics & Hardware Engineering Skill

## Overview
Guidelines and analysis procedures for clinical audiometer hardware, analog audio front-ends, digital control logic, and power conditioning on the AX60 board.

---

## 1. Transducer Output Paths & Electrical Characteristics
The device routes signals to distinct acoustic transducers with specific electrical impedances:

| Transducer Type | Typical Impedance | Control Signal | Key Design Consideration |
| :--- | :--- | :--- | :--- |
| **Air Conduction (AC) Left** | 10 $\Omega$ / 300 $\Omega$ (Telephonics TDH-39 / DD45) | `AC_Left_EN` (PE13) | Low THD, high power drive, click-free switching |
| **Air Conduction (AC) Right** | 10 $\Omega$ / 300 $\Omega$ | `AC_Right_EN` (PE14) | Match left channel frequency response within $\pm 0.5\text{ dB}$ |
| **Bone Conduction (BC)** | 10 $\Omega$ (Radioear B71 / B81) | `BC_EN` (PE7), `BC_L_R_EN` (PE10) | High mechanical power required at low frequencies (250Hz) |
| **Insert Earphones** | 50 $\Omega$ / 300 $\Omega$ (Etymotic ER-3A/5A) | `INSERT_EP_EN` (PE12) | High acoustic isolation, electrostatic discharge (ESD) protection |
| **Free Field (FF)** | Line level into external power amp | `FF_EN` (PE9) | Low-impedance balanced or single-ended line out |
| **White Noise / Masking** | Narrowband / Speech noise | `WN_EN` (PE8) | Crest factor management to avoid clipping |

---

## 2. Signal Integrity & Analog Path Hygiene
1. **PGA Zero-Crossing Enable (`ZCEN` - PB14)**:
   - When enabled, gain level changes only occur when the audio waveform crosses zero volts. This minimizes audible clicks during hearing test attenuator adjustments ($1\text{ dB}$ or $5\text{ dB}$ steps).
2. **Analog Ground & Digital Ground (AGND / DGND)**:
   - Analog components (PGA, PCM1808 ADC, Op-Amps) must connect to an isolated AGND plane tied to DGND at a single star point near the power supply.
3. **Op-Amp Output Buffer (`OPA_EN` - PB2)**:
   - High-current op-amp stages driving low-impedance transducers (e.g., $10\,\Omega$ B71 or TDH-39) require adequate heatsinking and short-circuit current limiting.

---

## 3. Power Supply & Battery Management
- **Battery Measurement**:
  - `BAT_READ` on `PA0` (ADC1_IN0) via resistor divider.
  - Scale factor check: $V_{\text{bat}} = V_{\text{adc}} \times \frac{R_1 + R_2}{R_2}$.
- **Mains / Power Detection**:
  - `POWER_DETECTION` on `PA1`: Monitor charging and external power presence.
