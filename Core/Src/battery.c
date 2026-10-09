/**
  ******************************************************************************
  * @file           : battery.c
  * @brief          : Battery Monitoring, DC Power Detection, and ADC Mapping
  *                   Ported from Audiometer AX30 V1.2 model_battery & BSP
  ******************************************************************************
  */

#include "battery.h"
#include "main.h"

/* ==========================================================================
 * Debug / Live Expressions Variables
 * ========================================================================== */
volatile uint32_t debug_battery_adc = 0U;
volatile uint32_t debug_battery_percent = 0U;
volatile uint32_t debug_battery_scenario = 0U;
volatile uint32_t debug_battery_mv = 0U;
volatile uint32_t debug_power_dc = 0U;

/* ==========================================================================
 * Private Model State
 * ========================================================================== */
static Battery_Snapshot_t s_snap;
static uint16_t s_idle_avg;
static uint16_t s_load_avg;
static uint8_t  s_have_idle;
static uint8_t  s_have_load;
static uint16_t s_rest_adc;
static uint8_t  s_have_rest;
static uint16_t s_dc_offset;
static uint8_t  s_prev_dc;
static uint16_t s_filt_adc;
static uint8_t  s_filt_seeded;
static uint8_t  s_held_pct;
static uint8_t  s_pct_seeded;
static uint8_t  s_held_bars;
static uint8_t  s_bars_seeded;
static uint8_t  s_had_pack;
static uint8_t  s_have_good;

#define BATTERY_IIR_OLD   7U
#define BATTERY_IIR_DIV   8U
#define BATTERY_BAR_LATCH 3U

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/**
  * @brief  Maps raw battery percentage (0..100) to examiner UI percentage (0..100).
  *         Direct linear mapping as implemented in audiometer_basic.
  */
uint8_t Model_Battery_UiFromRaw(uint8_t raw_pct)
{
  /* Direct 0-100% mapping from audiometer_basic */
  return raw_pct;
}

/**
  * @brief  Checks whether boot hold is required due to low battery (< 16%).
  */
uint8_t Model_Battery_NeedBootHold(const Battery_Snapshot_t *s)
{
  if (s == 0 || s->adc_fail != 0U || s->pack_present == 0U)
  {
    return 0U;
  }
  return (s->instant_percent < BATTERY_BOOT_MIN_PCT) ? 1U : 0U;
}

/**
  * @brief  Converts UI percentage to 0..4 battery bars.
  */
uint8_t Model_Battery_BarsFromPercent(uint32_t ui_pct)
{
  if (ui_pct == 0U)
  {
    return 0U;
  }
  if (ui_pct <= 30U)
  {
    return 1U;
  }
  if (ui_pct <= 60U)
  {
    return 2U;
  }
  if (ui_pct <= 90U)
  {
    return 3U;
  }
  return 4U;
}

/**
  * @brief  Maps raw 12-bit ADC count (2482..3475) to raw percentage (0..100).
  */
uint8_t Model_Battery_RawPctFromAvg(uint32_t adc_avg)
{
  uint32_t adc = adc_avg;

  if (adc < (uint32_t)BATTERY_ADC_EMPTY)
  {
    adc = (uint32_t)BATTERY_ADC_EMPTY;
  }
  if (adc > (uint32_t)BATTERY_ADC_FULL)
  {
    adc = (uint32_t)BATTERY_ADC_FULL;
  }
  return (uint8_t)(((adc - (uint32_t)BATTERY_ADC_EMPTY) * 100U) /
                   (uint32_t)BATTERY_ADC_SPAN);
}

/**
  * @brief  Estimates battery pack voltage in millivolts (Vadc * 3/2 resistor divider).
  */
static uint16_t PackMvEstimate(uint32_t adc_avg)
{
  /* 12-bit ADC, 3300 mV Vref; resistor divider ratio 2:1 (inverse 3/2) */
  uint32_t vadc_mv = (adc_avg * 3300U) / 4095U;
  uint32_t pack_mv = (vadc_mv * 3U) / 2U;
  if (pack_mv > 65535U)
  {
    pack_mv = 65535U;
  }
  return (uint16_t)pack_mv;
}

/**
  * @brief  Classifies the current battery operating scenario.
  */
static Battery_Scenario_t Classify(const Battery_Snapshot_t *s)
{
  if (s->adc_fail != 0U)
  {
    return BAT_SCEN_ADC_FAIL;
  }
  if (s->pack_present == 0U)
  {
    return (s->dc_plugged != 0U) ? BAT_SCEN_DC_ONLY : BAT_SCEN_NO_PACK;
  }
  if (s->dc_plugged != 0U)
  {
    return BAT_SCEN_CHARGING;
  }
  if (s->instant_percent <= BATTERY_CRIT_OFF_PCT)
  {
    return BAT_SCEN_CRIT_OFF;
  }
  if (s->instant_percent <= BATTERY_LOW_WARN_PCT)
  {
    return BAT_SCEN_LOW_WARN;
  }
  if (s->percent >= 100U)
  {
    return BAT_SCEN_FULL;
  }
  return BAT_SCEN_DISCHARGING;
}

/**
  * @brief  Calculates battery health estimate from load vs idle voltage sag.
  */
static void UpdateHealth(uint16_t adc_avg, uint8_t presenting, uint8_t pack_ok)
{
  int32_t sag;
  uint32_t sag_pct;

  if (pack_ok == 0U)
  {
    return;
  }

  if (presenting != 0U)
  {
    s_load_avg = adc_avg;
    s_have_load = 1U;
  }
  else
  {
    s_idle_avg = adc_avg;
    s_have_idle = 1U;
  }

  if (s_have_idle == 0U || s_have_load == 0U)
  {
    s_snap.health_est = 100U;
    return;
  }

  sag = (int32_t)s_idle_avg - (int32_t)s_load_avg;
  if (sag < 0)
  {
    sag = 0;
  }
  if (s_idle_avg == 0U)
  {
    s_snap.health_est = 100U;
    return;
  }
  sag_pct = ((uint32_t)sag * 100U) / (uint32_t)s_idle_avg;
  if (sag_pct > 100U)
  {
    sag_pct = 100U;
  }
  s_snap.health_est = (uint8_t)(100U - sag_pct);
}

static void UnseedHeld(void)
{
  s_filt_seeded = 0U;
  s_pct_seeded = 0U;
  s_bars_seeded = 0U;
}

/**
  * @brief  Applies 7/8 IIR low-pass filtering to the rest ADC reading.
  */
static uint32_t FilterRestAdc(uint32_t use_adc, uint8_t presenting)
{
  uint32_t filt;

  if (presenting != 0U && s_filt_seeded != 0U)
  {
    return (uint32_t)s_filt_adc;
  }
  if (s_filt_seeded == 0U)
  {
    s_filt_adc = (uint16_t)use_adc;
    s_filt_seeded = 1U;
    return use_adc;
  }
  filt = ((uint32_t)s_filt_adc * (uint32_t)BATTERY_IIR_OLD + use_adc) /
         (uint32_t)BATTERY_IIR_DIV;
  s_filt_adc = (uint16_t)filt;
  return (uint32_t)s_filt_adc;
}

/**
  * @brief  Applies 1% slew rate limiter so displayed percentage changes smoothly.
  */
static uint8_t HoldPercent(uint8_t instant, uint8_t dc)
{
  if (s_pct_seeded == 0U)
  {
    s_held_pct = instant;
    s_pct_seeded = 1U;
    return s_held_pct;
  }
  if (dc != 0U)
  {
    if (instant > s_held_pct)
    {
      s_held_pct = (uint8_t)(s_held_pct + 1U);
    }
  }
  else if (instant < s_held_pct)
  {
    s_held_pct = (uint8_t)(s_held_pct - 1U);
  }
  return s_held_pct;
}

/**
  * @brief  Latches battery bars with ±3% hysteresis around 30%, 60%, 90%.
  */
static uint8_t LatchBars(uint8_t held)
{
  uint8_t raw = Model_Battery_BarsFromPercent((uint32_t)held);

  if (s_bars_seeded == 0U)
  {
    s_held_bars = raw;
    s_bars_seeded = 1U;
    return s_held_bars;
  }
  if (raw == s_held_bars)
  {
    return s_held_bars;
  }
  if (raw > s_held_bars)
  {
    if (s_held_bars == 0U)
    {
      s_held_bars = 1U;
    }
    else if (s_held_bars == 1U && held >= (30U + (uint8_t)BATTERY_BAR_LATCH))
    {
      s_held_bars = 2U;
    }
    else if (s_held_bars == 2U && held >= (60U + (uint8_t)BATTERY_BAR_LATCH))
    {
      s_held_bars = 3U;
    }
    else if (s_held_bars == 3U && held >= (90U + (uint8_t)BATTERY_BAR_LATCH))
    {
      s_held_bars = 4U;
    }
  }
  else
  {
    if (s_held_bars == 4U && held <= (90U - (uint8_t)BATTERY_BAR_LATCH))
    {
      s_held_bars = 3U;
    }
    else if (s_held_bars == 3U && held <= (60U - (uint8_t)BATTERY_BAR_LATCH))
    {
      s_held_bars = 2U;
    }
    else if (s_held_bars == 2U && held <= (30U - (uint8_t)BATTERY_BAR_LATCH))
    {
      s_held_bars = 1U;
    }
    else if (s_held_bars == 1U && held == 0U)
    {
      s_held_bars = 0U;
    }
  }
  return s_held_bars;
}

static void CommitHeld(uint8_t instant, uint8_t dc)
{
  uint8_t held = HoldPercent(instant, dc);

  s_snap.instant_percent = instant;
  s_snap.percent = held;
  s_snap.capacity_est = held;
  s_snap.bars = LatchBars(held);
  debug_battery_percent = (uint32_t)held;
}

static void CommitFailNoGood(uint16_t adc_raw, uint8_t dc)
{
  UnseedHeld();
  s_snap.adc_raw = adc_raw;
  s_snap.adc_avg = BATTERY_ADC_FAIL;
  s_snap.dc_plugged = (dc != 0U) ? 1U : 0U;
  s_snap.pack_present = 0U;
  s_snap.pack_removed = 0U;
  s_snap.adc_fail = 1U;
  s_snap.raw_pct = 100U;
  s_snap.instant_percent = 100U;
  s_snap.percent = 100U;
  s_snap.bars = 4U;
  s_snap.capacity_est = 100U;
  s_snap.pack_mv_est = 0U;
  s_snap.scenario = BAT_SCEN_ADC_FAIL;
  debug_battery_adc = (uint32_t)adc_raw;
  debug_battery_percent = 100U;
  debug_battery_scenario = (uint32_t)BAT_SCEN_ADC_FAIL;
  debug_battery_mv = 0U;
  debug_power_dc = (uint32_t)s_snap.dc_plugged;
  s_prev_dc = s_snap.dc_plugged;
}

/**
  * @brief  Resets the battery model state machine and snapshot to defaults.
  */
void Model_Battery_Reset(void)
{
  s_idle_avg = 0U;
  s_load_avg = 0U;
  s_have_idle = 0U;
  s_have_load = 0U;
  s_rest_adc = 0U;
  s_have_rest = 0U;
  s_dc_offset = BATTERY_ADC_DC_OFFSET_DEFAULT;
  s_prev_dc = 0xFFU;
  UnseedHeld();
  s_filt_adc = 0U;
  s_held_pct = 100U;
  s_held_bars = 4U;
  s_had_pack = 0U;
  s_have_good = 0U;
  s_snap.adc_raw = BATTERY_ADC_FAIL;
  s_snap.adc_avg = BATTERY_ADC_FAIL;
  s_snap.dc_plugged = 0U;
  s_snap.pack_present = 0U;
  s_snap.pack_removed = 0U;
  s_snap.adc_fail = 1U;
  s_snap.raw_pct = 100U;
  s_snap.instant_percent = 100U;
  s_snap.percent = 100U;
  s_snap.bars = 4U;
  s_snap.capacity_est = 100U;
  s_snap.health_est = 100U;
  s_snap.pack_mv_est = 0U;
  s_snap.scenario = BAT_SCEN_ADC_FAIL;
  debug_battery_adc = 0U;
  debug_battery_percent = 100U;
  debug_battery_scenario = (uint32_t)BAT_SCEN_ADC_FAIL;
  debug_battery_mv = 0U;
  debug_power_dc = 0U;
}

/**
  * @brief  Processes sample pair, filters, classifies, and updates snapshot.
  */
void Model_Battery_ApplySamples(uint16_t adc0, uint16_t adc1,
                                uint8_t dc_plugged, uint8_t presenting)
{
  uint32_t avg;
  uint8_t dc = (dc_plugged != 0U) ? 1U : 0U;
  uint8_t fail0 = (adc0 == BATTERY_ADC_FAIL) ? 1U : 0U;
  uint8_t fail1 = (adc1 == BATTERY_ADC_FAIL) ? 1U : 0U;

  s_snap.pack_removed = 0U;
  s_snap.dc_plugged = dc;
  debug_power_dc = dc;

  if (fail0 != 0U && fail1 != 0U)
  {
    if (s_have_good == 0U)
    {
      CommitFailNoGood(adc1, dc);
      return;
    }
    s_snap.adc_raw = BATTERY_ADC_FAIL;
    s_snap.adc_fail = 0U;
    s_snap.scenario = Classify(&s_snap);
    debug_battery_scenario = (uint32_t)s_snap.scenario;
    s_prev_dc = dc;
    return;
  }

  if (fail0 != 0U)
  {
    avg = (uint32_t)adc1;
    s_snap.adc_raw = adc1;
  }
  else if (fail1 != 0U)
  {
    avg = (uint32_t)adc0;
    s_snap.adc_raw = adc0;
  }
  else
  {
    avg = ((uint32_t)adc0 + (uint32_t)adc1) / 2U;
    s_snap.adc_raw = adc1;
  }

  s_snap.adc_avg = (uint16_t)avg;
  s_snap.adc_fail = 0U;
  s_have_good = 1U;

  /* Check load sag below no-pack during audio presentation */
  if (avg < (uint32_t)BATTERY_ADC_NO_PACK &&
      presenting != 0U &&
      s_had_pack != 0U)
  {
    if (avg > (uint32_t)BATTERY_ADC_FLOAT_MAX)
    {
      UpdateHealth((uint16_t)avg, 1U, 1U);
    }
    s_snap.pack_present = 1U;
    s_snap.pack_removed = 0U;
    s_prev_dc = dc;
    debug_battery_adc = s_filt_seeded ? (uint32_t)s_filt_adc : avg;
    s_snap.scenario = Classify(&s_snap);
    debug_battery_scenario = (uint32_t)s_snap.scenario;
    return;
  }

  if (avg < (uint32_t)BATTERY_ADC_NO_PACK)
  {
    s_snap.pack_mv_est = PackMvEstimate(avg);
    debug_battery_mv = (uint32_t)s_snap.pack_mv_est;
    s_prev_dc = dc;
    if (avg > (uint32_t)BATTERY_ADC_FLOAT_MAX)
    {
      /* Dying pack sags under 2200: still a pack at UI 0% */
      s_had_pack = 1U;
      s_snap.pack_present = 1U;
      s_snap.raw_pct = 0U;
      UnseedHeld();
      debug_battery_adc = avg;
      CommitHeld(0U, dc);
      s_snap.scenario = Classify(&s_snap);
      debug_battery_scenario = (uint32_t)s_snap.scenario;
      return;
    }

    if (s_had_pack != 0U && dc == 0U)
    {
      s_snap.pack_present = 0U;
      s_snap.pack_removed = 1U;
      s_snap.raw_pct = 0U;
      UnseedHeld();
      debug_battery_adc = avg;
      CommitHeld(0U, dc);
      s_snap.scenario = Classify(&s_snap);
      debug_battery_scenario = (uint32_t)s_snap.scenario;
      return;
    }

    UnseedHeld();
    s_snap.pack_present = 0U;
    s_snap.raw_pct = 100U;
    s_snap.instant_percent = 100U;
    s_snap.percent = 100U;
    s_snap.bars = 4U;
    s_snap.capacity_est = 100U;
    s_snap.scenario = Classify(&s_snap);
    debug_battery_adc = avg;
    debug_battery_percent = 100U;
    debug_battery_scenario = (uint32_t)s_snap.scenario;
    return;
  }

  s_snap.pack_present = 1U;
  s_had_pack = 1U;
  {
    uint32_t use_adc = avg;

    if (dc == 0U)
    {
      if (presenting == 0U)
      {
        s_rest_adc = (uint16_t)avg;
        s_have_rest = 1U;
      }
    }
    else
    {
      if (s_prev_dc == 0U && s_have_rest != 0U)
      {
        if (avg > (uint32_t)s_rest_adc)
        {
          uint32_t off = avg - (uint32_t)s_rest_adc;
          if (off > (uint32_t)BATTERY_ADC_DC_OFFSET_MAX)
          {
            off = (uint32_t)BATTERY_ADC_DC_OFFSET_MAX;
          }
          s_dc_offset = (uint16_t)off;
        }
        else
        {
          s_dc_offset = 0U;
        }
      }
      if (use_adc > (uint32_t)s_dc_offset)
      {
        use_adc -= (uint32_t)s_dc_offset;
      }
      else
      {
        use_adc = (uint32_t)BATTERY_ADC_EMPTY;
      }
    }
    s_prev_dc = dc;

    use_adc = FilterRestAdc(use_adc, presenting);
    debug_battery_adc = use_adc;
    s_snap.raw_pct = Model_Battery_RawPctFromAvg(use_adc);
  }
  {
    uint8_t instant = Model_Battery_UiFromRaw(s_snap.raw_pct);
    instant = (uint8_t)clamp_u32((uint32_t)instant, 0U, 100U);
    CommitHeld(instant, dc);
  }
  s_snap.pack_mv_est = PackMvEstimate(avg);
  debug_battery_mv = (uint32_t)s_snap.pack_mv_est;
  UpdateHealth((uint16_t)avg, presenting, 1U);
  s_snap.scenario = Classify(&s_snap);
  debug_battery_scenario = (uint32_t)s_snap.scenario;
}

/**
  * @brief  Returns pointer to the read-only current battery snapshot.
  */
const Battery_Snapshot_t *Model_Battery_Get(void)
{
  return &s_snap;
}

/* ==========================================================================
 * BSP Hardware Initialization & Reading Implementation
 * ========================================================================== */

/**
  * @brief  Initializes GPIO pins for Battery Read (PA0) and Power Detection (PA1).
  */
void BSP_Battery_Init(void)
{
  /* 1. Clocks */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_ADC1_CLK_ENABLE();

  /* 2. Configure BAT_READ (PA0) as Analog */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = BAT_READ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BAT_READ_GPIO_Port, &GPIO_InitStruct);

  /* 3. Configure POWER_DETECTION (PA1) as Input with Pull-Up */
  GPIO_InitStruct.Pin = POWER_DETECTION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(POWER_DETECTION_GPIO_Port, &GPIO_InitStruct);

  /* 4. Configure ADC1 for 12-bit single conversion on Channel 0 (PA0) */
  ADC->CCR &= ~ADC_CCR_ADCPRE;     /* Prescaler /2 */
  ADC1->CR1 = 0;                   /* 12-bit resolution, scan mode disabled */
  ADC1->CR2 = ADC_CR2_ADON;        /* Power on ADC1 */
  ADC1->SMPR2 = (7U << 0);         /* 480 cycles sampling time on Channel 0 */
  ADC1->SQR1 = 0;                  /* 1 conversion */
  ADC1->SQR3 = 0;                  /* Rank 1 = Channel 0 */

  /* Reset software battery model */
  Model_Battery_Reset();
}

/**
  * @brief  Performs a single 12-bit conversion on ADC1_IN0 (PA0).
  * @retval Raw ADC value (0..4095) or BATTERY_ADC_FAIL (0xFFFF) on timeout.
  */
uint16_t BSP_Battery_ReadRawADC(void)
{
  /* Ensure ADC1 is powered */
  if (!(ADC1->CR2 & ADC_CR2_ADON))
  {
    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile int i = 0; i < 200; i++);
  }

  /* Clear status flags */
  ADC1->SR = 0;

  /* Start conversion */
  ADC1->CR2 |= ADC_CR2_SWSTART;

  /* Poll for End of Conversion (EOC) with timeout */
  uint32_t timeout = 50000;
  while (!(ADC1->SR & ADC_SR_EOC) && --timeout);

  if (timeout == 0)
  {
    return BATTERY_ADC_FAIL;
  }

  uint16_t val = (uint16_t)ADC1->DR;
  return val;
}

/**
  * @brief  Checks whether external DC power is plugged in.
  * @retval true if DC power detected (PA1 is LOW), false otherwise.
  */
bool BSP_DC_PowerDetected(void)
{
  /* Active-low: GPIO_PIN_RESET means external DC supply plugged in */
  return (HAL_GPIO_ReadPin(POWER_DETECTION_GPIO_Port, POWER_DETECTION_Pin) == GPIO_PIN_RESET);
}

/**
  * @brief  Periodic poll helper: takes 2 settled ADC samples, detects DC status,
  *         and feeds them into the battery state machine.
  */
void Battery_Poll(uint32_t current_ms, uint8_t is_presenting)
{
  (void)current_ms;

  uint16_t a = BSP_Battery_ReadRawADC();
  for (volatile int i = 0; i < 100; i++); /* Small settling gap */
  uint16_t b = BSP_Battery_ReadRawADC();
  uint8_t dc = BSP_DC_PowerDetected() ? 1U : 0U;

  Model_Battery_ApplySamples(a, b, dc, is_presenting);
}

/**
  * @brief  Converts battery percentage (0..100) to display bars (0..4).
  *         Exact mapping from audiometer_basic.
  */
uint32_t Get_Battery_State_From_Percentage(uint32_t pct)
{
  if (pct == 0) return 0;   /* 0% -> no bar */
  if (pct <= 30) return 1;  /* 0-30% -> 1 bar */
  if (pct <= 60) return 2;  /* 30-60% -> 2 bars */
  if (pct <= 90) return 3;  /* 60-90% -> 3 bars */
  return 4;                 /* 90-100% -> full (4 bars) */
}

/**
  * @brief  Reads ADC1 on PA0, maps calibrated ADC counts to 0..100% percentage.
  *         Exact implementation ported from audiometer_basic/Core/Src/main.c.
  *         Raw ADC value 2482 corresponds to 0% (empty battery threshold)
  *         Raw ADC value 3475 corresponds to 100% (fully charged battery threshold)
  *         If ADC reading is low (< 2200), no battery is inserted (returns 100%).
  */
uint32_t Read_Battery_Percentage(void)
{
  uint16_t adc_val = BSP_Battery_ReadRawADC();
  debug_battery_adc = adc_val; /* Save raw ADC code for Live Expressions */

  /* If battery ADC reading is low (< 2200), no battery is inserted
   * and the device is operating on external power. Skip low-battery sleep. */
  if (adc_val < 2200)
  {
    debug_battery_percent = 100;
    return 100;
  }

  /* Calibrated Battery Voltage Sensing and Mapping:
   * Raw ADC value 2482 corresponds to 0% (empty battery threshold)
   * Raw ADC value 3475 corresponds to 100% (fully charged battery threshold) */
  if (adc_val < 2482) adc_val = 2482;
  if (adc_val > 3475) adc_val = 3475;

  uint32_t percent = ((adc_val - 2482) * 100) / (3475 - 2482);
  debug_battery_percent = percent; /* Save calculated percentage for Live Expressions */
  return percent;
}
