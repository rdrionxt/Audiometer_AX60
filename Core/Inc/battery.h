/**
  ******************************************************************************
  * @file           : battery.h
  * @brief          : Header for Battery Monitoring, Power Detection, and ADC Mapping
  *                   Ported from Audiometer AX30 V1.2 model_battery & BSP
  ******************************************************************************
  */

#ifndef __BATTERY_H
#define __BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * Battery ADC & UI Remapping Thresholds (from Audiometer AX30 V1.2)
 * ========================================================================== */
#define BATTERY_ADC_EMPTY             2482U  /* Empty battery ADC count (12-bit, ~3.0V at divider) */
#define BATTERY_ADC_FULL              3475U  /* Full battery ADC count (12-bit, ~4.2V at divider) */
#define BATTERY_ADC_SPAN              993U   /* 3475 - 2482 */
#define BATTERY_ADC_NO_PACK           2200U  /* Below 2200 indicates no pack installed */
#define BATTERY_ADC_FLOAT_MAX         500U   /* Open / floating ADC count */
#define BATTERY_ADC_FAIL              0xFFFFU/* ADC read timeout / error */

/* Charger lifts the sense node (~10 UI%). Subtract while DC is in. */
#define BATTERY_ADC_DC_OFFSET_DEFAULT 80U
#define BATTERY_ADC_DC_OFFSET_MAX     250U

/* Hidden raw band mapped to examiner UI 0%. Not a boot gate. */
#define BATTERY_UI_HIDDEN_RAW         20U

/* Examiner UI% required to run full boot (must be above 15%). */
#define BATTERY_BOOT_MIN_PCT          16U
#define BATTERY_BOOT_SW7_PCT          20U
#define BATTERY_CRIT_OFF_PCT          5U
#define BATTERY_WAKE_PCT              25U
#define BATTERY_LOW_WARN_PCT          10U
#define BATTERY_LOW_CLEAR_PCT         15U
#define BATTERY_LOW_REPEAT_MS         60000U

/* Battery operating scenario classification */
typedef enum {
  BAT_SCEN_ADC_FAIL = 0,
  BAT_SCEN_NO_PACK,
  BAT_SCEN_DC_ONLY,
  BAT_SCEN_CHARGING,
  BAT_SCEN_CRIT_OFF,
  BAT_SCEN_LOW_WARN,
  BAT_SCEN_FULL,
  BAT_SCEN_DISCHARGING
} Battery_Scenario_t;

/* RAM Battery Snapshot */
typedef struct {
  uint16_t adc_raw;           /* Last raw conversion (or FAIL) */
  uint16_t adc_avg;           /* Mean of good tick samples; last-good if both FAIL */
  uint8_t  dc_plugged;        /* 1 = DC / Adapter connected, 0 = on battery */
  uint8_t  pack_present;      /* 1 = Battery pack installed */
  uint8_t  pack_removed;      /* Had a pack, now open/no-pack, DC out */
  uint8_t  adc_fail;          /* Both samples FAIL and no last-good yet */
  uint8_t  raw_pct;           /* 0–100 from filtered ADC 2482/3475, internal only */
  uint8_t  instant_percent;   /* Policy UI% from IIR (crit, boot, SW7, wake) */
  uint8_t  percent;           /* Examiner UI%: held, 1%/sample slew rate limited */
  uint8_t  bars;              /* 0–4 from held %, latched at 30/60/90 */
  uint8_t  capacity_est;      /* Alias of percent */
  uint8_t  health_est;        /* 100 unknown; sag-derived, clamp >= 0 */
  uint16_t pack_mv_est;       /* Vadc * 3/2 estimate (mV) */
  Battery_Scenario_t scenario;
} Battery_Snapshot_t;

/* Model battery core APIs */
void Model_Battery_Reset(void);
void Model_Battery_ApplySamples(uint16_t adc0, uint16_t adc1,
                                uint8_t dc_plugged, uint8_t presenting);
const Battery_Snapshot_t *Model_Battery_Get(void);
uint8_t Model_Battery_UiFromRaw(uint8_t raw_pct);
uint8_t Model_Battery_BarsFromPercent(uint32_t ui_pct);
uint8_t Model_Battery_RawPctFromAvg(uint32_t adc_avg);
uint8_t Model_Battery_NeedBootHold(const Battery_Snapshot_t *s);

/* BSP Hardware & Polling APIs */
void     BSP_Battery_Init(void);
uint16_t BSP_Battery_ReadRawADC(void);
bool     BSP_DC_PowerDetected(void);
void     Battery_Poll(uint32_t current_ms, uint8_t is_presenting);

/* Direct 0-100% Mapping APIs (from audiometer_basic) */
uint32_t Read_Battery_Percentage(void);
uint32_t Get_Battery_State_From_Percentage(uint32_t pct);

/* Debug / Live Expressions Variables */
extern volatile uint32_t debug_battery_adc;
extern volatile uint32_t debug_battery_percent;
extern volatile uint32_t debug_battery_scenario;
extern volatile uint32_t debug_battery_mv;
extern volatile uint32_t debug_power_dc;

#ifdef __cplusplus
}
#endif

#endif /* __BATTERY_H */
