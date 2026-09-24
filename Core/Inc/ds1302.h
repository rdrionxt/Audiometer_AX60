/**
  ******************************************************************************
  * @file           : ds1302.h
  * @brief          : DS1302 Real-Time Clock Driver Header for STM32F4 (AX60)
  ******************************************************************************
  */

#ifndef __DS1302_H
#define __DS1302_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "main.h"

/* DS1302 Pin Definitions for Audiometer AX60 */
#define DS1302_CE_PORT    RTC_EN_GPIO_Port
#define DS1302_CE_PIN     RTC_EN_Pin    /* PB1 - RTC_EN / CE */

#define DS1302_SCLK_PORT  RTC_CLK_GPIO_Port
#define DS1302_SCLK_PIN   RTC_CLK_Pin   /* PB6 - RTC_CLK / SCLK */

#define DS1302_IO_PORT    RTC_IO_GPIO_Port
#define DS1302_IO_PIN     RTC_IO_Pin    /* PB9 - RTC_IO / DATA */

/* DS1302 Register Addresses (Write commands) */
#define DS1302_REG_SECONDS      0x80
#define DS1302_REG_MINUTES      0x82
#define DS1302_REG_HOURS        0x84
#define DS1302_REG_DATE         0x86
#define DS1302_REG_MONTH        0x88
#define DS1302_REG_DAY          0x8A
#define DS1302_REG_YEAR         0x8C
#define DS1302_REG_WP           0x8E
#define DS1302_REG_TRICKLE      0x90
#define DS1302_REG_BURST_CLOCK  0xBE
#define DS1302_REG_BURST_RAM    0xFE

/* Date & Time Structure */
typedef struct {
    uint16_t year;    /* 2000 - 2099 */
    uint8_t  month;   /* 1 - 12 */
    uint8_t  day;     /* 1 - 31 */
    uint8_t  hour;    /* 0 - 23 (24-hour format) */
    uint8_t  min;     /* 0 - 59 */
    uint8_t  sec;     /* 0 - 59 */
    uint8_t  weekday; /* 1 - 7 */
} DS1302_DateTime_t;

/* Public Function Prototypes */
uint8_t DS1302_Init(void);
uint8_t DS1302_IsOscillatorRunning(void);
uint8_t DS1302_GetTime(DS1302_DateTime_t *dt);
uint8_t DS1302_SetTime(const DS1302_DateTime_t *dt);

void    DS1302_WriteReg(uint8_t reg, uint8_t val);
uint8_t DS1302_ReadReg(uint8_t reg);

#ifdef __cplusplus
}
#endif

#endif /* __DS1302_H */
