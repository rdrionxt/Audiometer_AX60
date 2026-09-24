#ifndef SD_CARD_H_
#define SD_CARD_H_

#include "stm32f4xx_hal.h"
#include "main.h"

extern uint8_t sd_initialized; /* 1 if SD_Init() succeeded */
extern volatile uint8_t sd_msc_busy; /* 1 while FatFs owns SPI (MSC must defer) */

uint8_t SD_Init(void);
void SD_SetMscBusy(uint8_t busy);
uint8_t SD_IsMscBusy(void);
void SD_PauseUsbMsc(void);
void SD_ResumeUsbMsc(void);
void SD_ReleaseBus(void);
uint8_t SD_ReadBlock(uint8_t *buf, uint32_t sector, uint32_t count);
uint8_t SD_WriteBlock(const uint8_t *buf, uint32_t sector, uint32_t count);
uint32_t SD_GetBlocks(void);
uint8_t SD_WaitReady(void);

#define SD_OK    0
#define SD_ERROR 1

#endif /* SD_CARD_H_ */
