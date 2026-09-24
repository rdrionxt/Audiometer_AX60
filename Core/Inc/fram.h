/* Core/Inc/fram.h — FM25CL64B-G FRAM Driver for AX60
 *
 * FM25CL64B-G: 64Kbit (8KB) SPI FRAM
 * SPI3: PC10=SCK, PC11=MISO, PB5=MOSI
 * CS  : PC13 (FRAM_CS_Pin on GPIOC)
 */
#ifndef __FRAM_H__
#define __FRAM_H__

#include "main.h"

/* ── FRAM Commands ─────────────────────────────────────────────── */
#define FRAM_CMD_WREN   0x06U   /* Write Enable                    */
#define FRAM_CMD_WRDI   0x04U   /* Write Disable                   */
#define FRAM_CMD_RDSR   0x05U   /* Read Status Register            */
#define FRAM_CMD_WRSR   0x01U   /* Write Status Register           */
#define FRAM_CMD_READ   0x03U   /* Read Memory                     */
#define FRAM_CMD_WRITE  0x02U   /* Write Memory                    */
#define FRAM_CMD_RDID   0x9FU   /* Read Device ID                  */
#define FRAM_CMD_SLEEP  0xB9U   /* Enter Sleep Mode                */

/* ── FRAM Size ─────────────────────────────────────────────────── */
#define FRAM_SIZE_BYTES 8192U   /* 64Kbit = 8192 bytes             */

/* ── Return codes ──────────────────────────────────────────────── */
#define FRAM_OK         0U
#define FRAM_ERROR      1U

/* ── Public API ────────────────────────────────────────────────── */
uint8_t FRAM_Write(uint16_t addr, const uint8_t *data, uint16_t len);
uint8_t FRAM_Read (uint16_t addr, uint8_t *data, uint16_t len);
uint8_t FRAM_ReadID(uint8_t *id_buf, uint8_t len);
uint8_t FRAM_Test (void);  /* Write/Read verify test, returns FRAM_OK or FRAM_ERROR */

#endif /* __FRAM_H__ */
