/* Core/Src/fram.c — FM25CL64B-G FRAM Driver for AX60
 *
 * FM25CL64B-G: 64Kbit (8KB) SPI FRAM
 * SPI3 bus shared with SD card (different CS pins)
 *   SCK  : PC10
 *   MISO : PC11
 *   MOSI : PB5
 *   CS   : PC13 (FRAM_CS_Pin on GPIOC)
 */
#include "fram.h"
#include <string.h>

/* ── External SPI handle (defined in main.c) ───────────────────── */
extern SPI_HandleTypeDef hspi3;

/* ── CS macros ─────────────────────────────────────────────────── */
#define FRAM_CS_LOW()   HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET)
#define FRAM_CS_HIGH()  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET)

/* ── Internal: send/receive one byte ───────────────────────────── */
static uint8_t SPI_Byte(uint8_t tx)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi3, &tx, &rx, 1, 100);
    return rx;
}

/* ── FRAM_Write ────────────────────────────────────────────────── */
uint8_t FRAM_Write(uint16_t addr, const uint8_t *data, uint16_t len)
{
    if (addr + len > FRAM_SIZE_BYTES) return FRAM_ERROR;

    /* Send WREN first */
    FRAM_CS_LOW();
    SPI_Byte(FRAM_CMD_WREN);
    FRAM_CS_HIGH();

    HAL_Delay(1);

    /* Send WRITE command + 16-bit address + data */
    FRAM_CS_LOW();
    SPI_Byte(FRAM_CMD_WRITE);
    SPI_Byte((uint8_t)(addr >> 8));   /* High byte */
    SPI_Byte((uint8_t)(addr & 0xFF)); /* Low byte  */

    for (uint16_t i = 0; i < len; i++)
    {
        SPI_Byte(data[i]);
    }
    FRAM_CS_HIGH();

    return FRAM_OK;
}

/* ── FRAM_Read ─────────────────────────────────────────────────── */
uint8_t FRAM_Read(uint16_t addr, uint8_t *data, uint16_t len)
{
    if (addr + len > FRAM_SIZE_BYTES) return FRAM_ERROR;

    FRAM_CS_LOW();
    SPI_Byte(FRAM_CMD_READ);
    SPI_Byte((uint8_t)(addr >> 8));
    SPI_Byte((uint8_t)(addr & 0xFF));

    for (uint16_t i = 0; i < len; i++)
    {
        data[i] = SPI_Byte(0xFF);
    }
    FRAM_CS_HIGH();

    return FRAM_OK;
}

/* ── FRAM_ReadID ───────────────────────────────────────────────── */
/* FM25CL64B returns: 7F 7F 7F 7F 7F C2 21 00
 * Manufacturer: 0xC2 (Cypress), Product: 0x21, Density: 0x00       */
uint8_t FRAM_ReadID(uint8_t *id_buf, uint8_t len)
{
    FRAM_CS_LOW();
    SPI_Byte(FRAM_CMD_RDID);
    for (uint8_t i = 0; i < len; i++)
    {
        id_buf[i] = SPI_Byte(0xFF);
    }
    FRAM_CS_HIGH();
    return FRAM_OK;
}

/* ── FRAM_Test ─────────────────────────────────────────────────── */
/* Writes a test pattern to address 0x0000 and reads it back.
 * Returns FRAM_OK (0) if pass, FRAM_ERROR (1) if fail.             */
uint8_t FRAM_Test(void)
{
    const uint8_t write_data[8] = {0xDE, 0xAD, 0xBE, 0xEF,
                                   0x12, 0x34, 0x56, 0x78};
    uint8_t read_data[8] = {0};

    /* Write test pattern */
    if (FRAM_Write(0x0000, write_data, 8) != FRAM_OK) return FRAM_ERROR;

    HAL_Delay(1);

    /* Read back */
    if (FRAM_Read(0x0000, read_data, 8) != FRAM_OK) return FRAM_ERROR;

    /* Verify */
    if (memcmp(write_data, read_data, 8) != 0) return FRAM_ERROR;

    return FRAM_OK;
}
