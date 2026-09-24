/* Core/Src/sd_card.c */
#include "sd_card.h"
#include "main.h"

#define SD_CS_LOW()   HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)
#define SD_CS_HIGH()  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET)

#define CMD0   0
#define CMD1   1
#define CMD8   8
#define CMD9   9
#define CMD12  12
#define CMD16  16
#define CMD17  17
#define CMD18  18
#define CMD23  23
#define CMD24  24
#define CMD25  25
#define CMD41  41
#define CMD55  55
#define CMD58  58

extern SPI_HandleTypeDef hspi3;
volatile uint8_t dbg_acmd_res = 0;
volatile uint8_t dbg_cmd0_res = 0;
volatile uint8_t dbg_cmd8_res = 0;
volatile uint8_t dbg_cmd55_res = 0;
volatile uint8_t dbg_cmd41_res = 0;

uint8_t  sd_initialized = 0;
volatile uint8_t sd_msc_busy = 0;
static uint8_t sd_type = 0;

/* ── SPI: pure HAL, no register access ─────────────────────── */
static uint8_t SPI_SendByte(uint8_t data)
{
    uint8_t rx = 0xFF;
    HAL_SPI_TransmitReceive(&hspi3, &data, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

/* ── SD low-level ───────────────────────────────────────────── */
uint8_t SD_WaitReady(void)
{
    uint8_t r;
    uint32_t deadline = HAL_GetTick() + 500U;

    do {
        r = SPI_SendByte(0xFF);
        if (r == 0xFF) {
            return SD_OK;
        }
    } while (HAL_GetTick() < deadline);

    return SD_ERROR;
}

static void SD_Deselect(void)
{
    SD_CS_HIGH();
    SPI_SendByte(0xFF);
}

static uint8_t SD_Select(void)
{
    SD_CS_LOW();
    SPI_SendByte(0xFF);
    if (SD_WaitReady() == SD_OK) return 0;
    SD_CS_HIGH();
    return 1; /* timeout */
}

static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
    uint8_t crc, res;

    /* ACMD = send CMD55 first */
    if (cmd & 0x80)
    {
        cmd &= 0x7F;
        res = SD_SendCmd(CMD55, 0);
        dbg_cmd55_res = res;
        if (res > 1) return res;
    }

    SD_Deselect();
    if (SD_Select() != 0) return 0xFF;

    SPI_SendByte(0x40 | cmd);
    SPI_SendByte((uint8_t)(arg >> 24));
    SPI_SendByte((uint8_t)(arg >> 16));
    SPI_SendByte((uint8_t)(arg >> 8));
    SPI_SendByte((uint8_t)arg);

    crc = 0x01;
    if (cmd == CMD0) crc = 0x95;
    if (cmd == CMD8) crc = 0x87;
    SPI_SendByte(crc);

    if (cmd == CMD12) SPI_SendByte(0xFF); /* skip one byte for CMD12 */

    uint8_t n = 10;
    do { res = SPI_SendByte(0xFF); } while ((res & 0x80) && --n);

    if (cmd == CMD0) dbg_cmd0_res = res;
    if (cmd == CMD8) dbg_cmd8_res = res;
    if (cmd == CMD41) dbg_cmd41_res = res;

    return res;
}

volatile uint8_t dbg_sd_err_step = 0;

/* ── Public functions ───────────────────────────────────────── */
void SD_SetMscBusy(uint8_t busy)
{
    sd_msc_busy = busy ? 1U : 0U;
}

uint8_t SD_IsMscBusy(void)
{
    return sd_msc_busy;
}

void SD_PauseUsbMsc(void)
{
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
}

void SD_ResumeUsbMsc(void)
{
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

void SD_ReleaseBus(void)
{
    SD_CS_HIGH();
    for (uint8_t i = 0; i < 8; i++)
    {
        SPI_SendByte(0xFF);
    }
}

static void SD_SetSpiPrescaler(uint32_t prescaler)
{
    __HAL_SPI_DISABLE(&hspi3);
    MODIFY_REG(hspi3.Instance->CR1, SPI_CR1_BR, prescaler);
    hspi3.Init.BaudRatePrescaler = prescaler;
    __HAL_SPI_ENABLE(&hspi3);
}

uint8_t SD_Init(void)
{
    uint8_t  n, type, ocr[4];
    uint16_t timeout;

    if (sd_initialized)
    {
        return SD_OK;
    }

    sd_initialized = 0;
    sd_type = 0;
    dbg_sd_err_step = 0;

    /* Give the card time to power up and stabilize */
    HAL_Delay(100);

    /* 200 dummy clocks (25 * 8) with CS HIGH */
    SD_CS_HIGH();
    for (n = 0; n < 25; n++) SPI_SendByte(0xFF);
    HAL_Delay(10);

    type = 0;
    timeout = 1000;

    /* Retry CMD0 up to 50 times (Crucial for weak pull-ups) */
    uint8_t cmd0_retries = 50;
    uint8_t cmd0_res = 0xFF;
    while (cmd0_retries--) {
        cmd0_res = SD_SendCmd(CMD0, 0);
        if (cmd0_res == 1) break;
        HAL_Delay(10);
    }

    if (cmd0_res == 1)                   /* Enter idle */
    {
        if (SD_SendCmd(CMD8, 0x1AA) == 1)           /* SDv2? */
        {
            for (n = 0; n < 4; n++) ocr[n] = SPI_SendByte(0xFF);
            if (ocr[2] == 0x01 && ocr[3] == 0xAA)
            {
                timeout = 50000; /* Give the SD card plenty of time to boot! */
                uint8_t acmd_res = 1;
                while (timeout) {
                    acmd_res = SD_SendCmd((0x80 | CMD41), 1UL << 30);
                    dbg_acmd_res = acmd_res;
                    if (acmd_res == 0) break; /* Success! Card is ready! */
                    timeout--;
                    HAL_Delay(1); /* Actually wait 1ms per attempt! */
                }
                
                if (acmd_res != 0) {
                    dbg_sd_err_step = 7; /* ACMD41 timed out, card stuck in idle */
                } else if (SD_SendCmd(CMD58, 0) == 0) {
                    for (n = 0; n < 4; n++) ocr[n] = SPI_SendByte(0xFF);
                    type = (ocr[0] & 0x40) ? 6 : 2;
                } else { 
                    dbg_sd_err_step = 4; /* CMD58 failed */
                }
            } else { dbg_sd_err_step = 3; }
        }
        else                                        /* SDv1 or MMC */
        {
            uint8_t cmd;
            if (SD_SendCmd((0x80 | CMD41), 0) <= 1)
                { type = 2; cmd = (0x80 | CMD41); }
            else
                { type = 1; cmd = CMD1; }

            while (timeout-- && SD_SendCmd(cmd, 0));
            if (!timeout || SD_SendCmd(CMD16, 512) != 0)
                { type = 0; dbg_sd_err_step = 5; }
        }
    } else { dbg_sd_err_step = 1; }

    if (type)
    {
        SPI_SendByte(0xFF);
        sd_type = type;
        sd_initialized = 1;

        /* Slow down SPI clock to allow the weak internal MISO pull-up to work! */
        SD_SetSpiPrescaler(SPI_BAUDRATEPRESCALER_32);

        (void)SD_GetBlocks();

        return SD_OK;
    }

    SD_Deselect();
    return SD_ERROR;
}

uint8_t SD_ReadBlock(uint8_t *buf, uint32_t sector, uint32_t count)
{
    if (!sd_initialized) return SD_ERROR;
    if (sd_type != 6) sector *= 512;

    if (count == 1) {
        if (SD_SendCmd(CMD17, sector) != 0) { SD_Deselect(); return SD_ERROR; }
        uint32_t t = 5000000; uint8_t token;
        do { token = SPI_SendByte(0xFF); } while (token == 0xFF && --t);
        if (token != 0xFE) { SD_Deselect(); return SD_ERROR; }
        
        for (int i = 0; i < 512; i++) {
            while ((hspi3.Instance->SR & SPI_FLAG_TXE) == 0);
            *(__IO uint8_t *)&hspi3.Instance->DR = 0xFF;
            while ((hspi3.Instance->SR & SPI_FLAG_RXNE) == 0);
            *buf++ = *(__IO uint8_t *)&hspi3.Instance->DR;
        }
        SPI_SendByte(0xFF); SPI_SendByte(0xFF);
    } else {
        if (SD_SendCmd(CMD18, sector) != 0) { SD_Deselect(); return SD_ERROR; }
        while (count--) {
            uint32_t t = 5000000; uint8_t token;
            do { token = SPI_SendByte(0xFF); } while (token == 0xFF && --t);
            if (token != 0xFE) { SD_SendCmd(CMD12, 0); SD_WaitReady(); SD_Deselect(); return SD_ERROR; }
            
            for (int i = 0; i < 512; i++) {
                while ((hspi3.Instance->SR & SPI_FLAG_TXE) == 0);
                *(__IO uint8_t *)&hspi3.Instance->DR = 0xFF;
                while ((hspi3.Instance->SR & SPI_FLAG_RXNE) == 0);
                *buf++ = *(__IO uint8_t *)&hspi3.Instance->DR;
            }
            SPI_SendByte(0xFF); SPI_SendByte(0xFF);
        }
        SD_SendCmd(CMD12, 0);
        SD_WaitReady();
    }
    SD_Deselect();
    return SD_OK;
}

uint8_t SD_WriteBlock(const uint8_t *buf, uint32_t sector, uint32_t count)
{
    if (!sd_initialized) return SD_ERROR;
    if (sd_type != 6) sector *= 512;

    if (count == 1) {
        if (SD_SendCmd(CMD24, sector) != 0) { SD_Deselect(); return SD_ERROR; }
        SPI_SendByte(0xFF); SPI_SendByte(0xFE);
        
        for (int i = 0; i < 512; i++) {
            while ((hspi3.Instance->SR & SPI_FLAG_TXE) == 0);
            *(__IO uint8_t *)&hspi3.Instance->DR = *buf++;
            while ((hspi3.Instance->SR & SPI_FLAG_RXNE) == 0);
            (void)*(__IO uint8_t *)&hspi3.Instance->DR; 
        }
        SPI_SendByte(0xFF); SPI_SendByte(0xFF);
        uint32_t t = 10000; uint8_t status;
        do { status = SPI_SendByte(0xFF); } while (status == 0xFF && --t);
        if ((status & 0x1F) != 0x05) { SD_Deselect(); return SD_ERROR; }
        if (SD_WaitReady() != SD_OK) { SD_Deselect(); return SD_ERROR; }
    } else {
        if (SD_SendCmd(CMD25, sector) != 0) { SD_Deselect(); return SD_ERROR; }
        while (count--) {
            SPI_SendByte(0xFF); SPI_SendByte(0xFC);
            
            for (int i = 0; i < 512; i++) {
                while ((hspi3.Instance->SR & SPI_FLAG_TXE) == 0);
                *(__IO uint8_t *)&hspi3.Instance->DR = *buf++;
                while ((hspi3.Instance->SR & SPI_FLAG_RXNE) == 0);
                (void)*(__IO uint8_t *)&hspi3.Instance->DR; 
            }
            SPI_SendByte(0xFF); SPI_SendByte(0xFF);
            uint32_t t = 10000; uint8_t status;
            do { status = SPI_SendByte(0xFF); } while (status == 0xFF && --t);
            if ((status & 0x1F) != 0x05) { SD_Deselect(); return SD_ERROR; }
            if (SD_WaitReady() != SD_OK) { SD_Deselect(); return SD_ERROR; }
        }
        SPI_SendByte(0xFD);
        if (SD_WaitReady() != SD_OK) { SD_Deselect(); return SD_ERROR; }
    }

    SPI_SendByte(0xFF);
    SD_Deselect();
    return SD_OK;
}

static uint32_t cached_capacity = 0;

uint32_t SD_GetBlocks(void)
{
    uint8_t  csd[16];
    uint32_t capacity = 0;
    uint8_t  token;

    if (!sd_initialized) return 0;
    if (cached_capacity > 0) return cached_capacity;

    if (SD_Select() != 0) return 0;

    if (SD_SendCmd(CMD9, 0) == 0)
    {
        uint32_t t = 5000000;
        do { token = SPI_SendByte(0xFF); } while (token == 0xFF && --t);

        if (token == 0xFE)
        {
            for (int i = 0; i < 16; i++) csd[i] = SPI_SendByte(0xFF);
            SPI_SendByte(0xFF); SPI_SendByte(0xFF); /* CRC */

            if ((csd[0] >> 6) == 1)                /* CSD v2 — SDHC/SDXC */
            {
                uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16)
                                | ((uint16_t)csd[8] << 8)
                                | csd[9];
                capacity = (c_size + 1) * 1024;
            }
            else                                   /* CSD v1 — SDv1/MMC */
            {
                uint8_t  n     = ((csd[9] & 0x03) << 1) | (csd[10] >> 7);
                uint8_t  bl    = csd[5] & 0x0F;
                uint32_t csize = ((uint32_t)(csd[6] & 0x03) << 10)
                               | ((uint16_t)csd[7] << 2)
                               | (csd[8] >> 6);
                capacity = (csize + 1) << (n + bl - 7);
            }
        }
    }

    SD_Deselect();
    if (capacity > 0) {
        cached_capacity = capacity;
    }
    return capacity;
}
