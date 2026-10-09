/**
  ******************************************************************************
  * @file           : ds1302.c
  * @brief          : DS1302 Real-Time Clock Driver Source for STM32F4 (AX60)
  ******************************************************************************
  */

#include "ds1302.h"

/* Diagnostic variables visible in STM32CubeIDE Live Expressions */
volatile uint8_t ds1302_dbg_burst[8] = {0};
volatile uint8_t ds1302_dbg_last_ok = 0;
volatile uint8_t ds1302_dbg_sec_reg = 0;

/* ── BCD Conversion Utilities ─────────────────────────────────────── */
static uint8_t dec_to_bcd(uint8_t val)
{
    return (uint8_t)(((val / 10) << 4) | (val % 10));
}

static uint8_t bcd_to_dec(uint8_t val)
{
    return (uint8_t)(((val >> 4) * 10) + (val & 0x0F));
}

/* ── Microsecond-level Bit-Bang Delay ─────────────────────────────── */
static inline void DS1302_Delay(void)
{
    for (volatile int i = 0; i < 150; i++)
    {
        __NOP();
    }
}

/* ── GPIO Pin Helpers & Direction Control ─────────────────────────── */
static inline void CE_High(void)   { HAL_GPIO_WritePin(DS1302_CE_PORT, DS1302_CE_PIN, GPIO_PIN_SET); }
static inline void CE_Low(void)    { HAL_GPIO_WritePin(DS1302_CE_PORT, DS1302_CE_PIN, GPIO_PIN_RESET); }
static inline void SCLK_High(void) { HAL_GPIO_WritePin(DS1302_SCLK_PORT, DS1302_SCLK_PIN, GPIO_PIN_SET); }
static inline void SCLK_Low(void)  { HAL_GPIO_WritePin(DS1302_SCLK_PORT, DS1302_SCLK_PIN, GPIO_PIN_RESET); }
static inline void IO_High(void)   { HAL_GPIO_WritePin(DS1302_IO_PORT, DS1302_IO_PIN, GPIO_PIN_SET); }
static inline void IO_Low(void)    { HAL_GPIO_WritePin(DS1302_IO_PORT, DS1302_IO_PIN, GPIO_PIN_RESET); }
static inline uint8_t IO_Read(void){ return (HAL_GPIO_ReadPin(DS1302_IO_PORT, DS1302_IO_PIN) == GPIO_PIN_SET) ? 1 : 0; }

static inline void IO_SetOutput(void)
{
    // PB9 MODER = 01 (General purpose output mode)
    GPIOB->MODER &= ~(3U << (9 * 2));
    GPIOB->MODER |=  (1U << (9 * 2));
}

static inline void IO_SetInput(void)
{
    // PB9 MODER = 00 (Input mode)
    GPIOB->MODER &= ~(3U << (9 * 2));
}

/* ── Low-Level Bit-Bang 3-Wire SPI Protocol ───────────────────────── */
static void DS1302_WriteByte_Raw(uint8_t byte)
{
    IO_SetOutput();
    
    for (uint8_t i = 0; i < 8; i++)
    {
        if (byte & (1 << i))
        {
            IO_High();
        }
        else
        {
            IO_Low();
        }
        DS1302_Delay();
        
        // DS1302 samples data on the rising edge of SCLK
        SCLK_High();
        DS1302_Delay();
        SCLK_Low();
        DS1302_Delay();
    }
}

static uint8_t DS1302_ReadByte_Raw(void)
{
    uint8_t byte = 0;
    
    IO_SetInput();
    
    for (uint8_t i = 0; i < 8; i++)
    {
        DS1302_Delay();
        if (IO_Read())
        {
            byte |= (1 << i);
        }
        
        // SCLK pulse: DS1302 shifts out next bit on falling edge of SCLK
        SCLK_High();
        DS1302_Delay();
        SCLK_Low();
        DS1302_Delay();
    }
    
    return byte;
}

/* ── Single Register Read / Write ─────────────────────────────────── */
void DS1302_WriteReg(uint8_t reg, uint8_t val)
{
    CE_High();
    DS1302_Delay();
    DS1302_WriteByte_Raw(reg & ~0x01); // Write command (bit 0 = 0)
    DS1302_WriteByte_Raw(val);
    CE_Low();
    DS1302_Delay();
}

uint8_t DS1302_ReadReg(uint8_t reg)
{
    uint8_t val = 0;
    
    CE_High();
    DS1302_Delay();
    DS1302_WriteByte_Raw(reg | 0x01);  // Read command (bit 0 = 1)
    val = DS1302_ReadByte_Raw();
    CE_Low();
    DS1302_Delay();
    
    return val;
}

/* ── Public Driver Functions ──────────────────────────────────────── */

/**
  * @brief  Initializes the GPIO pins and configures the DS1302 chip.
  * @retval 1 if successful, 0 if communication failed.
  */
uint8_t DS1302_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. Configure CE Pin (PB1) as Output Push-Pull (default LOW)
    HAL_GPIO_WritePin(DS1302_CE_PORT, DS1302_CE_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = DS1302_CE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DS1302_CE_PORT, &GPIO_InitStruct);

    // 2. Configure SCLK Pin (PB6) as Output Push-Pull (default LOW)
    HAL_GPIO_WritePin(DS1302_SCLK_PORT, DS1302_SCLK_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = DS1302_SCLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DS1302_SCLK_PORT, &GPIO_InitStruct);

    // 3. Configure I/O Pin (PB9) as Output Push-Pull with Pull-up
    HAL_GPIO_WritePin(DS1302_IO_PORT, DS1302_IO_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = DS1302_IO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DS1302_IO_PORT, &GPIO_InitStruct);

    CE_Low();
    SCLK_Low();
    DS1302_Delay();

    // Disable Write Protection (write 0x00 to register 0x8E)
    DS1302_WriteReg(DS1302_REG_WP, 0x00);

    // Disable Trickle Charger (safe for primary coin cells)
    DS1302_WriteReg(DS1302_REG_TRICKLE, 0x00);

    // Check if the oscillator is halted (Bit 7 of Seconds register = 1)
    uint8_t sec_reg = DS1302_ReadReg(DS1302_REG_SECONDS);
    if (sec_reg & 0x80)
    {
        // Clear CH bit (Bit 7) to start the 32.768kHz crystal oscillator
        DS1302_WriteReg(DS1302_REG_SECONDS, sec_reg & 0x7F);
    }

    return 1;
}

/**
  * @brief  Checks if the DS1302 32.768kHz oscillator is running.
  * @retval 1 if running, 0 if halted.
  */
uint8_t DS1302_IsOscillatorRunning(void)
{
    uint8_t sec_reg = DS1302_ReadReg(DS1302_REG_SECONDS);
    return ((sec_reg & 0x80) == 0) ? 1 : 0;
}

/**
  * @brief  Reads the current Date and Time from DS1302.
  * @param  dt: Pointer to output DateTime structure
  * @retval 1 on success, 0 on invalid pointer or corrupted reading.
  */
uint8_t DS1302_GetTime(DS1302_DateTime_t *dt)
{
    if (!dt) return 0;

    uint8_t burst[8] = {0};

    CE_High();
    DS1302_Delay();
    
    // Command 0xBF: Clock Burst Read
    DS1302_WriteByte_Raw(DS1302_REG_BURST_CLOCK | 0x01);
    
    for (uint8_t i = 0; i < 8; i++)
    {
        burst[i] = DS1302_ReadByte_Raw();
        ds1302_dbg_burst[i] = burst[i];
    }
    
    CE_Low();
    DS1302_Delay();

    // Decode BCD values
    dt->sec     = bcd_to_dec(burst[0] & 0x7F);
    dt->min     = bcd_to_dec(burst[1] & 0x7F);
    
    // Decode Hour (Handle 24-hour mode bit 7=0, 12-hour mode bit 7=1)
    if (burst[2] & 0x80) // 12-hour mode
    {
        uint8_t hr = bcd_to_dec(burst[2] & 0x1F);
        if (burst[2] & 0x20) // PM
        {
            if (hr < 12) hr += 12;
        }
        else // AM
        {
            if (hr == 12) hr = 0;
        }
        dt->hour = hr;
    }
    else // 24-hour mode
    {
        dt->hour = bcd_to_dec(burst[2] & 0x3F);
    }

    dt->day     = bcd_to_dec(burst[3] & 0x3F);
    dt->month   = bcd_to_dec(burst[4] & 0x1F);
    dt->weekday = bcd_to_dec(burst[5] & 0x07);
    dt->year    = 2000 + bcd_to_dec(burst[6]);

    // Sanity check to detect detached chip or floating bus
    if (dt->month == 0 || dt->month > 12 || dt->day == 0 || dt->day > 31 ||
        dt->hour > 23 || dt->min > 59 || dt->sec > 59)
    {
        // Try fallback individual register reads (reliable across DS1302 clone variants)
        uint8_t s = DS1302_ReadReg(DS1302_REG_SECONDS);
        uint8_t m = DS1302_ReadReg(DS1302_REG_MINUTES);
        uint8_t h = DS1302_ReadReg(DS1302_REG_HOURS);
        uint8_t d = DS1302_ReadReg(DS1302_REG_DATE);
        uint8_t mo = DS1302_ReadReg(DS1302_REG_MONTH);
        uint8_t w = DS1302_ReadReg(DS1302_REG_DAY);
        uint8_t y = DS1302_ReadReg(DS1302_REG_YEAR);

        ds1302_dbg_sec_reg = s;

        dt->sec     = bcd_to_dec(s & 0x7F);
        dt->min     = bcd_to_dec(m & 0x7F);
        dt->hour    = bcd_to_dec(h & 0x3F);
        dt->day     = bcd_to_dec(d & 0x3F);
        dt->month   = bcd_to_dec(mo & 0x1F);
        dt->weekday = bcd_to_dec(w & 0x07);
        dt->year    = 2000 + bcd_to_dec(y);

        if (dt->month == 0 || dt->month > 12 || dt->day == 0 || dt->day > 31 ||
            dt->hour > 23 || dt->min > 59 || dt->sec > 59)
        {
            ds1302_dbg_last_ok = 0;
            return 0; // Data read is invalid / hardware not responding
        }
    }

    ds1302_dbg_last_ok = 1;
    return 1;
}

/**
  * @brief  Sets the Date and Time on DS1302.
  * @param  dt: Pointer to input DateTime structure
  * @retval 1 on success, 0 on invalid pointer or range.
  */
uint8_t DS1302_SetTime(const DS1302_DateTime_t *dt)
{
    if (!dt) return 0;
    if (dt->month == 0 || dt->month > 12 || dt->day == 0 || dt->day > 31 ||
        dt->hour > 23 || dt->min > 59 || dt->sec > 59)
    {
        return 0;
    }

    // Disable Write Protection
    DS1302_WriteReg(DS1302_REG_WP, 0x00);

    uint8_t burst[8] = {0};
    burst[0] = dec_to_bcd(dt->sec);         // CH bit = 0 (oscillator enabled)
    burst[1] = dec_to_bcd(dt->min);
    burst[2] = dec_to_bcd(dt->hour);        // Bit 7 = 0 for 24-hour format
    burst[3] = dec_to_bcd(dt->day);
    burst[4] = dec_to_bcd(dt->month);
    burst[5] = dec_to_bcd(dt->weekday);
    burst[6] = dec_to_bcd((uint8_t)(dt->year % 100));
    burst[7] = 0x00;                        // WP disabled

    CE_High();
    DS1302_Delay();
    
    // Command 0xBE: Clock Burst Write
    DS1302_WriteByte_Raw(DS1302_REG_BURST_CLOCK & ~0x01);
    
    for (uint8_t i = 0; i < 8; i++)
    {
        DS1302_WriteByte_Raw(burst[i]);
    }
    
    CE_Low();
    DS1302_Delay();

    return 1;
}
