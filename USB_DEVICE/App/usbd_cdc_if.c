/* usbd_cdc_if.c — CDC Application Interface for Audiometer AX60
 *
 * Integrated with AX60 WebUI protocol and direct UART bridge.
 */

#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "usbd_composite.h"
#include "main.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_Composite_HandleTypeDef composite;
extern void WebUI_UartRxByte(uint8_t byte);

/* ── Receive working buffer (64-byte CDC packets land here) ───── */
static uint8_t rx_packet[CDC_DATA_FS_MAX_PACKET_SIZE];

/* ── TX buffer for packets ───────────────────────────────────── */
#define CDC_TX_BUF_SIZE  256U
static uint8_t tx_buf[CDC_TX_BUF_SIZE];

/* ── Line-coding shadow (returned on GET_LINE_CODING) ────────── */
static USBD_CDC_LineCodingTypeDef line_coding = {
    .bitrate    = 115200U,
    .format     = 0U,   /* 1 stop bit  */
    .paritytype = 0U,   /* No parity   */
    .datatype   = 8U,   /* 8 data bits */
};

uint8_t *CDC_GetRxBuffer(void)
{
    return rx_packet;
}

int8_t CDC_Init_FS(void)
{
    composite.cdc.TxState = 0U;
    return 0;
}

int8_t CDC_DeInit_FS(void)
{
    return 0;
}

int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    UNUSED(length);
    switch (cmd)
    {
        case CDC_SET_LINE_CODING:
            line_coding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |
                                                (pbuf[2] << 16) | (pbuf[3] << 24));
            line_coding.format     = pbuf[4];
            line_coding.paritytype = pbuf[5];
            line_coding.datatype   = pbuf[6];
            break;

        case CDC_GET_LINE_CODING:
            pbuf[0] = (uint8_t)(line_coding.bitrate);
            pbuf[1] = (uint8_t)(line_coding.bitrate >> 8);
            pbuf[2] = (uint8_t)(line_coding.bitrate >> 16);
            pbuf[3] = (uint8_t)(line_coding.bitrate >> 24);
            pbuf[4] = line_coding.format;
            pbuf[5] = line_coding.paritytype;
            pbuf[6] = line_coding.datatype;
            break;

        case CDC_SET_CONTROL_LINE_STATE:
            break;

        default:
            break;
    }
    return 0;
}

/* CDC_Receive_FS — routes received bytes directly to WebUI frame parser */
int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
    if (Buf != NULL && Len != NULL && *Len > 0U)
    {
        for (uint32_t i = 0; i < *Len; i++)
        {
            WebUI_UartRxByte(Buf[i]);
        }
    }
    return 0;
}

int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
    UNUSED(Buf); UNUSED(Len); UNUSED(epnum);
    composite.cdc.TxState = 0U;
    return 0;
}

void CDC_UnstickTx(void)
{
    composite.cdc.TxState = 0U;
}

/* CDC_Transmit_FS — transmit packet out to PC over USB Virtual COM Port */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint32_t Len)
{
    USBD_CDC_HandleTypeDef *hcdc = &composite.cdc;
    uint32_t copy_len;

    if (hcdc->TxState != 0U)
    {
        return USBD_BUSY;
    }

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return USBD_FAIL;
    }

    copy_len = (Len < CDC_TX_BUF_SIZE) ? Len : CDC_TX_BUF_SIZE;
    memcpy(tx_buf, Buf, copy_len);

    hcdc->TxBuffer = tx_buf;
    hcdc->TxLength = copy_len;
    hcdc->TxState  = 1U;

    return USBD_LL_Transmit(&hUsbDeviceFS, CDC_IN_EP, tx_buf, copy_len);
}
