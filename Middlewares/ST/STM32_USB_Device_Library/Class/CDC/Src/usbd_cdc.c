/* usbd_cdc.c — USB CDC Helper Functions
 * The composite class manages CDC state directly via the global composite handle.
 * These helpers are provided for API compatibility but are not used internally.
 */
#include "usbd_cdc.h"
#include "usbd_composite.h"
#include "usbd_ctlreq.h"

/* ── USBD_CDC_SetTxBuffer ─────────────────────────────────────── */
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev,
                              uint8_t *pbuff, uint32_t length)
{
    UNUSED(pdev);
    composite.cdc.TxBuffer = pbuff;
    composite.cdc.TxLength = length;
    return USBD_OK;
}

/* ── USBD_CDC_SetRxBuffer ─────────────────────────────────────── */
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff)
{
    UNUSED(pdev);
    composite.cdc.RxBuffer = pbuff;
    return USBD_OK;
}

/* ── USBD_CDC_TransmitPacket ──────────────────────────────────── */
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev)
{
    if (composite.cdc.TxState != 0U) return USBD_BUSY;

    composite.cdc.TxState = 1U;
    USBD_LL_Transmit(pdev, CDC_IN_EP,
                     composite.cdc.TxBuffer,
                     composite.cdc.TxLength);
    return USBD_OK;
}

/* ── USBD_CDC_ReceivePacket ───────────────────────────────────── */
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev)
{
    USBD_LL_PrepareReceive(pdev, CDC_OUT_EP,
                           composite.cdc.RxBuffer,
                           CDC_DATA_FS_MAX_PACKET_SIZE);
    return USBD_OK;
}
