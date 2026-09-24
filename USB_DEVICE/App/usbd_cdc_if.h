/* usbd_cdc_if.h — CDC Application Interface */
#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_cdc.h"

/* json_buf / json_ready live in view/view_cdc_json.h (View owns assembly). */

/* ── Called by composite class ───────────────────────────────── */
uint8_t *CDC_GetRxBuffer    (void);
int8_t   CDC_Init_FS        (void);
int8_t   CDC_DeInit_FS      (void);
int8_t   CDC_Control_FS     (uint8_t cmd, uint8_t *pbuf, uint16_t length);
int8_t   CDC_Receive_FS     (uint8_t *Buf, uint32_t *Len);
int8_t   CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum);

/* ── Called from BSP_CDC_Send ────────────────────────────────── */
void     CDC_UnstickTx      (void);
uint8_t  CDC_Transmit_FS    (uint8_t *Buf, uint32_t Len);
uint8_t  CDC_Transmit_Wait  (uint32_t timeout_ms);
uint8_t  CDC_TransmitBuffer (const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif
#endif /* __USBD_CDC_IF_H */
