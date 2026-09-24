/* usbd_cdc.h — USB CDC Class Header */
#ifndef __USBD_CDC_H
#define __USBD_CDC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_ioreq.h"

/* ── Endpoint addresses ───────────────────────────────────────── */
#define CDC_IN_EP                   0x82U   /* EP2 IN  – data device→host  */
#define CDC_OUT_EP                  0x02U   /* EP2 OUT – data host→device  */
#define CDC_CMD_EP                  0x83U   /* EP3 IN  – notifications     */

/* ── Packet sizes ─────────────────────────────────────────────── */
#define CDC_DATA_FS_MAX_PACKET_SIZE 64U
#define CDC_CMD_PACKET_SIZE         8U

/* ── CDC class-specific requests ──────────────────────────────── */
#define CDC_SEND_ENCAPSULATED_COMMAND   0x00U
#define CDC_GET_ENCAPSULATED_RESPONSE   0x01U
#define CDC_SET_COMM_FEATURE            0x02U
#define CDC_GET_COMM_FEATURE            0x03U
#define CDC_CLEAR_COMM_FEATURE          0x04U
#define CDC_SET_LINE_CODING             0x20U
#define CDC_GET_LINE_CODING             0x21U
#define CDC_SET_CONTROL_LINE_STATE      0x22U
#define CDC_SEND_BREAK                  0x23U

/* ── Data structures ──────────────────────────────────────────── */
typedef struct {
    uint32_t bitrate;
    uint8_t  format;
    uint8_t  paritytype;
    uint8_t  datatype;
} USBD_CDC_LineCodingTypeDef;

/* Application interface operations (provided by usbd_cdc_if.c) */
typedef struct {
    int8_t (* Init)     (void);
    int8_t (* DeInit)   (void);
    int8_t (* Control)  (uint8_t cmd, uint8_t *pbuf, uint16_t length);
    int8_t (* Receive)  (uint8_t *Buf, uint32_t *Len);
    int8_t (* TransmitCplt)(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
} USBD_CDC_ItfTypeDef;

/* Internal CDC state (lives inside composite handle) */
typedef struct {
    uint32_t data[CDC_DATA_FS_MAX_PACKET_SIZE / 4U]; /* ctrl data buffer */
    uint8_t  CmdOpCode;
    uint8_t  CmdLength;
    uint8_t *RxBuffer;
    uint8_t *TxBuffer;
    uint32_t RxLength;
    uint32_t TxLength;
    __IO uint32_t TxState;
    __IO uint32_t RxState;
} USBD_CDC_HandleTypeDef;

/* ── Helper function prototypes (called by composite) ─────────── */
uint8_t USBD_CDC_SetTxBuffer  (USBD_HandleTypeDef *pdev,
                                uint8_t *pbuff, uint32_t length);
uint8_t USBD_CDC_SetRxBuffer  (USBD_HandleTypeDef *pdev, uint8_t *pbuff);
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev);
uint8_t USBD_CDC_ReceivePacket (USBD_HandleTypeDef *pdev);

#ifdef __cplusplus
}
#endif
#endif /* __USBD_CDC_H */
