/* usbd_composite.h — USB Composite MSC + CDC Class */
#ifndef __USBD_COMPOSITE_H
#define __USBD_COMPOSITE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_ioreq.h"
#include "usbd_msc.h"
#include "usbd_cdc.h"

/* 1 = CDC COM only (no Windows USB Drive / MSC). FatFs still reads SD in firmware. */
#ifndef CDC_ONLY_NO_MSC
#define CDC_ONLY_NO_MSC  1
#endif

/* ── Composite descriptor total size ─────────────────────────── */
/*
 *  9  Config descriptor
 *  8  IAD (CDC)
 *  9  CDC Communication Interface
 *  5  CDC Header FD
 *  5  CDC Call Mgmt FD
 *  4  CDC ACM FD
 *  5  CDC Union FD
 *  7  CDC Interrupt IN EP
 *  9  CDC Data Interface
 *  7  CDC Bulk OUT EP
 *  7  CDC Bulk IN EP
 *  [optional MSC: 9+7+7]
 */
#if CDC_ONLY_NO_MSC
#define USB_COMPOSITE_CONFIG_DESC_SIZ   75U
#else
#define USB_COMPOSITE_CONFIG_DESC_SIZ   98U
#endif

/* ── Composite class handle ───────────────────────────────────── */
typedef struct {
    USBD_MSC_BOT_HandleTypeDef msc;
    USBD_CDC_HandleTypeDef     cdc;
} USBD_Composite_HandleTypeDef;

extern USBD_ClassTypeDef USBD_COMPOSITE;

/* Global composite handle — accessible from usbd_cdc_if.c */
extern USBD_Composite_HandleTypeDef composite;

#ifdef __cplusplus
}
#endif
#endif /* __USBD_COMPOSITE_H */
