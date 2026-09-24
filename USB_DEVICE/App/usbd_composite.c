/* usbd_composite.c — USB Composite MSC + CDC Class Implementation
 *
 * Endpoint map:
 *   EP0         Control (bidirectional)
 *   EP1 IN 0x81 MSC Bulk IN
 *   EP1 OUT 0x01 MSC Bulk OUT
 *   EP2 IN 0x82 CDC Data Bulk IN
 *   EP2 OUT 0x02 CDC Data Bulk OUT
 *   EP3 IN 0x83 CDC Command Interrupt IN
 *
 * Interface map:
 *   Interface 0  CDC Communication (+ IAD)
 *   Interface 1  CDC Data
 *   Interface 2  MSC
 */

#include "usbd_composite.h"
#include "usbd_msc_bot.h"
#include "usbd_msc_scsi.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"

/* ── Global handle (shared with usbd_cdc_if.c) ───────────────── */
USBD_Composite_HandleTypeDef composite;

/* ── Forward declarations ─────────────────────────────────────── */
static uint8_t COMPOSITE_Init          (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t COMPOSITE_DeInit        (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t COMPOSITE_Setup         (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t COMPOSITE_DataIn        (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t COMPOSITE_DataOut       (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t COMPOSITE_EP0_RxReady  (USBD_HandleTypeDef *pdev);
static uint8_t *COMPOSITE_GetCfgDesc  (uint16_t *length);
static uint8_t *COMPOSITE_GetDeviceQualifierDesc(uint16_t *length);

/* ── Class object ─────────────────────────────────────────────── */
USBD_ClassTypeDef USBD_COMPOSITE = {
    COMPOSITE_Init,
    COMPOSITE_DeInit,
    COMPOSITE_Setup,
    NULL,                            /* EP0_TxSent */
    COMPOSITE_EP0_RxReady,
    COMPOSITE_DataIn,
    COMPOSITE_DataOut,
    NULL,                            /* SOF */
    NULL,                            /* IsoINIncomplete */
    NULL,                            /* IsoOUTIncomplete */
    COMPOSITE_GetCfgDesc,
    COMPOSITE_GetCfgDesc,
    COMPOSITE_GetCfgDesc,
    COMPOSITE_GetDeviceQualifierDesc,
};

/* ── Configuration Descriptor ─────────────────────────────────── */
__ALIGN_BEGIN static uint8_t USBD_Composite_CfgDesc[USB_COMPOSITE_CONFIG_DESC_SIZ] __ALIGN_END =
{
    /* ── Configuration Descriptor (9 bytes) ─────────────────── */
    0x09,                                /* bLength */
    USB_DESC_TYPE_CONFIGURATION,         /* bDescriptorType */
    LOBYTE(USB_COMPOSITE_CONFIG_DESC_SIZ),  /* wTotalLength L */
    HIBYTE(USB_COMPOSITE_CONFIG_DESC_SIZ),  /* wTotalLength H */
#if CDC_ONLY_NO_MSC
    0x02,                                /* bNumInterfaces  (CDC only) */
#else
    0x03,                                /* bNumInterfaces  (CDC:2 + MSC:1) */
#endif
    0x01,                                /* bConfigurationValue */
    0x00,                                /* iConfiguration */
    0xC0,                                /* bmAttributes: self-powered */
    0x32,                                /* MaxPower: 100 mA */

    /* ── IAD for CDC (8 bytes) ───────────────────────────────── */
    0x08,                                /* bLength */
    0x0B,                                /* bDescriptorType: IAD */
    0x00,                                /* bFirstInterface: 0 */
    0x02,                                /* bInterfaceCount: 2 */
    0x02,                                /* bFunctionClass: CDC */
    0x02,                                /* bFunctionSubClass: ACM */
    0x01,                                /* bFunctionProtocol: AT */
    0x00,                                /* iFunction */

    /* ── CDC Communication Interface (9 bytes) ───────────────── */
    0x09,                                /* bLength */
    USB_DESC_TYPE_INTERFACE,             /* bDescriptorType */
    0x00,                                /* bInterfaceNumber: 0 */
    0x00,                                /* bAlternateSetting */
    0x01,                                /* bNumEndpoints: 1 (interrupt IN) */
    0x02,                                /* bInterfaceClass: CDC */
    0x02,                                /* bInterfaceSubClass: ACM */
    0x01,                                /* bInterfaceProtocol: AT */
    0x00,                                /* iInterface */

    /* ── CDC Header Functional Descriptor (5 bytes) ──────────── */
    0x05, 0x24, 0x00, 0x10, 0x01,

    /* ── CDC Call Management Functional Descriptor (5 bytes) ─── */
    0x05, 0x24, 0x01, 0x00, 0x01,       /* bDataInterface = 1 */

    /* ── CDC ACM Functional Descriptor (4 bytes) ─────────────── */
    0x04, 0x24, 0x02, 0x02,

    /* ── CDC Union Functional Descriptor (5 bytes) ───────────── */
    0x05, 0x24, 0x06, 0x00, 0x01,       /* master=0, slave=1 */

    /* ── CDC Interrupt IN Endpoint (7 bytes) ─────────────────── */
    0x07,                                /* bLength */
    USB_DESC_TYPE_ENDPOINT,              /* bDescriptorType */
    CDC_CMD_EP,                          /* bEndpointAddress: 0x83 */
    0x03,                                /* bmAttributes: Interrupt */
    LOBYTE(CDC_CMD_PACKET_SIZE),         /* wMaxPacketSize L */
    HIBYTE(CDC_CMD_PACKET_SIZE),         /* wMaxPacketSize H */
    0xFF,                                /* bInterval: 255 ms */

    /* ── CDC Data Interface (9 bytes) ────────────────────────── */
    0x09,                                /* bLength */
    USB_DESC_TYPE_INTERFACE,             /* bDescriptorType */
    0x01,                                /* bInterfaceNumber: 1 */
    0x00,                                /* bAlternateSetting */
    0x02,                                /* bNumEndpoints: 2 */
    0x0A,                                /* bInterfaceClass: CDC-Data */
    0x00,                                /* bInterfaceSubClass */
    0x00,                                /* bInterfaceProtocol */
    0x00,                                /* iInterface */

    /* ── CDC Data Bulk OUT Endpoint (7 bytes) ─────────────────── */
    0x07,
    USB_DESC_TYPE_ENDPOINT,
    CDC_OUT_EP,                          /* 0x02 */
    0x02,                                /* Bulk */
    LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
    HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
    0x00,

    /* ── CDC Data Bulk IN Endpoint (7 bytes) ──────────────────── */
    0x07,
    USB_DESC_TYPE_ENDPOINT,
    CDC_IN_EP,                           /* 0x82 */
    0x02,                                /* Bulk */
    LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
    HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
    0x00,

#if !CDC_ONLY_NO_MSC
    /* ── MSC Interface (9 bytes) ──────────────────────────────── */
    0x09,
    USB_DESC_TYPE_INTERFACE,
    0x02,                                /* bInterfaceNumber: 2 */
    0x00,
    0x02,                                /* bNumEndpoints: 2 */
    0x08,                                /* bInterfaceClass: Mass Storage */
    0x06,                                /* bInterfaceSubClass: SCSI */
    0x50,                                /* bInterfaceProtocol: BOT */
    0x01,                                /* iInterface */

    /* ── MSC Bulk IN Endpoint (7 bytes) ───────────────────────── */
    0x07,
    USB_DESC_TYPE_ENDPOINT,
    MSC_EPIN_ADDR,                       /* 0x81 */
    0x02,                                /* Bulk */
    LOBYTE(MSC_MAX_FS_PACKET),
    HIBYTE(MSC_MAX_FS_PACKET),
    0x00,

    /* ── MSC Bulk OUT Endpoint (7 bytes) ──────────────────────── */
    0x07,
    USB_DESC_TYPE_ENDPOINT,
    MSC_EPOUT_ADDR,                      /* 0x01 */
    0x02,                                /* Bulk */
    LOBYTE(MSC_MAX_FS_PACKET),
    HIBYTE(MSC_MAX_FS_PACKET),
    0x00,
#endif /* !CDC_ONLY_NO_MSC */
};

/* ── Device Qualifier Descriptor ──────────────────────────────── */
__ALIGN_BEGIN static uint8_t USBD_Composite_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
    USB_LEN_DEV_QUALIFIER_DESC,
    USB_DESC_TYPE_DEVICE_QUALIFIER,
    0x00, 0x02,
    0xEF, 0x02, 0x01,
    0x40, 0x01, 0x00,
};

/* ═══════════════════════════════════════════════════════════════ */
/* COMPOSITE_Init                                                   */
/* ═══════════════════════════════════════════════════════════════ */
static uint8_t COMPOSITE_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    UNUSED(cfgidx);

#if !CDC_ONLY_NO_MSC
    /* ── Open MSC endpoints ──────────────────────────────────── */
    USBD_LL_OpenEP(pdev, MSC_EPIN_ADDR,  USBD_EP_TYPE_BULK, MSC_MAX_FS_PACKET);
    USBD_LL_OpenEP(pdev, MSC_EPOUT_ADDR, USBD_EP_TYPE_BULK, MSC_MAX_FS_PACKET);
    pdev->ep_in [MSC_EPIN_ADDR  & 0xFU].is_used = 1U;
    pdev->ep_out[MSC_EPOUT_ADDR & 0xFU].is_used = 1U;
#endif

    /* ── Open CDC endpoints ──────────────────────────────────── */
    USBD_LL_OpenEP(pdev, CDC_IN_EP,  USBD_EP_TYPE_BULK, CDC_DATA_FS_MAX_PACKET_SIZE);
    USBD_LL_OpenEP(pdev, CDC_OUT_EP, USBD_EP_TYPE_BULK, CDC_DATA_FS_MAX_PACKET_SIZE);
    USBD_LL_OpenEP(pdev, CDC_CMD_EP, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
    pdev->ep_in [CDC_IN_EP  & 0xFU].is_used = 1U;
    pdev->ep_out[CDC_OUT_EP & 0xFU].is_used = 1U;
    pdev->ep_in [CDC_CMD_EP & 0xFU].is_used = 1U;

#if !CDC_ONLY_NO_MSC
    /* ── Init MSC BOT state machine ──────────────────────────── */
    memset(&composite.msc, 0, sizeof(composite.msc));
    pdev->pClassData = &composite.msc;   /* MSC BOT functions use pClassData */
    pdev->pClassDataCmsit[0] = &composite.msc;
    MSC_BOT_Init(pdev);
#endif

    /* ── Init CDC handle ─────────────────────────────────────── */
    memset(&composite.cdc, 0, sizeof(composite.cdc));
    /* composite.cdc is accessed directly via extern in usbd_cdc_if.c */

    /* Init CDC application layer */
    CDC_Init_FS();

    /* Prepare first CDC receive */
    composite.cdc.RxBuffer = CDC_GetRxBuffer();
    USBD_LL_PrepareReceive(pdev, CDC_OUT_EP,
                           composite.cdc.RxBuffer,
                           CDC_DATA_FS_MAX_PACKET_SIZE);

    return USBD_OK;
}

/* ═══════════════════════════════════════════════════════════════ */
/* COMPOSITE_DeInit                                                 */
/* ═══════════════════════════════════════════════════════════════ */
static uint8_t COMPOSITE_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    UNUSED(cfgidx);

#if !CDC_ONLY_NO_MSC
    /* MSC */
    pdev->pClassData = &composite.msc;
    pdev->pClassDataCmsit[0] = &composite.msc;
    MSC_BOT_DeInit(pdev);
    USBD_LL_CloseEP(pdev, MSC_EPIN_ADDR);
    USBD_LL_CloseEP(pdev, MSC_EPOUT_ADDR);
#endif

    /* CDC */
    USBD_LL_CloseEP(pdev, CDC_IN_EP);
    USBD_LL_CloseEP(pdev, CDC_OUT_EP);
    USBD_LL_CloseEP(pdev, CDC_CMD_EP);

    CDC_DeInit_FS();

    pdev->pClassData  = NULL;
    pdev->pClassDataCmsit[0] = NULL;
    return USBD_OK;
}

/* ═══════════════════════════════════════════════════════════════ */
/* COMPOSITE_Setup                                                  */
/* ═══════════════════════════════════════════════════════════════ */
static uint8_t COMPOSITE_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    uint8_t  ifalt = 0U;
    uint16_t status_info = 0U;

    uint8_t wIndex_iface = LOBYTE(req->wIndex);

    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
        /* ── Class requests ──────────────────────────────────── */
        case USB_REQ_TYPE_CLASS:
            if (!CDC_ONLY_NO_MSC && (wIndex_iface == 2U))          /* MSC interface */
            {
                pdev->pClassData = &composite.msc;
                pdev->pClassDataCmsit[0] = &composite.msc;
                switch (req->bRequest)
                {
                    case BOT_GET_MAX_LUN:
                        if ((req->wValue == 0U) && (req->wLength == 1U) &&
                            ((req->bmRequest & 0x80U) == 0x80U))
                        {
                            composite.msc.max_lun =
                                ((USBD_StorageTypeDef *)pdev->pUserData[0])->GetMaxLun();
                            USBD_CtlSendData(pdev,
                                             (uint8_t *)&composite.msc.max_lun, 1U);
                        }
                        else USBD_CtlError(pdev, req);
                        break;

                    case BOT_RESET:
                        if ((req->wValue == 0U) && (req->wLength == 0U) &&
                            ((req->bmRequest & 0x80U) != 0x80U))
                        {
                            MSC_BOT_Reset(pdev);
                        }
                        else USBD_CtlError(pdev, req);
                        break;

                    default:
                        USBD_CtlError(pdev, req);
                        break;
                }
            }
            else                              /* CDC interface (0 or 1) */
            {
                switch (req->bRequest)
                {
                    case CDC_SET_LINE_CODING:
                        composite.cdc.CmdOpCode = req->bRequest;
                        composite.cdc.CmdLength = (uint8_t)req->wLength;
                        USBD_CtlPrepareRx(pdev,
                                          (uint8_t *)composite.cdc.data,
                                          req->wLength);
                        break;

                    case CDC_GET_LINE_CODING:
                        CDC_Control_FS(CDC_GET_LINE_CODING,
                                       (uint8_t *)composite.cdc.data, req->wLength);
                        USBD_CtlSendData(pdev,
                                         (uint8_t *)composite.cdc.data, req->wLength);
                        break;

                    case CDC_SET_CONTROL_LINE_STATE:
                        CDC_Control_FS(CDC_SET_CONTROL_LINE_STATE,
                                       (uint8_t *)req, 0U);
                        break;

                    default:
                        break;
                }
            }
            break;

        /* ── Standard requests ───────────────────────────────── */
        case USB_REQ_TYPE_STANDARD:
            switch (req->bRequest)
            {
                case USB_REQ_GET_STATUS:
                    if (pdev->dev_state == USBD_STATE_CONFIGURED)
                        USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
                    else USBD_CtlError(pdev, req);
                    break;

                case USB_REQ_GET_INTERFACE:
                    if (pdev->dev_state == USBD_STATE_CONFIGURED)
                        USBD_CtlSendData(pdev, &ifalt, 1U);
                    else USBD_CtlError(pdev, req);
                    break;

                case USB_REQ_SET_INTERFACE:
                    break;

                case USB_REQ_CLEAR_FEATURE:
#if !CDC_ONLY_NO_MSC
                    if ((wIndex_iface == MSC_EPIN_ADDR) || (wIndex_iface == MSC_EPOUT_ADDR))
                    {
                        pdev->pClassData = &composite.msc;
                        pdev->pClassDataCmsit[0] = &composite.msc;
                        MSC_BOT_CplClrFeature(pdev, LOBYTE(req->wValue));
                    }
#endif
                    break;

                default:
                    USBD_CtlError(pdev, req);
                    break;
            }
            break;

        default:
            USBD_CtlError(pdev, req);
            break;
    }
    return USBD_OK;
}

/* ═══════════════════════════════════════════════════════════════ */
/* COMPOSITE_EP0_RxReady  (called after CDC SET_LINE_CODING data)  */
/* ═══════════════════════════════════════════════════════════════ */
static uint8_t COMPOSITE_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
    if (composite.cdc.CmdOpCode != 0xFFU)
    {
        CDC_Control_FS(composite.cdc.CmdOpCode,
                       (uint8_t *)composite.cdc.data,
                       composite.cdc.CmdLength);
        composite.cdc.CmdOpCode = 0xFFU;
    }
    return USBD_OK;
}

/* ═══════════════════════════════════════════════════════════════ */
/* COMPOSITE_DataIn                                                 */
/* ═══════════════════════════════════════════════════════════════ */
static uint8_t COMPOSITE_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
#if !CDC_ONLY_NO_MSC
    if (epnum == (MSC_EPIN_ADDR & 0x7FU))       /* EP1 → MSC */
    {
        pdev->pClassData = &composite.msc;
        pdev->pClassDataCmsit[0] = &composite.msc;
        MSC_BOT_DataIn(pdev, epnum);
    }
    else
#endif
    if (epnum == (CDC_IN_EP & 0x7FU))       /* EP2 → CDC TX done */
    {
        composite.cdc.TxState = 0U;
        CDC_TransmitCplt_FS(composite.cdc.TxBuffer,
                             &composite.cdc.TxLength, epnum);
    }
    return USBD_OK;
}

/* ═══════════════════════════════════════════════════════════════ */
/* COMPOSITE_DataOut                                                */
/* ═══════════════════════════════════════════════════════════════ */
static uint8_t COMPOSITE_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
#if !CDC_ONLY_NO_MSC
    if (epnum == MSC_EPOUT_ADDR)                 /* EP1 → MSC */
    {
        pdev->pClassData = &composite.msc;
        pdev->pClassDataCmsit[0] = &composite.msc;
        MSC_BOT_DataOut(pdev, epnum);
    }
    else
#endif
    if (epnum == CDC_OUT_EP)                /* EP2 → CDC RX data */
    {
        uint32_t len = USBD_LL_GetRxDataSize(pdev, epnum);
        CDC_Receive_FS(composite.cdc.RxBuffer, &len);

        /* Re-arm receive */
        USBD_LL_PrepareReceive(pdev, CDC_OUT_EP,
                               composite.cdc.RxBuffer,
                               CDC_DATA_FS_MAX_PACKET_SIZE);
    }
    return USBD_OK;
}

/* ═══════════════════════════════════════════════════════════════ */
/* Descriptor getters                                               */
/* ═══════════════════════════════════════════════════════════════ */
static uint8_t *COMPOSITE_GetCfgDesc(uint16_t *length)
{
    *length = (uint16_t)sizeof(USBD_Composite_CfgDesc);
    return USBD_Composite_CfgDesc;
}

static uint8_t *COMPOSITE_GetDeviceQualifierDesc(uint16_t *length)
{
    *length = (uint16_t)sizeof(USBD_Composite_DeviceQualifierDesc);
    return USBD_Composite_DeviceQualifierDesc;
}
