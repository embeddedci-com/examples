/* TinyUSB configuration: full-speed device on OTG_FS (PA11/PA12, TinyUSB rhport 0), bare metal. */
#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#define CFG_TUSB_MCU             OPT_MCU_STM32H7
#define CFG_TUSB_OS              OPT_OS_NONE
#define CFG_TUSB_RHPORT0_MODE    (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUD_ENDPOINT0_SIZE   64

#define CFG_TUD_CDC              1
#define CFG_TUD_MSC              1
#define CFG_TUD_HID              1

#define CFG_TUD_CDC_RX_BUFSIZE   512
#define CFG_TUD_CDC_TX_BUFSIZE   512
#define CFG_TUD_CDC_EP_BUFSIZE   64

#define CFG_TUD_MSC_EP_BUFSIZE   4096

/* The Teensy loader takes a 1024-byte block plus a 64-byte header in one SET_REPORT. */
#define CFG_TUD_HID_EP_BUFSIZE   1088

#endif
