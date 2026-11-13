/*
 * USART3 for a UART sensor (on the bench the pod emulates it): PC11 RX <- pod LA13,
 * PC10 TX -> pod LA14. Closed at boot (both pins inputs); `uart3 open <baud>` starts it.
 * Received bytes go to a 1 KB ring by interrupt; the console commands read it, or the NMEA
 * mode prints each complete sentence with its checksum verdict.
 */
#ifndef UART3_H
#define UART3_H

#include <stdint.h>

#define UART3_RING_SIZE 1024U

typedef struct
{
    uint8_t open;
    uint32_t baud;
    uint32_t rx_total;   /* bytes received since open */
    uint32_t tx_total;
    uint32_t framing;    /* framing errors (wrong baud, a break, garbage) */
    uint32_t noise;
    uint32_t overrun;    /* the hardware lost a byte */
    uint32_t dropped;    /* the ring was full */
} uart3_stats_t;

/* 1200 .. 1000000 baud, 8N1. -1 out of range. */
int uart3_open(uint32_t baud);
void uart3_close(void);
const uart3_stats_t *uart3_stats(void);
void uart3_write(const uint8_t *b, uint16_t n);
int uart3_getc(uint8_t *b);
uint16_t uart3_pending(void);
void uart3_flush(void);

#endif
