/*
 * SPI2 master for an SPI sensor (on the bench the pod emulates it on LA7-LA10):
 * PB13 SCK, PB15 MOSI, PB14 MISO (pulled up, so a released line reads 0xFF), PB12 CS
 * (software, active low). SPI1 belongs to the MCP2515 and is not touched here.
 */
#ifndef SPI2_H
#define SPI2_H

#include <stdint.h>

#define SPI2_MAX_BYTES 96U
#define SPI2_DEFAULT_HZ 500000U

void spi2_init(void);
/* Mode 0..3 and the fastest clock at or below hz (16 MHz / 2..256). Returns the clock it
 * runs at, 0 when the request is out of range (mode > 3, or below 62.5 kHz). */
uint32_t spi2_config(uint8_t mode, uint32_t hz);
uint8_t spi2_mode(void);
uint32_t spi2_hz(void);
/* One transfer of n bytes with CS low for all of them. */
void spi2_xfer(const uint8_t *tx, uint8_t *rx, uint16_t n);

#endif
