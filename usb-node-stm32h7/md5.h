/* MD5 (RFC 1321), for the ESP32 ROM loader's SPI_FLASH_MD5. */
#ifndef MD5_H
#define MD5_H
#include <stdint.h>
void md5(const uint8_t *data, uint32_t len, uint8_t out[16]);
#endif
